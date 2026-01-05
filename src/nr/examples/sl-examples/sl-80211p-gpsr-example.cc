/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only AND NIST-Software

/**
 * \ingroup examples
 * \file sl-80211p-gpsr-example.cc
 *
 * 802.11p WiFi example with GPSR routing protocol integration.
 *
 * \code{.unparsed}
$ ./ns3 run "sl-multi-lc-example --help"
    \endcode
 *
 */

#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/gpsr-helper.h"
#include "ns3/gpsr.h"
#include "ns3/gpsr-ptable.h"
#include "ns3/stats-module.h"
#include "ns3/traci-module.h"
#include "ns3/ipv4-list-routing.h"
// 802.11p/WiFi includes
#include "ns3/wifi-module.h"
#include "ns3/propagation-module.h"
#include "ns3/yans-wifi-helper.h"

#include <iomanip>
#include <ostream>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SlWifiGpsrExample");

uint32_t g_rxPktCounter = 0;            //!< Global variable to count RX packets
uint32_t g_txPktCounter = 0;            //!< Global variable to count TX packets
uint32_t g_rxPktDelivered = 0;          //!< Packets matched with TX record (successful delivery)
uint64_t g_rxPayloadBytes = 0;          //!< Payload bytes of successfully delivered packets
std::list<double> g_delays;             //!< Global list to store packet delays upon RX

// Global traffic parameters for CLI - VANET traffic pattern
uint32_t g_udpPacketSize = 412;         //!< Packet size (payload 400 + SeqTsSizeHeader ~12)
double g_dataRateKbps = 100.0;          //!< Data rate: ~25 packets/sec at 400B

// ========== Distance-based traffic generation ==========
std::vector<uint32_t> g_activeNodeIds;  //!< Track active node IDs from TraCI
NodeContainer* g_ueNodeContainerPtr = nullptr;  //!< Pointer to UE node container for traffic gen
double g_minDistanceForTraffic = 250.0;  //!< Minimum distance (m) for multi-hop traffic
uint32_t g_flowId = 0;                  //!< Global flow counter for unique port assignment
uint16_t g_basePort = 10000;            //!< Base port for flow-specific PacketSink

/*
 * \brief Trace sink function to count and logging the transmitted data packets
 *        and their corresponding transmission timestamp at the application layer
 *
 * \param p the packet
 * \param srcAddrs the source IP address in the packet
 * \param dstAddrs the destination IP address in the packet
 * \param seqTsSizeHeader the header containing the sequence number of the packet
 */
void
TxPacketTraceForDelay(Ptr<const Packet> p,
                      const Address& srcAddrs,
                      const Address& dstAddrs,
                      const SeqTsSizeHeader& seqTsSizeHeader)
{
    g_txPktCounter++;
    NS_LOG_DEBUG("TX: seq=" << seqTsSizeHeader.GetSeq());
}

/*
 * \brief Trace sink function to count and calculate the delay upon reception
 *        of a packet at the application layer
 *
 * \param p the packet
 * \param srcAddrs the source IP address in the packet
 * \param dstAddrs the destination IP address in the packet
 * \param seqTsSizeHeader the header containing the sequence number of the packet
 */
void
RxPacketTraceForDelay(Ptr<const Packet> p,
                      const Address& srcAddrs,
                      const Address& dstAddrs,
                      const SeqTsSizeHeader& seqTsSizeHeader)
{
    g_rxPktCounter++;
    g_rxPktDelivered++;
    g_rxPayloadBytes += p->GetSize();

    // 直接使用 SeqTsSizeHeader 内置的发送时间戳计算延迟
    Time txTime = seqTsSizeHeader.GetTs();
    double delay = (Simulator::Now() - txTime).GetMilliSeconds();
    g_delays.push_back(delay);

    NS_LOG_DEBUG("RX: seq=" << seqTsSizeHeader.GetSeq() << " delay=" << delay << "ms");
}

// NR-specific declarations removed for 802.11p version

/**
 * \brief Generate traffic between the most distant active node pair
 * 
 * This function finds the pair of active nodes with maximum geographic distance
 * and initiates a data flow between them to test GPSR multi-hop routing.
 */
void GenerateDistantTraffic()
{
    if (g_activeNodeIds.size() < 2 || g_ueNodeContainerPtr == nullptr)
    {
        // Not enough active nodes, try again later
        Simulator::Schedule(Seconds(2.0), &GenerateDistantTraffic);
        return;
    }

    // Collect all pairs with distance > threshold
    std::vector<std::tuple<uint32_t, uint32_t, double>> validPairs;
    
    for (size_t i = 0; i < g_activeNodeIds.size(); ++i)
    {
        for (size_t j = i + 1; j < g_activeNodeIds.size(); ++j)
        {
            Ptr<Node> n1 = g_ueNodeContainerPtr->Get(g_activeNodeIds[i]);
            Ptr<Node> n2 = g_ueNodeContainerPtr->Get(g_activeNodeIds[j]);
            if (!n1 || !n2) continue;
            
            Ptr<MobilityModel> mob1 = n1->GetObject<MobilityModel>();
            Ptr<MobilityModel> mob2 = n2->GetObject<MobilityModel>();
            if (!mob1 || !mob2) continue;
            
            Vector p1 = mob1->GetPosition();
            Vector p2 = mob2->GetPosition();
            double dist = CalculateDistance(p1, p2);
            
            // Collect pairs with distance > threshold
            if (dist >= g_minDistanceForTraffic)
            {
                validPairs.push_back(std::make_tuple(g_activeNodeIds[i], g_activeNodeIds[j], dist));
            }
        }
    }

    // Create multiple concurrent flows (5 flows per scheduling)
    uint32_t numFlows = std::min((size_t)5, validPairs.size());  // 5 concurrent flows
    for (uint32_t f = 0; f < numFlows; ++f)
    {
        // Randomly select a pair from remaining valid pairs
        size_t randomIdx = rand() % validPairs.size();
        uint32_t srcNodeId = std::get<0>(validPairs[randomIdx]);
        uint32_t dstNodeId = std::get<1>(validPairs[randomIdx]);
        double selectedDist = std::get<2>(validPairs[randomIdx]);
        
        // Remove selected pair to avoid duplicates
        validPairs.erase(validPairs.begin() + randomIdx);
        
        // Calculate destination IP: 7.0.0.(nodeId + 2)
        uint32_t ipOffset = dstNodeId + 2;
        std::ostringstream ipStream;
        ipStream << "7.0." << ((ipOffset >> 8) & 0xFF) << "." << (ipOffset & 0xFF);
        Ipv4Address dstIp(ipStream.str().c_str());
        
        // Assign unique port for this flow
        uint16_t port = g_basePort + g_flowId++;
        if (port > 65500) {
            NS_LOG_WARN("Port range exhausted, stopping traffic generation");
            return;
        }
        
        // Install PacketSink on destination node for this specific port
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", 
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        sinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer sinkApp = sinkHelper.Install(g_ueNodeContainerPtr->Get(dstNodeId));
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(Seconds(6.0));  // Slightly longer than OnOff to catch all packets
        // Connect RX trace for this sink
        sinkApp.Get(0)->TraceConnectWithoutContext("RxWithSeqTsSize",
                                                   MakeCallback(&RxPacketTraceForDelay));
        
        // Create and install OnOffApplication
        OnOffHelper onoff("ns3::UdpSocketFactory", 
                          InetSocketAddress(dstIp, port));
        onoff.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        std::string rateStr = std::to_string(g_dataRateKbps) + "kb/s";
        onoff.SetConstantRate(DataRate(rateStr), g_udpPacketSize);
        
        ApplicationContainer app = onoff.Install(g_ueNodeContainerPtr->Get(srcNodeId));
        app.Start(Seconds(0.0));
        app.Stop(Seconds(5.0));  // VANET: continuous BSM-like flow for 5 seconds
        
        // Connect TX trace for delay calculation
        app.Get(0)->TraceConnectWithoutContext("TxWithSeqTsSize",
                                               MakeCallback(&TxPacketTraceForDelay));
        
        NS_LOG_INFO("Multi-hop Traffic: Node " << srcNodeId << " -> Node " << dstNodeId 
                    << " (IP:" << dstIp << ", port=" << port << ", dist=" << selectedDist << "m, flow " 
                    << (f+1) << "/" << numFlows << ")");
    }
    
    if (numFlows == 0)
    {
        NS_LOG_DEBUG("No distant pair found (threshold=" << g_minDistanceForTraffic << "m)");
    }

    // Schedule next traffic generation - VANET: 1 second interval
    Simulator::Schedule(Seconds(1.0), &GenerateDistantTraffic);
}

// ========== 802.11p WiFi SNR Tracking ==========
// Mac48Address to Ipv4Address mapping for WiFi SNR updates
std::map<Mac48Address, Ipv4Address> g_macToIpMap;

/**
 * \brief Trace callback for WiFi PHY reception - updates GPSR neighbor SNR
 * 
 * This callback is connected to MonitorSnifferRx trace of WifiPhy.
 * It extracts sender MAC, looks up IP, and updates GPSR position table.
 */
void
NotifyWifiPhyRx(Ptr<ns3::gpsr::RoutingProtocol> gpsr, 
                Ptr<const Packet> packet, 
                uint16_t channelFreqMhz,
                WifiTxVector txVector,
                MpduInfo aMpdu,
                SignalNoiseDbm signalNoise,
                uint16_t staId)
{
    // Extract source MAC from packet
    WifiMacHeader hdr;
    Ptr<Packet> copy = packet->Copy();
    if (copy->PeekHeader(hdr) == 0)
    {
        return;  // Not a valid WiFi frame
    }
    
    Mac48Address srcMac = hdr.GetAddr2();  // Transmitter address
    
    // Look up IP from MAC mapping
    auto it = g_macToIpMap.find(srcMac);
    if (it == g_macToIpMap.end())
    {
        return;  // Unknown MAC, skip
    }
    
    Ipv4Address neighborIp = it->second;
    
    // Calculate SNR = signal - noise (in dB), then convert to linear
    double snrDb = signalNoise.signal - signalNoise.noise;
    double snrLinear = std::pow(10.0, snrDb / 10.0);
    
    NS_LOG_DEBUG("WiFi-RX: srcMac=" << srcMac 
                 << " ip=" << neighborIp
                 << " signal=" << signalNoise.signal << "dBm"
                 << " noise=" << signalNoise.noise << "dBm"
                 << " snr=" << snrDb << "dB");
    
    // Update SINR in GPSR position table
    gpsr->GetPositionTable()->UpdateSinr(neighborIp, snrLinear);
}

int
main(int argc, char* argv[])
{
    // Scenario parameters
    uint16_t ueNum = 400;  // Max concurrent vehicles from SUMO
    
    // Traffic parameters (legacy - for compatibility)
    uint32_t udpPacketSize = 200;
    double dataRate = 8; // 8 kilobits per second

    // Simulation parameters
    Time trafficTime = Seconds(2.0);

    // Testing flag
    bool testing = false;

    // 802.11p parameters
    double txPower = 17;  // dBm (~400m range with TwoRayGround)

    CommandLine cmd(__FILE__);
    cmd.AddValue("trafficTime", "The time traffic will be active in seconds", trafficTime);
    cmd.AddValue("packetSize", "packet size in bytes to be used by best effort traffic", udpPacketSize);
    cmd.AddValue("dataRate", "The data rate in kilobits per second for best effort traffic", dataRate);
    cmd.AddValue("testing", "Testing flag for verification", testing);

    // Parse the command line
    cmd.Parse(argc, argv);
    
    // Sync CLI params to global variables for traffic generation
    g_udpPacketSize = udpPacketSize;
    g_dataRateKbps = dataRate;

    // Final simulation time
    Time startupTime = Seconds(2.0);  // Time for SUMO/TraCI to start
    Time finalSimTime = trafficTime + startupTime + Seconds(0.05);

    // Enable GPSR debug logging
    //LogComponentEnable("GpsrPositionTable", LOG_LEVEL_DEBUG);  // Disabled to reduce SINR logs
    LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_INFO);

    // Create UE node pool - pre-allocate nodes for SUMO vehicles
    NodeContainer ueNodeContainer;
    ueNodeContainer.Create(ueNum);

    // Setup initial mobility with velocity support
    // Using ConstantVelocityMobilityModel allows TraCI to update velocity from SUMO
    // Nodes will be moved to correct positions by TraCI when vehicles depart
    MobilityHelper initialMobility;
    initialMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    Ptr<GridPositionAllocator> gridAlloc = CreateObject<GridPositionAllocator>();
    gridAlloc->SetAttribute("MinX", DoubleValue(10000.0));  // 10km away from simulation
    gridAlloc->SetAttribute("MinY", DoubleValue(10000.0));
    gridAlloc->SetAttribute("DeltaX", DoubleValue(500.0));  // 500m spacing (beyond comm range)
    gridAlloc->SetAttribute("DeltaY", DoubleValue(500.0));
    gridAlloc->SetAttribute("GridWidth", UintegerValue(20));  // 20x20 grid
    gridAlloc->SetAttribute("LayoutType", StringValue("RowFirst"));
    initialMobility.SetPositionAllocator(gridAlloc);
    initialMobility.Install(ueNodeContainer);

    // Setup TraCI client - will be configured later after NR stack installation
    Ptr<TraciClient> sumoClient = CreateObject<TraciClient>();
    sumoClient->SetAttribute("SumoConfigPath", StringValue("/home/lyh/ns3-nr-v2x/ns-3-dev/src/nr/examples/grid_network_fed/grid.sumocfg"));
    sumoClient->SetAttribute("SumoBinaryPath", StringValue(""));  // Use system SUMO
    sumoClient->SetAttribute("SynchInterval", TimeValue(Seconds(0.1)));
    sumoClient->SetAttribute("StartTime", TimeValue(Seconds(0.0)));
    sumoClient->SetAttribute("SumoGUI", BooleanValue(false));
    sumoClient->SetAttribute("SumoPort", UintegerValue(3400));
    sumoClient->SetAttribute("PenetrationRate", DoubleValue(1.0));
    sumoClient->SetAttribute("SumoLogFile", BooleanValue(true));
    sumoClient->SetAttribute("SumoStepLog", BooleanValue(false));
    sumoClient->SetAttribute("SumoSeed", IntegerValue(10));
    sumoClient->SetAttribute("SumoWaitForSocket", TimeValue(Seconds(1.0)));

    /* Multi-hop topology for GPSR demo:
     *
     *   UE0..........(20 m)..........UE1..........(20 m)..........UE2
     * (0, 0, 1.5)               (20, 0, 1.5)               (40, 0, 1.5)
     *
     * Traffic: UE0 -> UE2 (requires GPSR multi-hop via UE1)
     */
    Packet::EnableChecking();
    Packet::EnablePrinting();

    // Enable detailed logging for debugging GPSR + 802.11p integration
    std::string logDir = "/home/lyh/ns3-nr-v2x/ns-3-dev/src/nr/examples/sl-examples/";
    std::ofstream logFile(logDir + "sl-80211p-gpsr-debug.log");
    if (logFile.is_open())
    {
        NS_LOG_INFO("Debug log file opened: " << logDir << "sl-80211p-gpsr-debug.log");
    }
    
    // Logging configuration - simplified for production
    LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_INFO);    // Forwarding-related logs only
    //LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_DEBUG);   // DEBUG for LocalDelivery
    //LogComponentEnable("GpsrPacket", LOG_LEVEL_DEBUG);             // DEBUG for Deserialize
    //LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_WARN);     // Only warnings/errors
    //LogComponentEnable("SlWifiGpsrExample", LOG_LEVEL_DEBUG);    // Very verbose
    LogComponentEnable("SlWifiGpsrExample", LOG_LEVEL_INFO);       // Key events only
    //LogComponentEnable("PacketSink", LOG_LEVEL_INFO);              // Diagnostic for crash
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_NODE);
    //LogComponentEnableAll(LOG_PREFIX_FUNC);  // Disabled to reduce log size
    
    // Redirect log output to file (clog for NS_LOG, cerr for std::cerr markers)
    std::clog.rdbuf(logFile.rdbuf());
    std::cerr.rdbuf(logFile.rdbuf());
    NS_LOG_INFO("=== SL-80211p-GPSR Example Debug Log Started ===");
    std::cerr << "[CERR-TEST] std::cerr redirect test from main()" << std::endl;
    /************************* 802.11p WiFi Configuration *************************/
    // 802.11p channel and propagation model
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    // Friis model (~1km range, ideal free space) - commented out
    //wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel",
    //                                "Frequency", DoubleValue(5.9e9));
    // TwoRayGround model (~300m range, more realistic for V2X)
    wifiChannel.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel",
                                    "Frequency", DoubleValue(5.9e9),
                                    "HeightAboveZ", DoubleValue(1.5));  // Vehicle antenna height
    
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());
    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    wifiPhy.Set("TxPowerStart", DoubleValue(txPower));
    wifiPhy.Set("TxPowerEnd", DoubleValue(txPower));
    // Set 802.11p channel: 5890MHz, 10MHz bandwidth, Band 5GHz
    wifiPhy.Set("ChannelSettings", StringValue("{178, 10, BAND_5GHZ, 0}"));
    
    // 802.11p WiFi with Ad-hoc MAC (no wave module needed)
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211p);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue("OfdmRate6MbpsBW10MHz"),
                                  "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"),
                                  "NonUnicastMode", StringValue("OfdmRate6MbpsBW10MHz"));
    
    // Ad-hoc MAC for V2X broadcast communication
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    
    // Install 802.11p devices
    NetDeviceContainer ueNetDev = wifi.Install(wifiPhy, wifiMac, ueNodeContainer);
    
    // Fix random streams
    int64_t stream = 1;
    stream += wifi.AssignStreams(ueNetDev, stream);

    // Configure internet with GPSR routing
    GpsrHelper gpsr;
    InternetStackHelper internet;
    internet.SetRoutingHelper(gpsr);
    internet.Install(ueNodeContainer);
    stream += internet.AssignStreams(ueNodeContainer, stream);

    // Wire up GPSR m_downTarget callback chain for header insertion
    // This must be called AFTER InternetStackHelper.Install()
    gpsr.Install(ueNodeContainer);
    NS_LOG_INFO("GPSR routing protocol installed on all UEs");

    // ========== Build Mac48Address to IPv4 mapping + Connect WiFi SNR traces ==========
    NS_LOG_INFO("Building MAC to IP mapping and connecting WiFi SNR traces...");
    for (uint32_t i = 0; i < ueNodeContainer.GetN(); ++i)
    {
        Ptr<Node> node = ueNodeContainer.Get(i);
        Ptr<NetDevice> device = ueNetDev.Get(i);
        
        // Get MAC address from WiFi device
        Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(device);
        if (!wifiDevice)
        {
            NS_LOG_WARN("Node " << i << " has no WifiNetDevice, skipping");
            continue;
        }
        Mac48Address mac = wifiDevice->GetMac()->GetAddress();
        
        // Calculate IP address: 7.0.0.(nodeId + 2)
        uint32_t ipOffset = i + 2;
        std::ostringstream ipStream;
        ipStream << "7.0." << ((ipOffset >> 8) & 0xFF) << "." << (ipOffset & 0xFF);
        Ipv4Address nodeIp(ipStream.str().c_str());
        
        // Add to global mapping
        g_macToIpMap[mac] = nodeIp;
        NS_LOG_DEBUG("MAC->IP mapping: " << mac << " -> " << nodeIp);
        
        // Find GPSR routing protocol
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        if (!ipv4) continue;
        
        Ptr<Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
        Ptr<ns3::gpsr::RoutingProtocol> gpsrProto;
        
        Ptr<Ipv4ListRouting> listRouting = DynamicCast<Ipv4ListRouting>(rp);
        if (listRouting)
        {
            for (uint32_t j = 0; j < listRouting->GetNRoutingProtocols(); ++j)
            {
                int16_t priority;
                Ptr<Ipv4RoutingProtocol> proto = listRouting->GetRoutingProtocol(j, priority);
                gpsrProto = DynamicCast<ns3::gpsr::RoutingProtocol>(proto);
                if (gpsrProto) break;
            }
        }
        else
        {
            gpsrProto = DynamicCast<ns3::gpsr::RoutingProtocol>(rp);
        }
        
        if (!gpsrProto)
        {
            NS_LOG_WARN("Node " << i << " has no GPSR routing protocol");
            continue;
        }
        
        // Connect WiFi PHY MonitorSnifferRx trace for SNR updates
        Ptr<WifiPhy> wifiPhy = wifiDevice->GetPhy();
        if (wifiPhy)
        {
            wifiPhy->TraceConnectWithoutContext(
                "MonitorSnifferRx",
                MakeBoundCallback(&NotifyWifiPhyRx, gpsrProto));
            NS_LOG_DEBUG("Node " << i << " WiFi SNR trace connected");
        }
    }
    NS_LOG_INFO("WiFi MAC->IP mapping and SNR trace binding complete");

    /************************** IP Address Assignment (802.11p) ********************/
    // Assign IP addresses: 7.0.0.(nodeId + 2), i.e., Node 0 -> 7.0.0.2, Node 1 -> 7.0.0.3
    Ipv4AddressHelper ipv4Helper;
    ipv4Helper.SetBase("7.0.0.0", "255.255.0.0", "0.0.0.2");  // Start from 7.0.0.2
    Ipv4InterfaceContainer ueIpIface = ipv4Helper.Assign(ueNetDev);
    
    NS_LOG_DEBUG("Device 0 has address " << ueIpIface.GetAddress(0)); // 7.0.0.2
    NS_LOG_DEBUG("Device 1 has address " << ueIpIface.GetAddress(1)); // 7.0.0.3
    
    // Note: No TFT/SidelinkInfo needed for 802.11p - WiFi uses ARP for L2 resolution

    /*
     * Configure the applications:
     * Using GenerateDistantTraffic for distance-based multi-hop testing
     */
    ApplicationContainer allClientApps;
    ApplicationContainer allServerApps;

    // ========== Global PacketSink removed - using per-flow PacketSink in GenerateDistantTraffic ==========
    // ApplicationContainer globalServerApps;
    // uint16_t globalPort = 9001;
    // ... (removed to avoid unused RX triggers)
    
    /************************ END Traffic flows configuration ******************/

    /******************** Application packet tracing ***************************/
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> PacketTraceForDelayStream =
        ascii.CreateFileStream("NrSlAppRxPacketDelayTrace.txt");
    *PacketTraceForDelayStream->GetStream()
        << "time(s)\trxNodeId\tsrcIp\tdstIp\tseqNum\tdelay(ms)" << std::endl;

    for (uint16_t ac = 0; ac < allClientApps.GetN(); ac++)
    {
        allClientApps.Get(ac)->TraceConnectWithoutContext("TxWithSeqTsSize",
                                                          MakeCallback(&TxPacketTraceForDelay));
    }
    for (uint16_t ac = 0; ac < allServerApps.GetN(); ac++)
    {
        allServerApps.Get(ac)->TraceConnectWithoutContext("RxWithSeqTsSize",
                                                          MakeCallback(&RxPacketTraceForDelay));
    }
    // Global PacketSink trace connection removed - per-flow sinks connected in GenerateDistantTraffic
    /******************** END Application packet  tracing **********************/

    // NR-specific grant tracing removed for 802.11p

    // Setup TraCI callbacks for node creation/destruction
    // Callback receives vehicle ID (e.g., "car10") and returns the corresponding node
    std::function<Ptr<Node> (const std::string&)> setupNewNode = [&] (const std::string& vehicleId) -> Ptr<Node>
    {
        // Parse numeric ID from vehicle ID (e.g., "car10" -> 10)
        std::string numStr = vehicleId.substr(3);  // Skip "car" prefix
        uint32_t nodeId = std::stoul(numStr);
        
        if (nodeId >= ueNodeContainer.GetN())
            NS_FATAL_ERROR("Vehicle ID " << vehicleId << " exceeds node pool size " << ueNodeContainer.GetN());

        // Get pre-configured node matching the vehicle ID
        Ptr<Node> includedNode = ueNodeContainer.Get(nodeId);
        
        // Track active node for distance-based traffic generation
        g_activeNodeIds.push_back(nodeId);
        
        NS_LOG_INFO("TraCI: Vehicle " << vehicleId << " entered, using node " << nodeId 
                    << " (active nodes: " << g_activeNodeIds.size() << ")");
        return includedNode;
    };

    // Callback for node shutdown (vehicle leaves)
    std::function<void (Ptr<Node>)> shutdownNode = [] (Ptr<Node> exNode)
    {
        // Remove node from active tracking
        uint32_t nodeId = exNode->GetId();
        auto it = std::find(g_activeNodeIds.begin(), g_activeNodeIds.end(), nodeId);
        if (it != g_activeNodeIds.end())
        {
            g_activeNodeIds.erase(it);
        }
        
        // Move node far away from simulation area
        Ptr<ConstantVelocityMobilityModel> mob = exNode->GetObject<ConstantVelocityMobilityModel>();
        if (mob)
        {
            mob->SetPosition(Vector(10000.0 + (rand() % 100), 10000.0 + (rand() % 100), 0.0));
            mob->SetVelocity(Vector(0.0, 0.0, 0.0));  // Stop the node
        }
        NS_LOG_INFO("TraCI: Vehicle left, node " << nodeId << " moved to parking (active: " 
                    << g_activeNodeIds.size() << ")");
    };

    // Set global pointer for traffic generation
    g_ueNodeContainerPtr = &ueNodeContainer;
    
    // Start TraCI client and connect to SUMO
    sumoClient->SumoSetup(setupNewNode, shutdownNode);
    NS_LOG_INFO("TraCI connected to SUMO, synchronizing vehicle positions");

    // Schedule distance-based traffic generation (start after vehicles enter)
    Simulator::Schedule(Seconds(20.0), &GenerateDistantTraffic);
    NS_LOG_INFO("Scheduled distance-based traffic generation (minDist=" << g_minDistanceForTraffic << "m, start at 20s)");

    Simulator::Stop(finalSimTime);
    Simulator::Run();

    // Print 802.11p simulation results
    std::cout << "=== 802.11p GPSR Simulation Results ===" << std::endl;
    std::cout << "Total Tx packets = " << g_txPktCounter << std::endl;
    std::cout << "Total Rx packets = " << g_rxPktCounter << std::endl;
    if (!g_delays.empty())
    {
        double delaySum = 0;
        for (auto it = g_delays.begin(); it != g_delays.end(); it++)
        {
            delaySum += *it;
        }
        std::cout << "Average packet delay = " << delaySum / g_delays.size() << " ms" << std::endl;
    }
    
    // Control overhead statistics
    uint64_t ctrlBytes = 0;
    uint64_t ctrlPkts = 0;
    for (uint32_t i = 0; i < ueNodeContainer.GetN(); i++)
    {
        Ptr<Ipv4> ipv4 = ueNodeContainer.Get(i)->GetObject<Ipv4>();
        if (!ipv4) continue;
        Ptr<Ipv4RoutingProtocol> routing = ipv4->GetRoutingProtocol();
        Ptr<Ipv4ListRouting> listRouting = DynamicCast<Ipv4ListRouting>(routing);
        if (listRouting)
        {
            for (uint32_t j = 0; j < listRouting->GetNRoutingProtocols(); j++)
            {
                int16_t priority;
                Ptr<Ipv4RoutingProtocol> proto = listRouting->GetRoutingProtocol(j, priority);
                Ptr<ns3::gpsr::RoutingProtocol> gpsr = DynamicCast<ns3::gpsr::RoutingProtocol>(proto);
                if (gpsr)
                {
                    ctrlBytes += gpsr->GetCtrlHelloTxBytes();
                    ctrlPkts += gpsr->GetCtrlHelloTxPkts();
                }
            }
        }
        else
        {
            // Non-ListRouting scenario: try direct cast
            Ptr<ns3::gpsr::RoutingProtocol> gpsr = DynamicCast<ns3::gpsr::RoutingProtocol>(routing);
            if (gpsr)
            {
                ctrlBytes += gpsr->GetCtrlHelloTxBytes();
                ctrlPkts += gpsr->GetCtrlHelloTxPkts();
            }
        }
    }
    std::cout << "Total Rx packets (delivered) = " << g_rxPktDelivered << std::endl;
    std::cout << "Total Rx payload bytes = " << g_rxPayloadBytes << std::endl;
    std::cout << "Total HELLO ctrl bytes = " << ctrlBytes << std::endl;
    std::cout << "Total HELLO ctrl pkts = " << ctrlPkts << std::endl;
    if (g_rxPayloadBytes > 0)
    {
        std::cout << "Control overhead (bytes) = " << (double)ctrlBytes / g_rxPayloadBytes << std::endl;
    }
    if (g_rxPktDelivered > 0)
    {
        std::cout << "NRL (pkts) = " << (double)ctrlPkts / g_rxPktDelivered << std::endl;
    }

    Simulator::Destroy();
    if (testing)
    {
        if (g_rxPktCounter < 0.95 * g_txPktCounter)
        {
            std::cout << "Not enough packets received" << std::endl;
            return 1;
        }
    }
    return 0;
}

// NR-specific TraceGrantCreated, WriteGrantCreated, and GetPacketSize removed for 802.11p version
