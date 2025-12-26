/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only AND NIST-Software

/**
 * \ingroup examples
 * \file sl-nr-gpsr-example.cc
 *
 * NR Sidelink example with GPSR routing protocol integration.
 *
 * This example is based on cttc-nr-v2x-demo-simple and simulates a simple
 * topology consisting of 2 out-of-coverage UEs, where UE-1 transmits, and UE-2
 * receives.
 * The purpose of this scenario is to show how different sidelink (SL) traffic
 * profiles can be configured in the simulation and associated to different
 * traffic flows. The scheduler then will take into consideration those traffic
 * profiles and grant resources accordingly during the simulation.
 *
 * If interested in learning how to configure the other aspects of a NR SL
 * simulation please refer to cttc-nr-v2x-demo-simple for a nice tutorial.
 *
 * Traffic generation:
 * UE-1 has three traffic flows directed towards UE-2. Each traffic flow is
 * implemented by an OnOffApplication with constant rate that generates CBR
 * traffic for the duration of the 'trafficTime' simulation parameter. The
 * packet size and the data rate can be adjusted with the simulation
 * parameters 'udpPacketSize' and 'dataRate', respectively.
 * Please note that the traffic profiles described below were selected to
 * support the default traffic configuration of 200 bytes packets and a data
 * rate of 16 kbps. Changing those values without adjusting the traffic profile
 * accordingly may result in poor performance.
 * All traffic flows are sent to the multicast address, but each of them to a
 * different port. UE-2 listens to that address and those ports and consumes
 * the packets with a PacketSink application
 *
 *
 * Traffic profiles:
 * Three different traffic profiles are created, one for each traffic flow.
 * The structure SidelinkInfo contains the traffic profile parameters to be
 * propagated in the stack. A SidelinkInfo is created for each traffic flow.
 * A Traffic Flow Template (TFT) is created for each traffic flow associating
 * the corresponding SidelinkInfo, multicast address and port. Three SL data
 * radio bearer are created, each configured with one of the above described
 * TFTs, which will result in the creation of a logical channel per traffic
 * flow on the stack. The scheduler (NrSlUeMacSchedulerDefault) will grant
 * resources for each flow according to the traffic profiles.
 *
 * In this scenario we configure four parameters of the traffic profiles:
 *  - m_dynamic: flag for whether the logical channel scheduling is dynamic
 *               scheduling (i.e., per PDU) or Semi-persistent scheduling (SPS)
 *  - m_dstL2Id: L2 id to which the traffic will be sent
 *  - m_priority: priority (the higher the value, the higher the priority of
 *                the logical channel)
 *  - m_rri: Resource Reservation Interval for SPS scheduling
 *
 *  We created different configurations for the three traffic profiles. The
 *  table below shows the simulation parameters that control which
 *  configuration is used and the resulting values for the traffic profile
 *  parameters of each traffic flow.
 *
 *  -------------------------|-----------------------------
 * |      Simulation         | Resulting traffic profile   |
 * |    configuration        | configuration per flow      |
 * |-------------------------|-----------------------------|
 * | Parameter       | Value | Flow 1  | Flow 2  | Flow 3  |
 * |-----------------|-------|---------|---------|---------|
 * | schedTypeConfig |   1   | Dynamic | Dynamic | Dynamic |
 * |-----------------|-------|---------|---------|---------|
 * | schedTypeConfig |   2   |   SPS   |   SPS   |   SPS   |
 * |-----------------|-------|---------|---------|---------|
 * | schedTypeConfig |   3   | Dynamic | Dynamic |   SPS   |
 * |-----------------|-------|---------|---------|---------|
 * | schedTypeConfig |   4   |   SPS   |   SPS   | Dynamic |
 * |-----------------|-------|---------|---------|---------|
 * | dstL2IdConfig   |   1   |   254   |   254   |   254   |
 * |-----------------|-------|---------|---------|---------|
 * | dstL2IdConfig   |   2   |   255   |   254   |   255   |
 * |-----------------|-------|---------|---------|---------|
 * | dstL2IdConfig   |   3   |    2    |   254   |   255   |
 * |-----------------|-------|---------|---------|---------|
 * | priorityConfig  |   1   |    1    |    1    |    1    |
 * |-----------------|-------|---------|---------|---------|
 * | priorityConfig  |   2   |    1    |    2    |    3    |
 * |-----------------|-------|---------|---------|---------|
 * | priorityConfig  |   3   |    2    |    2    |    1    |
 * |-----------------|-------|---------|---------|---------|
 * | priorityConfig  |   4   |    1    |    1    |    2    |
 * |-----------------|-------|---------|---------|---------|
 * |-----------------|-------|---------|---------|---------|
 * | rriConfig       |   1   |   20    |   20    |   20    |
 * |-----------------|-------|---------|---------|---------|
 * | rriConfig       |   2   |   100   |    50   |   100   |
 *  -----------------|-------|---------|---------|---------
 *
 *  Three other parameters related to the scheduling that we configure are
 *  (1) 'prioToSps' which sets the scheduler attribute 'PriorityToSps' and
 *      is a flag to give scheduling priority to logical channels that are
 *      configured with SPS scheduling in case of priority tie
 *  (2) 'harqEnabled' which, if enabled, causes the scheduler to add additional
 *      resources for retransmissions, and
 *  (3) 'psfchPeriod' which affects scheduling by imposing requirements on
 *      the number of slots between retransmission attempts
 *
 * If 'psfchPeriod' is set to zero, then no PSFCH feedback channel will be
 * added to the resource pool, and the setting of 'harqEnabled' will instead
 * cause blind retransmissions to be scheduled
 *
 * The default configuration is:
 * --schedTypeConfig=1 (i.e., all LCs will have dynamic (per-packet) scheduling)
 * --dstL2IdConfig=1   (i.e., all LCs send to the same L2 ID of value 255)
 * --priorityConfig=1  (i.e., all LCs have the same priority of value 1)
 * --rriConfig=1       (i.e., all LCs have the same RRI, which doesn't matter in
 *                      this configuration as schedTypeConfig=1)
 * --prioToSps=false   (i.e., no priority to SPS, which doesn't matter in this
 *                      configuration as schedTypeConfig=1)
 * --harqEnabled=true  (retransmissions slots will be scheduled)
 * --psfchPeriod=4     (retransmissions slots will be scheduled)
 *
 * The selection of dstL2Id value configures the following:
 * - If the dstL2Id is 255, the CastType will be Broadcast
 * - If the dstL2Id is 254, the CastType will be Groupcast
 * - If the dstL2Id is 2, the CastType will be Unicast

 * The example will print on-screen the number of transmitted and received
 * packets during the simulation and the average packet delay.
 *
 * \code{.unparsed}
$ ./ns3 run "sl-multi-lc-example --help"
    \endcode
 *
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/gpsr-helper.h"
#include "ns3/gpsr.h"
#include "ns3/gpsr-ptable.h"
#include "ns3/stats-module.h"
#include "ns3/traci-module.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/nr-sl-phy-mac-common.h"

#include <iomanip>
#include <ostream>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SlNrGpsrExample");

uint32_t g_rxPktCounter = 0;            //!< Global variable to count RX packets
uint32_t g_txPktCounter = 0;            //!< Global variable to count TX packets
std::list<double> g_delays;             //!< Global list to store packet delays upon RX
std::ofstream g_fileGrantCreated;       //!< File stream for saving scheduling output
std::ostringstream g_firstGrantCreated; //!< String stream for saving first scheduling output
bool g_firstGrant = true; //!< Flag to control writing first grant to g_firstGrantCreated

// ========== Distance-based traffic generation ==========
std::vector<uint32_t> g_activeNodeIds;  //!< Track active node IDs from TraCI
NodeContainer* g_ueNodeContainerPtr = nullptr;  //!< Pointer to UE node container for traffic gen
double g_minDistanceForTraffic = 250.0;  //!< Minimum distance (m) for multi-hop traffic

/*
 * Structure to keep track of the transmission time of the packets at the
 * application layer. Used to calculate packet delay.
 */
struct PacketWithRxTimestamp
{
    Ptr<const Packet> p;
    Time txTimestamp;
};

/*
 * Map to store received packets and reception timestamps at the application
 * layer. Used to calculate packet delay at the application layer.
 */
std::map<std::string, PacketWithRxTimestamp> g_rxPacketsForDelayCalc;

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
    std::ostringstream oss;
    oss << InetSocketAddress::ConvertFrom(dstAddrs).GetPort() << "(" << seqTsSizeHeader.GetSeq()
        << ")";
    std::string mapKey = oss.str();
    PacketWithRxTimestamp mapValue;
    mapValue.p = p;
    mapValue.txTimestamp = Simulator::Now();
    g_rxPacketsForDelayCalc.insert(std::pair<std::string, PacketWithRxTimestamp>(mapKey, mapValue));
    NS_LOG_DEBUG(" TX: " << mapKey);
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

    double delay = 0.0;
    std::ostringstream oss;
    oss << InetSocketAddress::ConvertFrom(dstAddrs).GetPort() << "(" << seqTsSizeHeader.GetSeq()
        << ")";
    std::string mapKey = oss.str();

    auto it = g_rxPacketsForDelayCalc.find(mapKey);
    if (it == g_rxPacketsForDelayCalc.end())
    {
        NS_LOG_WARN("Rx packet not found for delay calculation: " << mapKey);
        // Don't calculate delay for this packet but still count it
    }
    else
    {
        delay =
            Simulator::Now().GetSeconds() * 1000.0 - it->second.txTimestamp.GetSeconds() * 1000.0;
        g_delays.push_back(delay);
        g_rxPacketsForDelayCalc.erase(mapKey);
    }
    NS_LOG_DEBUG(" RX: " << mapKey << delay);
}

// Forward declaration
void TraceGrantCreated(std::string context,
                       const struct NrSlUeMacScheduler::GrantInfo& grantInfo,
                       uint16_t psfchPeriod);

void WriteGrantCreated(std::ostream& grantStream,
                       std::string context,
                       const struct NrSlUeMacScheduler::GrantInfo& grantInfo,
                       uint16_t psfchPeriod);

uint32_t GetPacketSize(double dataRateKbps, Time rri);

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

    if (!validPairs.empty())
    {
        // Randomly select one pair from valid pairs
        size_t randomIdx = rand() % validPairs.size();
        uint32_t srcNodeId = std::get<0>(validPairs[randomIdx]);
        uint32_t dstNodeId = std::get<1>(validPairs[randomIdx]);
        double selectedDist = std::get<2>(validPairs[randomIdx]);
        
        // Calculate destination IP: 7.0.0.(nodeId + 2)
        uint32_t ipOffset = dstNodeId + 2;
        std::ostringstream ipStream;
        ipStream << "7.0." << ((ipOffset >> 8) & 0xFF) << "." << (ipOffset & 0xFF);
        Ipv4Address dstIp(ipStream.str().c_str());
        
        // Create and install OnOffApplication
        uint16_t port = 9001;  // Must match global PacketSink port
        OnOffHelper onoff("ns3::UdpSocketFactory", 
                          InetSocketAddress(dstIp, port));
        onoff.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        onoff.SetConstantRate(DataRate("200kb/s"), 512);
        
        ApplicationContainer app = onoff.Install(g_ueNodeContainerPtr->Get(srcNodeId));
        app.Start(Seconds(0.0));
        app.Stop(Seconds(1.5));  // Short burst
        
        // Connect TX trace for delay calculation
        app.Get(0)->TraceConnectWithoutContext("TxWithSeqTsSize",
                                               MakeCallback(&TxPacketTraceForDelay));
        
        NS_LOG_INFO("Multi-hop Traffic: Node " << srcNodeId << " -> Node " << dstNodeId 
                    << " (IP:" << dstIp << ", dist=" << selectedDist << "m, " 
                    << validPairs.size() << " valid pairs)");
    }
    else
    {
        NS_LOG_DEBUG("No distant pair found (threshold=" << g_minDistanceForTraffic << "m)");
    }

    // Schedule next traffic generation
    Simulator::Schedule(Seconds(2.0), &GenerateDistantTraffic);
}

/**
 * \brief Trace callback for PSSCH reception - updates GPSR neighbor SINR
 * 
 * This function is called when a node receives a PSSCH packet via SL.
 * It extracts the sender L2 ID and SINR, then updates the GPSR position table.
 * 
 * \param gpsr Pointer to the GPSR routing protocol instance
 * \param params The trace parameters containing SINR and sender info
 */
void
NotifySlPsschRx(Ptr<ns3::gpsr::RoutingProtocol> gpsr, SlRxDataPacketTraceParams params)
{
    // Filter out corrupted packets - unreliable SINR data
    if (params.m_corrupt || params.m_sci2Corrupted)
    {
        return;
    }
    
    // Map L2 ID to IP address: IP = 7.0.X.Y where (X*256 + Y) = L2 ID + 1
    // L2 ID is IMSI which is node_index + 1, so IP offset = L2 ID + 1 = node_index + 2
    uint32_t ipOffset = params.m_srcL2Id + 1;  // L2 ID starts at 1, IP offset starts at 2
    std::ostringstream ipStream;
    ipStream << "7.0." << ((ipOffset >> 8) & 0xFF) << "." << (ipOffset & 0xFF);
    Ipv4Address neighborIp(ipStream.str().c_str());
    
    // Update SINR in GPSR position table (only if neighbor exists)
    gpsr->GetPositionTable()->UpdateSinr(neighborIp, params.m_sinr);
}

int
main(int argc, char* argv[])
{
    // Scenario parameters
    uint16_t interUeDistance = 150; // 150m to force multi-hop (UE0-UE2 = 300m)
    uint16_t enableSingleFlow = 0; // 0 = disable all fixed flows, use only distance-based random traffic

    // Traffic parameters
    uint32_t udpPacketSize = 200;
    double dataRate = 8; // 8 kilobits per second

    // Traffic profile parameters
    uint16_t schedTypeConfig = 1;
    uint16_t dstL2IdConfig = 3;  // 3 = Unicast (flow 1 to unicast, flow 2 to groupcast, flow 3 to broadcast)
    uint16_t priorityConfig = 1;
    uint16_t rriConfig = 2;  // Use 100ms RRI for 4-node scenario
    bool prioToSps = false;
    bool harqEnabled = true;
    uint16_t psfchPeriod = 4;
    // If packet delay budget is zero, model will use T2 as selection window edge
    Time pdb = Seconds(0);

    // Simulation parameters.
    Time trafficTime = Seconds(2.0);

    // Testing flag
    bool testing = false;

    // NR parameters
    uint16_t numerologyBwpSl = 0; // SCS=15kHz (1ms slots) for reduced CPU load
    double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
    uint16_t bandwidthBandSl = 100;         // Multiple of 100 KHz; 100 = 10 MHz (typical V2X bandwidth)
    double txPower = 23;                    // dBm

    CommandLine cmd(__FILE__);
    cmd.AddValue("trafficTime", "The time traffic will be active in seconds", trafficTime);
    cmd.AddValue("interUeDistance", "The distance among the UEs in the topology", interUeDistance);
    cmd.AddValue("packetSize",
                 "packet size in bytes to be used by best effort traffic",
                 udpPacketSize);
    cmd.AddValue("dataRate",
                 "The data rate in kilobits per second for best effort traffic",
                 dataRate);
    cmd.AddValue("schedTypeConfig",
                 "The scheduling configuration for the three traffic profiles",
                 schedTypeConfig);
    cmd.AddValue("dstL2IdConfig",
                 "The dstL2Id configuration for the three traffic profiles",
                 dstL2IdConfig);
    cmd.AddValue("priorityConfig",
                 "The priority configuration for the three traffic profiles",
                 priorityConfig);
    cmd.AddValue("rriConfig", "The RRI configuration for the three traffic profiles", rriConfig);
    cmd.AddValue(
        "prioToSps",
        "Give scheduling priority to SPS logical channels in case of a tie (if set to True)",
        prioToSps);
    cmd.AddValue("harqEnabled", "Whether HARQ is enabled", harqEnabled);
    cmd.AddValue("psfchPeriod", "PSFCH period, in slots", psfchPeriod);
    cmd.AddValue("enableSingleFlow", "Enable single flow only (1, 2, or 3)", enableSingleFlow);
    cmd.AddValue(
        "testing",
        "Testing flag to do verification that the example is working as expected (if set to True)",
        testing);

    // Parse the command line
    cmd.Parse(argc, argv);

    // Check command line values
    NS_ABORT_MSG_UNLESS(enableSingleFlow <= 3, "Unsupported value: " << enableSingleFlow);

    // Final simulation time
    Time slBearersActivationTime = Seconds(2.0);
    Time finalSlBearersActivationTime = slBearersActivationTime;
    Time finalSimTime = trafficTime + finalSlBearersActivationTime +
                        Seconds(0.05); // Give some extra simulation time for packets to be Rx

    /*
     * Default values for the simulation. We are progressively removing all
     * the instances of SetDefault, but we need it for legacy code (LTE)
     */
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // Create UE node pool - pre-allocate nodes for SUMO vehicles
    NodeContainer ueNodeContainer;
    uint16_t ueNum = 50;  // Max concurrent vehicles from SUMO
    ueNodeContainer.Create(ueNum);

    // Setup initial mobility (far away from simulation area)
    // Nodes will be moved to correct positions by TraCI when vehicles depart
    MobilityHelper initialMobility;
    initialMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
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
    sumoClient->SetAttribute("SumoConfigPath", StringValue("/home/lyh/ns3-nr-v2x/ns-3-dev/src/nr/examples/sumo-small/grid_50veh/grid.sumocfg"));
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

    // Enable detailed logging for debugging GPSR + NR sidelink integration
    std::string logDir = "/home/lyh/ns3-nr-v2x/ns-3-dev/src/nr/examples/sl-examples/";
    std::ofstream logFile(logDir + "sl-nr-gpsr-debug.log");
    if (logFile.is_open())
    {
        NS_LOG_INFO("Debug log file opened: " << logDir << "sl-nr-gpsr-debug.log");
    }
    
    LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_DEBUG);  // Back to DEBUG for visibility
    //LogComponentEnable("EpcTftClassifier", LOG_LEVEL_INFO);
    //LogComponentEnable("LteSlTft", LOG_LEVEL_INFO);
    LogComponentEnable("NrSlUeMac", LOG_LEVEL_WARN);            // Reduced from INFO
    LogComponentEnable("SlNrGpsrExample", LOG_LEVEL_INFO);
    //LogComponentEnable("EpcUeNas", LOG_LEVEL_INFO);             // Commented out to reduce log
    //LogComponentEnable("NrSpectrumPhy", LOG_LEVEL_INFO);
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_NODE);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    
    // Redirect log output to file (clog for NS_LOG, cerr for std::cerr markers)
    std::clog.rdbuf(logFile.rdbuf());
    std::cerr.rdbuf(logFile.rdbuf());
    NS_LOG_INFO("=== SL-NR-GPSR Example Debug Log Started ===");
    std::cerr << "[CERR-TEST] std::cerr redirect test from main()" << std::endl;
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    /*
     * Spectrum division. We create one operational band, containing
     * one component carrier, and a single bandwidth part
     * centered at the frequency specified by centralFrequencyBandSl.
     * We will use the StreetCanyon channel modeling.
     */
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    CcBwpCreator::SimpleOperationBandConf bandConfSl(centralFrequencyBandSl,
                                                     bandwidthBandSl,
                                                     numCcPerBand,
                                                     BandwidthPartInfo::V2V_Highway);
    OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc(bandConfSl);

    Config::SetDefault("ns3::SpectrumChannel::MaxLossDb", DoubleValue(100.0));
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->InitializeOperationBand(&bandSl);

    allBwps = CcBwpCreator::GetAllBwps({bandSl});

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPower));

    // NR Sidelink attribute of UE MAC, which are common for all the UEs
    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(true));
    nrHelper->SetUeMacAttribute("T1", UintegerValue(2));
    nrHelper->SetUeMacAttribute("T2", UintegerValue(33));
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));

    uint8_t bwpIdForGbrMcptt = 0;

    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    // following parameter has no impact at the moment because:
    // 1. No support for PQI based mapping between the application and the LCs
    // 2. No scheduler to consider PQI
    // However, till such time all the NR SL examples should use GBR_MC_PUSH_TO_TALK
    // because we hard coded the PQI 65 in UE RRC.
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdForGbrMcptt));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdForGbrMcptt);

    // Install UE NetDevices
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodeContainer, allBwps);
    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    /**************************** SL configuration *****************************/
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);

    // SL error model
    std::string errorModel = "ns3::NrEesmIrT1";
    nrSlHelper->SetSlErrorModel(errorModel);
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    // SL scheduler
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(14));
    nrSlHelper->SetUeSlSchedulerAttribute("PriorityToSps", BooleanValue(prioToSps));

    nrSlHelper->PrepareUeForSidelink(ueNetDev, bwpIdContainer);

    // SlResourcePoolNr IE
    LteRrcSap::SlResourcePoolNr slResourcePoolNr;
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();
    std::vector<std::bitset<1>> slBitmap = {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1};
    ptrFactory->SetSlTimeResources(slBitmap);
    ptrFactory->SetSlSensingWindow(100); // T0 in ms
    ptrFactory->SetSlSelectionWindow(5);
    ptrFactory->SetSlFreqResourcePscch(10); // PSCCH RBs
    ptrFactory->SetSlSubchannelSize(50);
    ptrFactory->SetSlMaxNumPerReserve(3);
    ptrFactory->SetSlPsfchPeriod(psfchPeriod);
    ptrFactory->SetSlMinTimeGapPsfch(3);

    std::list<uint16_t> resourceReservePeriodList = {0, 20, 50, 100}; // in ms
    ptrFactory->SetSlResourceReservePeriodList(resourceReservePeriodList);
    // Once parameters are configured, we can create the pool
    LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool();
    slResourcePoolNr = pool;

    // Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
    LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
    slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
    uint16_t poolId = 0;
    LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
    slResourcePoolIdNr.id = poolId;
    slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
    slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

    // Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
    slBwpPoolConfigCommonNr.slTxPoolSelectedNormal[slResourcePoolIdNr.id] = slresoPoolConfigNr;

    // Configure the BWP IE
    LteRrcSap::Bwp bwp;
    bwp.numerology = numerologyBwpSl;
    bwp.symbolsPerSlots = 14;
    bwp.rbPerRbg = 1;
    bwp.bandwidth = bandwidthBandSl;

    // Configure the SlBwpGeneric IE
    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp = bwp;
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);

    // Configure the SlBwpConfigCommonNr IE
    LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
    slBwpConfigCommonNr.haveSlBwpGeneric = true;
    slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
    slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
    slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

    // Configure the SlFreqConfigCommonNr IE, which hold the array to store
    // the configuration of all Sidelink BWP (s).
    LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
    for (const auto& it : bwpIdContainer)
    {
        // it is the BWP id
        slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
    }

    // Configure the TddUlDlConfigCommon IE
    LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
    tddUlDlConfigCommon.tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";

    // Configure the SlPreconfigGeneralNr IE
    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    // Configure the SlUeSelectedConfig IE
    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    slUeSelectedPreConfig.slProbResourceKeep = 0;
    // Configure the SlPsschTxParameters IE
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = 5;
    // Configure the SlPsschTxConfigList IE
    LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
    pscchTxConfigList.slPsschTxParameters[0] = psschParams;
    slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

    /*
     * Finally, configure the SidelinkPreconfigNr This is the main structure
     * that needs to be communicated to NrSlUeRrc class
     */
    LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
    slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
    slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
    slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;

    // Communicate the above pre-configuration to the NrSlHelper
    nrSlHelper->InstallNrSlPreConfiguration(ueNetDev, slPreConfigNr);

    /****************************** End SL Configuration ***********************/

    // Fix random streams
    int64_t stream = 1;
    stream += nrHelper->AssignStreams(ueNetDev, stream);
    stream += nrSlHelper->AssignStreams(ueNetDev, stream);

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

    // ========== Connect RxPsschTraceUe to GPSR SINR updates ==========
    // For each node, get its GPSR instance and connect the PHY trace to update SINR
    NS_LOG_INFO("Connecting SINR traces for GPSR neighbor quality tracking...");
    for (uint32_t i = 0; i < ueNodeContainer.GetN(); ++i)
    {
        Ptr<Node> node = ueNodeContainer.Get(i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        if (!ipv4)
        {
            NS_LOG_WARN("Node " << i << " has no Ipv4, skipping SINR trace binding");
            continue;
        }
        
        // Find GPSR routing protocol (may be inside ListRouting)
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
                if (gpsrProto)
                {
                    break;
                }
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
        
        // Get NrSpectrumPhy for the Sidelink BWP
        Ptr<NrUeNetDevice> ueDevice = DynamicCast<NrUeNetDevice>(ueNetDev.Get(i));
        if (!ueDevice)
        {
            NS_LOG_WARN("Node " << i << " has no NrUeNetDevice");
            continue;
        }
        
        // Get PHY for the SL BWP (bwpIdForGbrMcptt is the SL BWP ID)
        Ptr<NrUePhy> uePhy = ueDevice->GetPhy(bwpIdForGbrMcptt);
        if (!uePhy)
        {
            NS_LOG_WARN("Node " << i << " has no PHY for BWP " << bwpIdForGbrMcptt);
            continue;
        }
        
        Ptr<NrSpectrumPhy> spectrumPhy = uePhy->GetSpectrumPhy();
        if (!spectrumPhy)
        {
            NS_LOG_WARN("Node " << i << " SpectrumPhy not found");
            continue;
        }
        
        // Connect trace source to callback
        bool connected = spectrumPhy->TraceConnectWithoutContext(
            "RxPsschTraceUe", 
            MakeBoundCallback(&NotifySlPsschRx, gpsrProto));
        
        if (connected)
        {
            NS_LOG_DEBUG("Node " << i << " SINR trace connected successfully");
        }
        else
        {
            NS_LOG_WARN("Node " << i << " failed to connect SINR trace");
        }
    }
    NS_LOG_INFO("SINR trace binding complete for GPSR enhancement");

    // Target IP
    Ipv4Address groupAddress4("225.0.0.0"); // use multicast address as destination
    Ipv4Address unicastAddress4("7.0.0.198");  // Node 196's IP (7.0.0.2 + 196 = 7.0.0.198)

    /************************** Traffic flows configuration ********************/
    /*
     * All traffic flows will be directed to the target IP (groupAddress) but each to a
     * different port. For each flow:
     * - we create a traffic profile (SlInfo),
     * - we create a traffic flow template (LteSlTft) using the traffic profile, and
     * - we activate a SL data radio bearer that will steer the traffic according to the TFT
     */

    // Create the traffic profiles
    uint32_t dstL2Broadcast = 255;
    uint32_t dstL2Groupcast = 254;
    uint32_t dstL2Unicast = 197; // IMSI 197 is assigned to node_196 (last node)
                               // Source L2 ID is the lower bits of the IMSI.

    SidelinkInfo slInfo1;
    slInfo1.m_harqEnabled = harqEnabled;
    slInfo1.m_pdb = pdb;
    SidelinkInfo slInfo2;
    slInfo2.m_harqEnabled = harqEnabled;
    slInfo2.m_pdb = pdb;
    SidelinkInfo slInfo3;
    slInfo3.m_harqEnabled = harqEnabled;
    slInfo3.m_pdb = pdb;

    // Assign the three traffic profile parameter values depending on the configuration
    switch (schedTypeConfig)
    {
    case 1:
        slInfo1.m_dynamic = true;
        slInfo2.m_dynamic = true;
        slInfo3.m_dynamic = true;
        break;
    case 2:
        slInfo1.m_dynamic = false;
        slInfo2.m_dynamic = false;
        slInfo3.m_dynamic = false;
        break;
    case 3:
        slInfo1.m_dynamic = true;
        slInfo2.m_dynamic = true;
        slInfo3.m_dynamic = false;
        break;
    case 4:
        slInfo1.m_dynamic = false;
        slInfo2.m_dynamic = false;
        slInfo3.m_dynamic = true;
        break;
    }

    switch (dstL2IdConfig)
    {
    case 1:
        slInfo1.m_dstL2Id = dstL2Groupcast;
        slInfo1.m_castType = SidelinkInfo::CastType::Groupcast;
        slInfo2.m_dstL2Id = dstL2Groupcast;
        slInfo2.m_castType = SidelinkInfo::CastType::Groupcast;
        slInfo3.m_dstL2Id = dstL2Groupcast;
        slInfo3.m_castType = SidelinkInfo::CastType::Groupcast;
        break;
    case 2:
        slInfo1.m_dstL2Id = dstL2Broadcast;
        slInfo1.m_castType = SidelinkInfo::CastType::Broadcast;
        slInfo2.m_dstL2Id = dstL2Groupcast;
        slInfo2.m_castType = SidelinkInfo::CastType::Groupcast;
        slInfo3.m_dstL2Id = dstL2Broadcast;
        slInfo3.m_castType = SidelinkInfo::CastType::Broadcast;
        break;
    case 3:
        slInfo1.m_dstL2Id = dstL2Unicast;
        slInfo1.m_castType = SidelinkInfo::CastType::Unicast;
        slInfo2.m_dstL2Id = dstL2Groupcast;
        slInfo2.m_castType = SidelinkInfo::CastType::Groupcast;
        slInfo3.m_dstL2Id = dstL2Broadcast;
        slInfo3.m_castType = SidelinkInfo::CastType::Broadcast;
        break;
    }

    switch (priorityConfig)
    {
    case 1:
        slInfo1.m_priority = 1;
        slInfo2.m_priority = 1;
        slInfo3.m_priority = 1;
        break;
    case 2:
        slInfo1.m_priority = 1;
        slInfo2.m_priority = 2;
        slInfo3.m_priority = 3;
        break;
    case 3:
        slInfo1.m_priority = 2;
        slInfo2.m_priority = 2;
        slInfo3.m_priority = 1;
        break;
    case 4:
        slInfo1.m_priority = 1;
        slInfo2.m_priority = 1;
        slInfo3.m_priority = 2;
        break;
    }

    switch (rriConfig)
    {
    case 1:
        slInfo1.m_rri = MilliSeconds(20);
        slInfo2.m_rri = MilliSeconds(20);
        slInfo3.m_rri = MilliSeconds(20);
        break;
    case 2:
        slInfo1.m_rri = MilliSeconds(100);
        slInfo2.m_rri = MilliSeconds(50);
        slInfo3.m_rri = MilliSeconds(100);
        break;
    }

    // Create TFTs and activate SL-DRBs
    Address remoteAddress1;
    Address remoteAddress2;
    Address remoteAddress3;
    Address localAddress1;
    Address localAddress2;
    Address localAddress3;
    uint16_t port1 = 8001;
    uint16_t port2 = 8002;
    uint16_t port3 = 8003;
    Ptr<LteSlTft> tft1;
    Ptr<LteSlTft> tft2;
    Ptr<LteSlTft> tft3;

    // Assign IP address for the UEs
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(ueNetDev);
    NS_LOG_DEBUG("Device 0 has address " << ueIpIface.GetAddress(0)); // 7.0.0.2
    NS_LOG_DEBUG("Device 1 has address " << ueIpIface.GetAddress(1)); // 7.0.0.3

    // Note: Static routing removed - GPSR handles routing based on geographic position

    // Create TFTs for each traffic profile and corresponding addresses/port
    // Use groupAddress4 for both Groupcast and Broadcast cast types
    localAddress1 = InetSocketAddress(Ipv4Address::GetAny(), port1);
    if (dstL2IdConfig == 3)
    {
        remoteAddress1 = InetSocketAddress(unicastAddress4, port1);
        tft1 =
            Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, unicastAddress4, port1, slInfo1);
    }
    else
    {
        remoteAddress1 = InetSocketAddress(groupAddress4, port1);
        tft1 = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, port1, slInfo1);
    }

    remoteAddress2 = InetSocketAddress(groupAddress4, port2);
    localAddress2 = InetSocketAddress(Ipv4Address::GetAny(), port2);
    tft2 = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, port2, slInfo2);

    remoteAddress3 = InetSocketAddress(groupAddress4, port3);
    localAddress3 = InetSocketAddress(Ipv4Address::GetAny(), port3);
    tft3 = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, port3, slInfo3);

    // Activate SL data radio bearers for each traffic flow template and profile
    
    // Add TFT for GPSR HELLO broadcast traffic (required for GPSR neighbor discovery)
    Ipv4Address gpsrBroadcast("7.255.255.255");  // Subnet broadcast for GPSR HELLO
    uint16_t gpsrPort = 666;  // GPSR uses port 666 for control messages
    SidelinkInfo slInfoGpsr;
    slInfoGpsr.m_dstL2Id = 255;  // Broadcast L2 ID
    slInfoGpsr.m_castType = SidelinkInfo::CastType::Broadcast;
    slInfoGpsr.m_dynamic = true;
    slInfoGpsr.m_priority = 3;  // Lower priority than data (1) to avoid slot conflicts
    slInfoGpsr.m_harqEnabled = false;  // Broadcast has no HARQ
    Ptr<LteSlTft> tftGpsr = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, gpsrBroadcast, gpsrPort, slInfoGpsr);
    nrSlHelper->ActivateNrSlBearer(Seconds(0), ueNetDev, tftGpsr);  // Activate at t=0 for routing
    NS_LOG_INFO("GPSR HELLO TFT activated for " << gpsrBroadcast << ":" << gpsrPort);

    // ========== Complete TFT mapping for all 400 nodes ==========
    // GPSR multi-hop routing requires IP → L2 ID mapping for every possible next-hop.
    // Without this, NR Sidelink cannot find the L2 ID for intermediate nodes.
    // Formula: IP = 7.0.X.Y where X*256 + Y = node_index + 2, L2 ID = node_index + 1
    NS_LOG_INFO("Creating TFT mappings for " << ueNum << " nodes...");
    for (uint32_t i = 0; i < ueNum; ++i)
    {
        // Calculate IP address: 7.0.0.2 + i
        uint32_t ipOffset = i + 2;  // Starts from 7.0.0.2
        uint8_t ipByte2 = (ipOffset >> 8) & 0xFF;  // X in 7.0.X.Y
        uint8_t ipByte3 = ipOffset & 0xFF;         // Y in 7.0.X.Y
        Ipv4Address nodeIp((7 << 24) | (ipByte2 << 8) | ipByte3);  // 7.0.X.Y
        
        // L2 ID = IMSI = node_index + 1
        uint32_t l2Id = i + 1;
        
        // Create SidelinkInfo for this node
        SidelinkInfo slInfoNode;
        slInfoNode.m_dstL2Id = l2Id;
        slInfoNode.m_castType = SidelinkInfo::CastType::Unicast;
        slInfoNode.m_dynamic = true;
        slInfoNode.m_harqEnabled = harqEnabled;
        slInfoNode.m_pdb = pdb;
        slInfoNode.m_priority = 1;
        
        // Create TFT: nodeIp → l2Id (no port = wildcard port matching)
        // FIX: Configure as TRANSMIT-only so nodes can send to any destination
        // based on IP, but do NOT listen to all L2 IDs (prevents broadcast storm).
        // IMPORTANT: Use 3-arg constructor (no port) so m_hasRemotePort=false
        Ptr<LteSlTft> tftNode = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, 
                                                  nodeIp, slInfoNode);
        nrSlHelper->ActivateNrSlBearer(Seconds(0), ueNetDev, tftNode);
        
        // FIX: Configure RECEIVE-only bearer solely for the node's own L2 ID.
        // This ensures each node only receives unicast traffic destined for itself.
        // We must activate this ONLY on the specific node 'i'.
        SidelinkInfo slInfoSelfRx = slInfoNode;
        slInfoSelfRx.m_dstL2Id = l2Id; // Listen to own L2 ID
        Ptr<LteSlTft> tftSelfRx = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, 
                                                   nodeIp, slInfoSelfRx);
        // Activate ONLY on node 'i'
        nrSlHelper->ActivateNrSlBearer(Seconds(0), NetDeviceContainer(ueNetDev.Get(i)), tftSelfRx);
    }
    NS_LOG_INFO("TFT mappings created for all " << ueNum << " nodes.");
    // ========== End of Complete TFT mapping ==========

    if (!enableSingleFlow || enableSingleFlow == 1)
    {
        // FIX: Only activate application flow bearers on endpoints, not all UEs
        // Source node (node 0, IMSI 1): TRANSMIT only
        // Destination node (node 49, IMSI 50): RECEIVE only
        // This prevents intermediate nodes from establishing application-level RX/TX
        
        // Source: Node 0 - TRANSMIT only
        SidelinkInfo slInfoTx = slInfo1;
        Ptr<LteSlTft> tft1Tx = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, 
                                                 unicastAddress4, port1, slInfoTx);
        nrSlHelper->ActivateNrSlBearer(finalSlBearersActivationTime, 
                                       NetDeviceContainer(ueNetDev.Get(0)), tft1Tx);
        
        // Destination: Node 49 (ueNum-1) - RECEIVE only
        SidelinkInfo slInfoRx = slInfo1;
        Ptr<LteSlTft> tft1Rx = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, 
                                                 unicastAddress4, port1, slInfoRx);
        nrSlHelper->ActivateNrSlBearer(finalSlBearersActivationTime, 
                                       NetDeviceContainer(ueNetDev.Get(ueNum - 1)), tft1Rx);
    }
    if (!enableSingleFlow || enableSingleFlow == 2)
    {
        // For groupcast/broadcast flows, keep BIDIRECTIONAL on all UEs
        nrSlHelper->ActivateNrSlBearer(finalSlBearersActivationTime, ueNetDev, tft2);
    }
    if (!enableSingleFlow || enableSingleFlow == 3)
    {
        // For groupcast/broadcast flows, keep BIDIRECTIONAL on all UEs
        nrSlHelper->ActivateNrSlBearer(finalSlBearersActivationTime, ueNetDev, tft3);
    }

    /*
     * Configure the applications:
     * Client app: OnOff application configure to generate CBR traffic
     * Server app: PacketSink application.
     */

    // Configure client applications
    std::string dataRateString = std::to_string(dataRate) + "kb/s";
    OnOffHelper sidelinkClient1("ns3::UdpSocketFactory", remoteAddress1);
    sidelinkClient1.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    sidelinkClient1.SetConstantRate(DataRate(dataRateString),
                                    GetPacketSize(dataRate, slInfo1.m_rri));

    OnOffHelper sidelinkClient2("ns3::UdpSocketFactory", remoteAddress2);
    sidelinkClient2.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    sidelinkClient2.SetConstantRate(DataRate(dataRateString),
                                    GetPacketSize(dataRate, slInfo2.m_rri));

    OnOffHelper sidelinkClient3("ns3::UdpSocketFactory", remoteAddress3);
    sidelinkClient3.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    sidelinkClient3.SetConstantRate(DataRate(dataRateString),
                                    GetPacketSize(dataRate, slInfo3.m_rri));

    ApplicationContainer allClientApps;
    ApplicationContainer allServerApps;

    if (false) // Disable legacy flows - only use random distance-based traffic
    {
        // Install client applications on the first UE (Tx)
        ApplicationContainer clientApps1 = sidelinkClient1.Install(ueNodeContainer.Get(0));
        clientApps1.Start(finalSlBearersActivationTime);
        clientApps1.Stop(finalSimTime);
        allClientApps.Add(clientApps1);
    }

    if (false) // Disable legacy flows - only use random distance-based traffic
    {
        ApplicationContainer clientApps2 = sidelinkClient2.Install(ueNodeContainer.Get(0));
        clientApps2.Start(finalSlBearersActivationTime);
        clientApps2.Stop(finalSimTime);
        allClientApps.Add(clientApps2);
    }

    if (false) // Disable legacy flows - only use random distance-based traffic
    {
        ApplicationContainer clientApps3 = sidelinkClient3.Install(ueNodeContainer.Get(0));
        clientApps3.Start(finalSlBearersActivationTime);
        clientApps3.Stop(finalSimTime);
        allClientApps.Add(clientApps3);
    }

    // ========== Global PacketSink for distance-based traffic ==========
    // Install PacketSink on ALL nodes to receive traffic from random distant sources
    ApplicationContainer globalServerApps;
    uint16_t globalPort = 9001;  // Port for distance-based traffic (avoid conflict with 8001)
    PacketSinkHelper globalSink("ns3::UdpSocketFactory", 
                                InetSocketAddress(Ipv4Address::GetAny(), globalPort));
    globalSink.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    
    for (uint32_t i = 0; i < ueNum; ++i)
    {
        ApplicationContainer nodeServerApp = globalSink.Install(ueNodeContainer.Get(i));
        nodeServerApp.Start(Seconds(1.0));
        globalServerApps.Add(nodeServerApp);
    }
    NS_LOG_INFO("Installed PacketSink on " << ueNum << " nodes (port " << globalPort << ")");
    // ========== End global PacketSink ==========

    if (!enableSingleFlow || enableSingleFlow == 1)
    {
        // Configure server applications and install them in the second UE (Rx)
        ApplicationContainer serverApps1;
        PacketSinkHelper sidelinkSink1("ns3::UdpSocketFactory", localAddress1);
        sidelinkSink1.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        serverApps1 = sidelinkSink1.Install(ueNodeContainer.Get(ueNum - 1));  // Node 196 (last node)
        serverApps1.Start(Seconds(2.0));
        allServerApps.Add(serverApps1);
    }

    if (!enableSingleFlow || enableSingleFlow == 2)
    {
        ApplicationContainer serverApps2;
        PacketSinkHelper sidelinkSink2("ns3::UdpSocketFactory", localAddress2);
        sidelinkSink2.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        serverApps2 = sidelinkSink2.Install(ueNodeContainer.Get(ueNum - 1));  // last node
        serverApps2.Start(Seconds(2.0));
        allServerApps.Add(serverApps2);
    }

    if (!enableSingleFlow || enableSingleFlow == 3)
    {
        ApplicationContainer serverApps3;
        PacketSinkHelper sidelinkSink3("ns3::UdpSocketFactory", localAddress3);
        sidelinkSink3.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        serverApps3 = sidelinkSink3.Install(ueNodeContainer.Get(ueNum - 1));  // last node
        serverApps3.Start(Seconds(2.0));
        allServerApps.Add(serverApps3);
    }

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
    // Connect RX trace to global PacketSinks for distance-based traffic
    for (uint32_t ac = 0; ac < globalServerApps.GetN(); ac++)
    {
        globalServerApps.Get(ac)->TraceConnectWithoutContext("RxWithSeqTsSize",
                                                             MakeCallback(&RxPacketTraceForDelay));
    }
    /******************** END Application packet  tracing **********************/

    g_fileGrantCreated.open("sl-multi-lc-scheduling.dat", std::ofstream::out);
    auto ueDevice0 = ueNetDev.Get(0)->GetObject<NrUeNetDevice>();
    auto ueMac0 = ueDevice0->GetMac(0)->GetObject<NrSlUeMac>();
    PointerValue v;
    ueMac0->GetAttribute("NrSlUeMacScheduler", v);
    auto scheduler0 = v.Get<NrSlUeMacScheduler>()->GetObject<NrSlUeMacSchedulerFixedMcs>();
    scheduler0->TraceConnect("GrantCreated", "0", MakeCallback(&TraceGrantCreated));

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
        Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
        if (mob)
        {
            mob->SetPosition(Vector(10000.0 + (rand() % 100), 10000.0 + (rand() % 100), 0.0));
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
    Simulator::Schedule(Seconds(5.0), &GenerateDistantTraffic);
    NS_LOG_INFO("Scheduled distance-based traffic generation (minDist=" << g_minDistanceForTraffic << "m)");

    Simulator::Stop(finalSimTime);
    Simulator::Run();

    g_fileGrantCreated.close();
    std::cout << "schedTypeConfig = " << schedTypeConfig << "; dstL2IdConfig = " << dstL2IdConfig;
    std::cout << " priorityConfig = " << priorityConfig << "; rriConfig = " << rriConfig
              << std::endl;
    std::cout << "prioToSps = " << prioToSps << "; harqEnabled = " << harqEnabled
              << "; psfchPeriod = " << psfchPeriod << std::endl;

    std::cout << "Total Tx packets = " << g_txPktCounter << std::endl;
    std::cout << "Total Rx packets = " << g_rxPktCounter << std::endl;
    double delaySum = 0;
    for (auto it = g_delays.begin(); it != g_delays.end(); it++)
    {
        delaySum += *it;
    }
    std::cout << "Average packet delay = " << delaySum / g_delays.size() << " ms" << std::endl;
    std::cout << "Output trace of first grant created:" << std::endl;
    std::cout << g_firstGrantCreated.str();

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

void
TraceGrantCreated(std::string context,
                  const struct NrSlUeMacScheduler::GrantInfo& grantInfo,
                  uint16_t psfchPeriod)
{
    if (g_firstGrant)
    {
        WriteGrantCreated(g_fileGrantCreated, context, grantInfo, psfchPeriod);
        WriteGrantCreated(g_firstGrantCreated, context, grantInfo, psfchPeriod);
        g_firstGrant = false;
    }
    else
    {
        WriteGrantCreated(g_fileGrantCreated, context, grantInfo, psfchPeriod);
    }
}

void
WriteGrantCreated(std::ostream& grantStream,
                  std::string context,
                  const struct NrSlUeMacScheduler::GrantInfo& grantInfo,
                  uint16_t psfchPeriod)
{
    grantStream << Now().As(Time::S) << " " << context << " ";
    grantStream << (grantInfo.isDynamic ? "dynamic " : "sps ");
    grantStream << +grantInfo.harqId << " ";
    grantStream << (grantInfo.harqEnabled ? "harq" : "no-harq");
    if (!grantInfo.isDynamic)
    {
        grantStream << " " << +grantInfo.cReselCounter << " " << +grantInfo.slResoReselCounter
                    << " " << +grantInfo.nSelected << " " << +grantInfo.tbTxCounter
                    << grantInfo.rri.GetMilliSeconds() << std::endl;
    }
    else
    {
        grantStream << std::endl;
    }
    for (const auto& it : grantInfo.slotAllocations)
    {
        uint64_t slot = it.sfn.Normalize();
        double slotDurationS = 0.001 / (1 << it.sfn.GetNumerology());
        double slotTimeS = slot * slotDurationS;
        grantStream << "    " << std::fixed << std::setprecision(6) << slotTimeS << " "
                    << it.sfn.Normalize() << " ";
        grantStream << it.slPsschSubChStart << ":" << it.slPsschSubChLength << " " << it.dstL2Id
                    << " ";
        grantStream << psfchPeriod << " " << it.txSci1A << " " << +it.slotNumInd;
        for (const auto& it2 : it.slRlcPduInfo)
        {
            grantStream << " (LCID " << +it2.lcid << " size " << it2.size << ")";
        }
        grantStream << std::endl;
    }
}

// Get the packet size in bytes that supports the provided data rate and RRI
uint32_t
GetPacketSize(double dataRateKbps, Time rri)
{
    return static_cast<uint32_t>(dataRateKbps * 1000 * rri.GetSeconds() / 8);
}
