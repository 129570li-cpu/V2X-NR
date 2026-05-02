/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * 802.11p + GPSR baseline extended with ELITE controller support.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/elite-module.h"
#include "ns3/gpsr-helper.h"
#include "ns3/gpsr.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-module.h"
#include "ns3/traci-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <list>
#include <map>
#include <sstream>
#include <tuple>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SlWifiEliteExample");

namespace
{

uint32_t g_rxPktCounter = 0;
uint32_t g_txPktCounter = 0;
uint32_t g_rxPktDelivered = 0;
uint64_t g_rxPayloadBytes = 0;
std::list<double> g_delays;

uint32_t g_udpPacketSize = 412;
double g_dataRateKbps = 100.0;
double g_minDistanceForTraffic = 500.0;
uint32_t g_flowId = 0;
uint16_t g_basePort = 10000;

std::vector<uint32_t> g_activeNodeIds;
NodeContainer* g_ueNodeContainerPtr = nullptr;
std::map<Mac48Address, Ipv4Address> g_macToIpMap;

Ptr<gpsr::RoutingProtocol>
GetGpsrRoutingProtocol(Ptr<Node> node)
{
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4)
    {
        return nullptr;
    }

    Ptr<Ipv4RoutingProtocol> routing = ipv4->GetRoutingProtocol();
    Ptr<Ipv4ListRouting> listRouting = DynamicCast<Ipv4ListRouting>(routing);
    if (!listRouting)
    {
        return DynamicCast<gpsr::RoutingProtocol>(routing);
    }

    for (uint32_t i = 0; i < listRouting->GetNRoutingProtocols(); ++i)
    {
        int16_t priority = 0;
        Ptr<Ipv4RoutingProtocol> protocol = listRouting->GetRoutingProtocol(i, priority);
        Ptr<gpsr::RoutingProtocol> gpsr = DynamicCast<gpsr::RoutingProtocol>(protocol);
        if (gpsr)
        {
            return gpsr;
        }
    }
    return nullptr;
}

void
TxPacketTraceForDelay(Ptr<const Packet> packet,
                      const Address& srcAddrs,
                      const Address& dstAddrs,
                      const SeqTsSizeHeader& header)
{
    g_txPktCounter++;
    NS_LOG_DEBUG("TX seq=" << header.GetSeq());
}

void
RxPacketTraceForDelay(Ptr<const Packet> packet,
                      const Address& srcAddrs,
                      const Address& dstAddrs,
                      const SeqTsSizeHeader& header)
{
    g_rxPktCounter++;
    g_rxPktDelivered++;
    g_rxPayloadBytes += packet->GetSize();

    const double delayMs = (Simulator::Now() - header.GetTs()).GetMilliSeconds();
    g_delays.push_back(delayMs);
    NS_LOG_DEBUG("RX seq=" << header.GetSeq() << " delay=" << delayMs << " ms");
}

void
NotifyWifiPhyRx(Ptr<gpsr::RoutingProtocol> gpsr,
                Ptr<const Packet> packet,
                uint16_t channelFreqMhz,
                WifiTxVector txVector,
                MpduInfo aMpdu,
                SignalNoiseDbm signalNoise,
                uint16_t staId)
{
    WifiMacHeader hdr;
    Ptr<Packet> copy = packet->Copy();
    if (copy->PeekHeader(hdr) == 0)
    {
        return;
    }

    auto it = g_macToIpMap.find(hdr.GetAddr2());
    if (it == g_macToIpMap.end())
    {
        return;
    }

    const double snrDb = signalNoise.signal - signalNoise.noise;
    const double snrLinear = std::pow(10.0, snrDb / 10.0);
    gpsr->GetPositionTable()->UpdateSinr(it->second, snrLinear);
}

void
GenerateDistantTraffic()
{
    if (g_activeNodeIds.size() < 2 || g_ueNodeContainerPtr == nullptr)
    {
        Simulator::Schedule(Seconds(2.0), &GenerateDistantTraffic);
        return;
    }

    std::vector<std::tuple<uint32_t, uint32_t, double>> validPairs;
    for (size_t i = 0; i < g_activeNodeIds.size(); ++i)
    {
        for (size_t j = i + 1; j < g_activeNodeIds.size(); ++j)
        {
            Ptr<Node> n1 = g_ueNodeContainerPtr->Get(g_activeNodeIds[i]);
            Ptr<Node> n2 = g_ueNodeContainerPtr->Get(g_activeNodeIds[j]);
            Ptr<MobilityModel> m1 = n1->GetObject<MobilityModel>();
            Ptr<MobilityModel> m2 = n2->GetObject<MobilityModel>();
            if (!m1 || !m2)
            {
                continue;
            }

            const double distance = CalculateDistance(m1->GetPosition(), m2->GetPosition());
            if (distance >= g_minDistanceForTraffic)
            {
                validPairs.emplace_back(g_activeNodeIds[i], g_activeNodeIds[j], distance);
            }
        }
    }

    const uint32_t numFlows = std::min<size_t>(5, validPairs.size());
    for (uint32_t f = 0; f < numFlows; ++f)
    {
        const size_t randomIdx = std::rand() % validPairs.size();
        const uint32_t srcNodeId = std::get<0>(validPairs[randomIdx]);
        const uint32_t dstNodeId = std::get<1>(validPairs[randomIdx]);
        const double selectedDist = std::get<2>(validPairs[randomIdx]);
        validPairs.erase(validPairs.begin() + randomIdx);

        const uint32_t ipOffset = dstNodeId + 2;
        std::ostringstream ipStream;
        ipStream << "7.0." << ((ipOffset >> 8) & 0xff) << "." << (ipOffset & 0xff);
        Ipv4Address dstIp(ipStream.str().c_str());

        const uint16_t port = g_basePort + g_flowId++;
        const Time now = Simulator::Now();
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        sinkHelper.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        ApplicationContainer sinkApp = sinkHelper.Install(g_ueNodeContainerPtr->Get(dstNodeId));
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(Seconds(6.0));
        sinkApp.Get(0)->TraceConnectWithoutContext("RxWithSeqTsSize",
                                                   MakeCallback(&RxPacketTraceForDelay));

        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(dstIp, port));
        onoff.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        onoff.SetConstantRate(DataRate(std::to_string(g_dataRateKbps) + "kb/s"),
                              g_udpPacketSize);
        ApplicationContainer app = onoff.Install(g_ueNodeContainerPtr->Get(srcNodeId));
        app.Start(Seconds(0.0));
        app.Stop(Seconds(5.0));
        app.Get(0)->TraceConnectWithoutContext("TxWithSeqTsSize",
                                               MakeCallback(&TxPacketTraceForDelay));

        NS_LOG_INFO("ELITE traffic: node " << srcNodeId << " -> " << dstNodeId
                                           << " dstIp=" << dstIp
                                           << " dist=" << selectedDist << " m");
    }

    Simulator::Schedule(Seconds(1.0), &GenerateDistantTraffic);
}

} // namespace

int
main(int argc, char* argv[])
{
    uint16_t ueNum = 400;
    uint32_t udpPacketSize = 412;
    double dataRate = 100.0;
    Time trafficTime = Seconds(60.0);
    Time startupTime = Seconds(2.0);
    double txPower = 17.0;
    uint32_t eliteEpisodesPerCycle = 10;
    Time eliteCycleInterval = Seconds(1.0);
    std::string sumoConfigPath =
        "/home/lyh/ns3-nr-v2x/ns-3-dev/src/nr/examples/grid_network_fed/grid.sumocfg";
    std::string sumoNetPath =
        "/home/lyh/ns3-nr-v2x/ns-3-dev/src/nr/examples/grid_network_fed/grid.net.xml";

    CommandLine cmd(__FILE__);
    cmd.AddValue("ueNum", "Maximum concurrent SUMO vehicles", ueNum);
    cmd.AddValue("trafficTime", "Traffic active time in seconds", trafficTime);
    cmd.AddValue("packetSize", "UDP payload size in bytes", udpPacketSize);
    cmd.AddValue("dataRate", "Application data rate in kb/s", dataRate);
    cmd.AddValue("txPower", "802.11p transmit power in dBm", txPower);
    cmd.AddValue("eliteEpisodesPerCycle", "ELITE training episodes per objective per cycle",
                 eliteEpisodesPerCycle);
    cmd.AddValue("eliteCycleInterval", "ELITE sync/train cycle interval", eliteCycleInterval);
    cmd.AddValue("sumoConfigPath", "Path to SUMO .sumocfg", sumoConfigPath);
    cmd.AddValue("sumoNetPath", "Path to SUMO net.xml", sumoNetPath);
    cmd.Parse(argc, argv);

    g_udpPacketSize = udpPacketSize;
    g_dataRateKbps = dataRate;

    LogComponentEnable("SlWifiEliteExample", LOG_LEVEL_INFO);
    LogComponentEnable("EliteController", LOG_LEVEL_INFO);
    LogComponentEnableAll(LOG_PREFIX_TIME);
    LogComponentEnableAll(LOG_PREFIX_NODE);

    Packet::EnableChecking();
    Packet::EnablePrinting();

    NodeContainer ueNodeContainer;
    ueNodeContainer.Create(ueNum);
    g_ueNodeContainerPtr = &ueNodeContainer;

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    Ptr<GridPositionAllocator> gridAlloc = CreateObject<GridPositionAllocator>();
    gridAlloc->SetAttribute("MinX", DoubleValue(10000.0));
    gridAlloc->SetAttribute("MinY", DoubleValue(10000.0));
    gridAlloc->SetAttribute("DeltaX", DoubleValue(500.0));
    gridAlloc->SetAttribute("DeltaY", DoubleValue(500.0));
    gridAlloc->SetAttribute("GridWidth", UintegerValue(20));
    gridAlloc->SetAttribute("LayoutType", StringValue("RowFirst"));
    mobility.SetPositionAllocator(gridAlloc);
    mobility.Install(ueNodeContainer);

    Ptr<TwinEnvironment> twin = CreateObject<TwinEnvironment>();
    const bool roadOk = twin->InitializeRoadGraphFromSumoNetXml(sumoNetPath);
    if (!roadOk)
    {
        NS_FATAL_ERROR("Failed to parse SUMO road graph: " << sumoNetPath);
    }

    Ptr<EliteController> eliteController = CreateObject<EliteController>();
    eliteController->SetTwinEnvironment(twin);
    eliteController->SetCycleInterval(eliteCycleInterval);
    eliteController->SetEpisodesPerCycle(eliteEpisodesPerCycle);

    Ptr<TraciClient> sumoClient = CreateObject<TraciClient>();
    sumoClient->SetAttribute("SumoConfigPath", StringValue(sumoConfigPath));
    sumoClient->SetAttribute("SumoBinaryPath", StringValue(""));
    sumoClient->SetAttribute("SynchInterval", TimeValue(Seconds(0.1)));
    sumoClient->SetAttribute("StartTime", TimeValue(Seconds(0.0)));
    sumoClient->SetAttribute("SumoGUI", BooleanValue(false));
    sumoClient->SetAttribute("SumoPort", UintegerValue(3400));
    sumoClient->SetAttribute("PenetrationRate", DoubleValue(1.0));
    sumoClient->SetAttribute("SumoLogFile", BooleanValue(true));
    sumoClient->SetAttribute("SumoStepLog", BooleanValue(false));
    sumoClient->SetAttribute("SumoSeed", IntegerValue(10));
    sumoClient->SetAttribute("SumoWaitForSocket", TimeValue(Seconds(1.0)));

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel",
                                   "Frequency",
                                   DoubleValue(5.9e9),
                                   "HeightAboveZ",
                                   DoubleValue(1.5));

    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());
    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    wifiPhy.Set("TxPowerStart", DoubleValue(txPower));
    wifiPhy.Set("TxPowerEnd", DoubleValue(txPower));
    wifiPhy.Set("ChannelSettings", StringValue("{178, 10, BAND_5GHZ, 0}"));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211p);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("OfdmRate6MbpsBW10MHz"),
                                 "ControlMode",
                                 StringValue("OfdmRate6MbpsBW10MHz"),
                                 "NonUnicastMode",
                                 StringValue("OfdmRate6MbpsBW10MHz"));

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer ueNetDev = wifi.Install(wifiPhy, wifiMac, ueNodeContainer);

    GpsrHelper gpsr;
    InternetStackHelper internet;
    internet.SetRoutingHelper(gpsr);
    internet.Install(ueNodeContainer);
    gpsr.Install(ueNodeContainer);

    Ipv4AddressHelper ipv4Helper;
    ipv4Helper.SetBase("7.0.0.0", "255.255.0.0", "0.0.0.2");
    ipv4Helper.Assign(ueNetDev);

    uint32_t attachedGpsr = 0;
    for (uint32_t i = 0; i < ueNodeContainer.GetN(); ++i)
    {
        Ptr<Node> node = ueNodeContainer.Get(i);
        Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(ueNetDev.Get(i));
        Ptr<gpsr::RoutingProtocol> gpsrProtocol = GetGpsrRoutingProtocol(node);
        if (!wifiDevice || !gpsrProtocol)
        {
            continue;
        }

        const uint32_t ipOffset = i + 2;
        std::ostringstream ipStream;
        ipStream << "7.0." << ((ipOffset >> 8) & 0xff) << "." << (ipOffset & 0xff);
        g_macToIpMap[wifiDevice->GetMac()->GetAddress()] = Ipv4Address(ipStream.str().c_str());

        gpsrProtocol->SetEliteController(eliteController);
        wifiDevice->GetPhy()->TraceConnectWithoutContext(
            "MonitorSnifferRx",
            MakeBoundCallback(&NotifyWifiPhyRx, gpsrProtocol));
        ++attachedGpsr;
    }

    std::function<Ptr<Node>(const std::string&)> setupNewNode =
        [&ueNodeContainer](const std::string& vehicleId) -> Ptr<Node> {
        const std::string numStr = vehicleId.substr(3);
        const uint32_t nodeId = std::stoul(numStr);
        if (nodeId >= ueNodeContainer.GetN())
        {
            NS_FATAL_ERROR("Vehicle ID " << vehicleId << " exceeds node pool size");
        }
        g_activeNodeIds.push_back(nodeId);
        return ueNodeContainer.Get(nodeId);
    };

    std::function<void(Ptr<Node>)> shutdownNode = [](Ptr<Node> exNode) {
        const uint32_t nodeId = exNode->GetId();
        auto it = std::find(g_activeNodeIds.begin(), g_activeNodeIds.end(), nodeId);
        if (it != g_activeNodeIds.end())
        {
            g_activeNodeIds.erase(it);
        }

        Ptr<ConstantVelocityMobilityModel> mob =
            exNode->GetObject<ConstantVelocityMobilityModel>();
        if (mob)
        {
            mob->SetPosition(Vector(10000.0 + (std::rand() % 100),
                                    10000.0 + (std::rand() % 100),
                                    0.0));
            mob->SetVelocity(Vector(0.0, 0.0, 0.0));
        }
    };

    sumoClient->SumoSetup(setupNewNode, shutdownNode);
    eliteController->Start();
    Simulator::Schedule(Seconds(20.0), &GenerateDistantTraffic);

    std::cout << "=== 802.11p ELITE Simulation Setup ===" << std::endl;
    std::cout << "road_graph_ok=" << std::boolalpha << roadOk << std::endl;
    std::cout << "junctions=" << twin->GetJunctions().size() << std::endl;
    std::cout << "roads=" << twin->GetRoadSegments().size() << std::endl;
    std::cout << "gpsr_elite_attached=" << attachedGpsr << std::endl;

    Simulator::Stop(startupTime + trafficTime + Seconds(0.05));
    Simulator::Run();

    double delaySum = 0.0;
    for (double delay : g_delays)
    {
        delaySum += delay;
    }

    uint64_t ctrlBytes = 0;
    uint64_t ctrlPkts = 0;
    for (uint32_t i = 0; i < ueNodeContainer.GetN(); ++i)
    {
        Ptr<gpsr::RoutingProtocol> gpsrProtocol = GetGpsrRoutingProtocol(ueNodeContainer.Get(i));
        if (gpsrProtocol)
        {
            ctrlBytes += gpsrProtocol->GetCtrlHelloTxBytes();
            ctrlPkts += gpsrProtocol->GetCtrlHelloTxPkts();
        }
    }

    std::cout << "=== 802.11p ELITE Simulation Results ===" << std::endl;
    std::cout << "Total Tx packets = " << g_txPktCounter << std::endl;
    std::cout << "Total Rx packets = " << g_rxPktCounter << std::endl;
    std::cout << "Total Rx packets (delivered) = " << g_rxPktDelivered << std::endl;
    std::cout << "Total Rx payload bytes = " << g_rxPayloadBytes << std::endl;
    if (!g_delays.empty())
    {
        std::cout << "Average packet delay = " << delaySum / g_delays.size() << " ms"
                  << std::endl;
    }
    std::cout << "Total HELLO ctrl bytes = " << ctrlBytes << std::endl;
    std::cout << "Total HELLO ctrl pkts = " << ctrlPkts << std::endl;
    if (g_rxPayloadBytes > 0)
    {
        std::cout << "Control overhead (bytes) = "
                  << static_cast<double>(ctrlBytes) / g_rxPayloadBytes << std::endl;
    }
    if (g_rxPktDelivered > 0)
    {
        std::cout << "NRL (pkts) = " << static_cast<double>(ctrlPkts) / g_rxPktDelivered
                  << std::endl;
    }

    eliteController->Stop();
    Simulator::Destroy();
    return 0;
}
