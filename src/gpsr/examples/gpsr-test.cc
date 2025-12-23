/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Test Script - Based on original gpsr-test1.cc
 * Tests GPSR routing with WiFi ad-hoc network
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/gpsr-helper.h"
#include "ns3/gpsr.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("GpsrTest");

// Global counters for packets
uint32_t g_txPackets = 0;
uint32_t g_rxPackets = 0;

// Callbacks
void
TxCallback(Ptr<const Packet> p)
{
    g_txPackets++;
    NS_LOG_INFO("TX packet " << p->GetUid() << " size " << p->GetSize());
}

void
RxCallback(Ptr<const Packet> p)
{
    g_rxPackets++;
    NS_LOG_INFO("RX packet " << p->GetUid() << " size " << p->GetSize());
}

class GpsrExample
{
  public:
    GpsrExample();
    bool Configure(int argc, char** argv);
    void Run();
    void Report(std::ostream& os);

  private:
    uint32_t m_size;       // Number of nodes
    uint32_t m_gridWidth;  // Width of the Node Grid
    double m_step;         // Distance between nodes, meters
    double m_totalTime;    // Simulation time, seconds
    bool m_pcap;           // Write per-device PCAP traces if true
    bool m_verbose;        // Enable verbose logging

    NodeContainer m_nodes;
    NetDeviceContainer m_devices;
    Ipv4InterfaceContainer m_interfaces;

    void CreateNodes();
    void CreateDevices();
    void InstallInternetStack();
    void InstallApplications();
};

int
main(int argc, char** argv)
{
    GpsrExample test;
    if (!test.Configure(argc, argv))
    {
        NS_FATAL_ERROR("Configuration failed. Aborted.");
    }

    test.Run();
    test.Report(std::cout);
    return 0;
}

GpsrExample::GpsrExample()
    : m_size(4),
      m_gridWidth(2),
      m_step(80),
      m_totalTime(30),
      m_pcap(false),
      m_verbose(false)
{
}

bool
GpsrExample::Configure(int argc, char** argv)
{
    SeedManager::SetSeed(12345);
    CommandLine cmd;

    cmd.AddValue("pcap", "Write PCAP traces.", m_pcap);
    cmd.AddValue("size", "Number of nodes.", m_size);
    cmd.AddValue("time", "Simulation time, s.", m_totalTime);
    cmd.AddValue("step", "Grid step, m", m_step);
    cmd.AddValue("verbose", "Enable verbose logging", m_verbose);

    cmd.Parse(argc, argv);

    if (m_verbose)
    {
        LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_DEBUG);
        LogComponentEnable("GpsrPositionTable", LOG_LEVEL_DEBUG);
        LogComponentEnable("GpsrTest", LOG_LEVEL_INFO);
    }

    return true;
}

void
GpsrExample::Run()
{
    CreateNodes();
    CreateDevices();
    InstallInternetStack();
    InstallApplications();

    std::cout << "Starting simulation for " << m_totalTime << " s with " << m_size
              << " nodes...\n";
    std::cout << "Grid: " << m_gridWidth << "x" << (m_size / m_gridWidth) << ", step: " << m_step
              << " m\n";

    Simulator::Stop(Seconds(m_totalTime));
    Simulator::Run();
    Simulator::Destroy();
}

void
GpsrExample::Report(std::ostream& os)
{
    os << "\n=== GPSR Test Results ===\n";
    os << "Packets sent: " << g_txPackets << "\n";
    os << "Packets received: " << g_rxPackets << "\n";
    if (g_txPackets > 0)
    {
        os << "Packet delivery ratio: " << (100.0 * g_rxPackets / g_txPackets) << "%\n";
    }
    os << "=========================\n";
}

void
GpsrExample::CreateNodes()
{
    std::cout << "Creating " << m_size << " nodes " << m_step << " m apart.\n";
    m_nodes.Create(m_size);

    // Name nodes
    for (uint32_t i = 0; i < m_size; ++i)
    {
        std::ostringstream os;
        os << "node-" << i;
        Names::Add(os.str(), m_nodes.Get(i));
    }

    // Create static grid
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX",
                                  DoubleValue(0.0),
                                  "MinY",
                                  DoubleValue(0.0),
                                  "DeltaX",
                                  DoubleValue(m_step),
                                  "DeltaY",
                                  DoubleValue(m_step),
                                  "GridWidth",
                                  UintegerValue(m_gridWidth),
                                  "LayoutType",
                                  StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(m_nodes);

    // Print node positions
    for (uint32_t i = 0; i < m_size; ++i)
    {
        Ptr<MobilityModel> mm = m_nodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mm->GetPosition();
        std::cout << "  Node " << i << " at (" << pos.x << ", " << pos.y << ")\n";
    }
}

void
GpsrExample::CreateDevices()
{
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("OfdmRate6Mbps"),
                                 "RtsCtsThreshold",
                                 UintegerValue(0));

    m_devices = wifi.Install(wifiPhy, wifiMac, m_nodes);

    if (m_pcap)
    {
        wifiPhy.EnablePcapAll(std::string("gpsr-test"));
    }
}

void
GpsrExample::InstallInternetStack()
{
    GpsrHelper gpsr;

    InternetStackHelper stack;
    stack.SetRoutingHelper(gpsr);
    stack.Install(m_nodes);

    // Wire up GPSR m_downTarget callback chain for header insertion
    // This must be called AFTER InternetStackHelper.Install() 
    // because UDP protocol needs to be installed first
    gpsr.Install(m_nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.0.0");
    m_interfaces = address.Assign(m_devices);

    // Print IP addresses
    for (uint32_t i = 0; i < m_size; ++i)
    {
        std::cout << "  Node " << i << " IP: " << m_interfaces.GetAddress(i) << "\n";
    }
}

void
GpsrExample::InstallApplications()
{
    uint16_t port = 9;
    uint32_t packetSize = 512;
    uint32_t maxPacketCount = 20;
    Time interPacketInterval = Seconds(1.0);

    // Server on last node (bottom-right of grid)
    uint32_t serverNode = m_size - 1;
    UdpEchoServerHelper server(port);
    ApplicationContainer serverApps = server.Install(m_nodes.Get(serverNode));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(m_totalTime - 0.1));

    // Client on first node (top-left of grid)
    uint32_t clientNode = 0;
    UdpEchoClientHelper client(m_interfaces.GetAddress(serverNode), port);
    client.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
    client.SetAttribute("Interval", TimeValue(interPacketInterval));
    client.SetAttribute("PacketSize", UintegerValue(packetSize));

    ApplicationContainer clientApps = client.Install(m_nodes.Get(clientNode));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(m_totalTime - 0.1));

    // Connect callbacks with correct signature
    Ptr<UdpEchoClient> echoClient = clientApps.Get(0)->GetObject<UdpEchoClient>();
    echoClient->TraceConnectWithoutContext("Tx", MakeCallback(&TxCallback));

    Ptr<UdpEchoServer> echoServer = serverApps.Get(0)->GetObject<UdpEchoServer>();
    echoServer->TraceConnectWithoutContext("Rx", MakeCallback(&RxCallback));

    std::cout << "Applications:\n";
    std::cout << "  Client: Node " << clientNode << " (" << m_interfaces.GetAddress(clientNode)
              << ")\n";
    std::cout << "  Server: Node " << serverNode << " (" << m_interfaces.GetAddress(serverNode)
              << ")\n";
    std::cout << "  Packets: " << maxPacketCount << " x " << packetSize << " bytes\n";
}
