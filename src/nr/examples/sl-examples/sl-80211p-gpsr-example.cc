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
#include "ns3/gpsr-dcc.h"
#include "ns3/gpsr-metric-supervisor.h"
#include "ns3/stats-module.h"
#include "ns3/traci-module.h"
#include "ns3/ipv4-list-routing.h"
// 802.11p/WiFi includes
#include "ns3/wifi-module.h"
#include "ns3/propagation-module.h"
#include "ns3/yans-wifi-helper.h"
// L2/L3 header helpers for unicast filtering
#include "ns3/ipv4-queue-disc-item.h"
#include "ns3/llc-snap-header.h"
#include "ns3/node-list.h"
// ARP includes for static cache population
#include "ns3/arp-l3-protocol.h"
#include "ns3/arp-cache.h"
#include "ns3/arp-header.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-interface.h"

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
double g_minDistanceForTraffic = 500.0;  //!< Minimum distance (m) for multi-hop traffic
uint32_t g_flowId = 0;                  //!< Global flow counter for unique port assignment
uint16_t g_basePort = 10000;            //!< Base port for flow-specific PacketSink

// ========== DCC Global State ==========
Ptr<ns3::gpsr::GpsrMetricSupervisor> g_metricSupervisor = nullptr;  //!< Global CBR monitor
std::unordered_map<uint32_t, Ptr<ns3::gpsr::GpsrDcc>> g_dccPerNode; //!< DCC per node

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
    InetSocketAddress dstInet = InetSocketAddress::ConvertFrom(dstAddrs);
    // Get source IP from node context
    uint32_t nodeId = Simulator::GetContext();
    Ptr<Node> node = g_ueNodeContainerPtr->Get(nodeId);
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    Ipv4Address srcIp = ipv4->GetAddress(1, 0).GetLocal();  // Interface 1, Address 0
    NS_LOG_INFO("APP-TX: UID=" << p->GetUid() << " seq=" << seqTsSizeHeader.GetSeq()
                << " src=" << srcIp << " dst=" << dstInet.GetIpv4());
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

    InetSocketAddress srcInet = InetSocketAddress::ConvertFrom(srcAddrs);
    InetSocketAddress dstInet = InetSocketAddress::ConvertFrom(dstAddrs);
    NS_LOG_INFO("APP-RX: UID=" << p->GetUid() << " seq=" << seqTsSizeHeader.GetSeq() 
                << " src=" << srcInet.GetIpv4() << " dst=" << dstInet.GetIpv4()
                << " delay=" << delay << "ms");
}

// === MAC/PHY Trace Callbacks for Unicast Debugging ===
bool TryGetNodeIdFromContext(const std::string& context, uint32_t* nodeId)
{
    std::string::size_type nodeStart = context.find("/NodeList/");
    if (nodeStart == std::string::npos)
    {
        return false;
    }
    nodeStart += 10;
    std::string::size_type nodeEnd = context.find("/", nodeStart);
    if (nodeEnd == std::string::npos || nodeEnd == nodeStart)
    {
        return false;
    }

    uint32_t id = 0;
    for (std::string::size_type i = nodeStart; i < nodeEnd; ++i)
    {
        char c = context[i];
        if (c < '0' || c > '9')
        {
            return false;
        }
        id = id * 10 + static_cast<uint32_t>(c - '0');
    }
    *nodeId = id;
    return true;
}

bool HasLlcSnapHeader(Ptr<const Packet> p)
{
    uint8_t buf[3] = {0};
    uint32_t copied = p->CopyData(buf, 3);
    return copied == 3 && buf[0] == 0xaa && buf[1] == 0xaa && buf[2] == 0x03;
}

bool ExtractIpv4Destination(Ptr<const Packet> p, Ipv4Address* dst)
{
    if (!dst)
    {
        return false;
    }

    bool hasLlc = HasLlcSnapHeader(p);
    Ptr<Packet> copy = p->Copy();

    if (hasLlc)
    {
        LlcSnapHeader llc;
        uint32_t llcSize = llc.GetSerializedSize();
        if (copy->GetSize() < llcSize)
        {
            return false;
        }
        copy->RemoveHeader(llc);
        if (llc.GetType() != 0x0800) // IPv4
        {
            return false;
        }
    }
    else
    {
        uint8_t first = 0;
        if (copy->CopyData(&first, 1) != 1 || (first >> 4) != 4)
        {
            return false;
        }
    }

    if (copy->GetSize() < 20)
    {
        return false;
    }

    Ipv4Header ip;
    copy->RemoveHeader(ip);
    *dst = ip.GetDestination();
    return true;
}

bool IsDirectedBroadcast(Ipv4Address dst, uint32_t nodeId)
{
    if (nodeId >= NodeList::GetNNodes())
    {
        return false;
    }
    Ptr<Node> node = NodeList::GetNode(nodeId);
    if (!node)
    {
        return false;
    }
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4)
    {
        return false;
    }

    for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i)
    {
        for (uint32_t j = 0; j < ipv4->GetNAddresses(i); ++j)
        {
            if (ipv4->GetAddress(i, j).GetBroadcast() == dst)
            {
                return true;
            }
        }
    }
    return false;
}

bool IsDirectedBroadcast(Ipv4Address dst, Ptr<Ipv4> ipv4, uint32_t interface)
{
    if (!ipv4 || interface >= ipv4->GetNInterfaces())
    {
        return false;
    }
    for (uint32_t j = 0; j < ipv4->GetNAddresses(interface); ++j)
    {
        if (ipv4->GetAddress(interface, j).GetBroadcast() == dst)
        {
            return true;
        }
    }
    return false;
}

bool IsUnicastIpv4Packet(Ptr<const Packet> p, uint32_t nodeId)
{
    Ipv4Address dst;
    if (!ExtractIpv4Destination(p, &dst))
    {
        return false;
    }
    if (dst == Ipv4Address::GetZero() || dst.IsMulticast() || dst.IsBroadcast())
    {
        return false;
    }
    if (IsDirectedBroadcast(dst, nodeId))
    {
        return false;
    }
    return true;
}

bool IsUnicastIpv4Packet(Ptr<const Packet> p, Ptr<Ipv4> ipv4, uint32_t interface)
{
    Ipv4Address dst;
    if (!ExtractIpv4Destination(p, &dst))
    {
        return false;
    }
    if (dst == Ipv4Address::GetZero() || dst.IsMulticast() || dst.IsBroadcast() ||
        IsDirectedBroadcast(dst, ipv4, interface))
    {
        return false;
    }
    return true;
}

bool IsUnicastIpv4QueueDiscItem(Ptr<const QueueDiscItem> item,
                                uint32_t nodeId,
                                Ipv4Address* dstOut)
{
    Ptr<const Ipv4QueueDiscItem> ipv4Item = DynamicCast<const Ipv4QueueDiscItem>(item);
    if (!ipv4Item || ipv4Item->GetProtocol() != Ipv4L3Protocol::PROT_NUMBER)
    {
        return false;
    }

    Ipv4Address dst = ipv4Item->GetHeader().GetDestination();
    if (dst == Ipv4Address::GetZero() || dst.IsMulticast() || dst.IsBroadcast() ||
        IsDirectedBroadcast(dst, nodeId))
    {
        return false;
    }

    if (dstOut)
    {
        *dstOut = dst;
    }
    return true;
}

const char*
Ipv4DropReasonToString(Ipv4L3Protocol::DropReason reason)
{
    switch (reason)
    {
    case Ipv4L3Protocol::DROP_TTL_EXPIRED:
        return "TTL_EXPIRED";
    case Ipv4L3Protocol::DROP_NO_ROUTE:
        return "NO_ROUTE";
    case Ipv4L3Protocol::DROP_BAD_CHECKSUM:
        return "BAD_CHECKSUM";
    case Ipv4L3Protocol::DROP_INTERFACE_DOWN:
        return "IFACE_DOWN";
    case Ipv4L3Protocol::DROP_ROUTE_ERROR:
        return "ROUTE_ERROR";
    case Ipv4L3Protocol::DROP_FRAGMENT_TIMEOUT:
        return "FRAG_TIMEOUT";
    case Ipv4L3Protocol::DROP_DUPLICATE:
        return "DUPLICATE";
    }
    return "UNKNOWN";
}

void
Ipv4DropCallback(std::string context,
                 const Ipv4Header& header,
                 Ptr<const Packet> p,
                 Ipv4L3Protocol::DropReason reason,
                 Ptr<Ipv4> ipv4,
                 uint32_t interface)
{
    uint32_t nodeId = 0;
    if (!TryGetNodeIdFromContext(context, &nodeId)) return;

    Ipv4Address dst = header.GetDestination();
    if (dst == Ipv4Address::GetZero() || dst.IsMulticast() || dst.IsBroadcast() ||
        IsDirectedBroadcast(dst, ipv4, interface))
    {
        return;
    }

    NS_LOG_INFO("IPV4-DROP: UID=" << p->GetUid()
                 << " node=" << nodeId
                 << " dst=" << dst
                 << " reason=" << Ipv4DropReasonToString(reason)
                 << " iface=" << interface);
}

void
Ipv4TxCallback(std::string context,
               Ptr<const Packet> p,
               Ptr<Ipv4> ipv4,
               uint32_t interface)
{
    uint32_t nodeId = 0;
    if (!TryGetNodeIdFromContext(context, &nodeId)) return;
    if (!IsUnicastIpv4Packet(p, ipv4, interface)) return;

    Ipv4Address dst;
    if (!ExtractIpv4Destination(p, &dst)) return;
    NS_LOG_INFO("IPV4-TX: UID=" << p->GetUid()
                 << " node=" << nodeId
                 << " dst=" << dst
                 << " iface=" << interface);
}

void
QueueDiscDropCallback(std::string context, Ptr<const QueueDiscItem> item)
{
    uint32_t nodeId = 0;
    if (!TryGetNodeIdFromContext(context, &nodeId)) return;

    Ipv4Address dst;
    if (!IsUnicastIpv4QueueDiscItem(item, nodeId, &dst)) return;

    NS_LOG_INFO("QDISC-DROP: UID=" << item->GetPacket()->GetUid()
                 << " node=" << nodeId
                 << " dst=" << dst);
}

void
QueueDiscDropReasonCallback(std::string context, Ptr<const QueueDiscItem> item, const char* reason)
{
    uint32_t nodeId = 0;
    if (!TryGetNodeIdFromContext(context, &nodeId)) return;

    Ipv4Address dst;
    if (!IsUnicastIpv4QueueDiscItem(item, nodeId, &dst)) return;

    NS_LOG_INFO("QDISC-DROP-REASON: UID=" << item->GetPacket()->GetUid()
                 << " node=" << nodeId
                 << " dst=" << dst
                 << " reason=" << (reason ? reason : "UNKNOWN"));
}

void
TrafficControlDropCallback(Ptr<const Packet> p)
{
    uint32_t nodeId = Simulator::GetContext();
    if (nodeId == Simulator::NO_CONTEXT) return;
    if (!IsUnicastIpv4Packet(p, nodeId)) return;

    Ipv4Address dst;
    if (!ExtractIpv4Destination(p, &dst)) return;
    NS_LOG_INFO("TC-DROP: UID=" << p->GetUid()
                 << " node=" << nodeId
                 << " dst=" << dst);
}

void
ArpCacheDropCallback(Ptr<const Packet> p)
{
    uint32_t nodeId = Simulator::GetContext();
    if (nodeId == Simulator::NO_CONTEXT) return;

    Ipv4Address dst;
    if (!ExtractIpv4Destination(p, &dst)) return;
    if (dst == Ipv4Address::GetZero() || dst.IsMulticast() || dst.IsBroadcast() ||
        IsDirectedBroadcast(dst, nodeId))
    {
        return;
    }

    NS_LOG_INFO("ARP-CACHE-DROP: UID=" << p->GetUid()
                 << " node=" << nodeId
                 << " dst=" << dst);
}

void
ArpL3DropCallback(Ptr<const Packet> p)
{
    uint32_t nodeId = Simulator::GetContext();
    if (nodeId == Simulator::NO_CONTEXT) return;

    NS_LOG_INFO("ARP-L3-DROP: UID=" << p->GetUid()
                 << " node=" << nodeId
                 << " size=" << p->GetSize());
}

void
MacTxCallback(std::string context, Ptr<const Packet> p)
{
    uint32_t nodeId = 0;
    if (!TryGetNodeIdFromContext(context, &nodeId)) return;
    if (!IsUnicastIpv4Packet(p, nodeId)) return;
    NS_LOG_INFO("MAC-TX: UID=" << p->GetUid() << " node=" << nodeId << " size=" << p->GetSize());
}

void
MacTxDropCallback(std::string context, Ptr<const Packet> p)
{
    uint32_t nodeId = 0;
    if (!TryGetNodeIdFromContext(context, &nodeId)) return;
    if (!IsUnicastIpv4Packet(p, nodeId)) return;
    NS_LOG_INFO("MAC-TX-DROP: UID=" << p->GetUid() << " node=" << nodeId << " size=" << p->GetSize());
}

void
MacRxCallback(std::string context, Ptr<const Packet> p)
{
    uint32_t nodeId = 0;
    if (!TryGetNodeIdFromContext(context, &nodeId)) return;
    if (!IsUnicastIpv4Packet(p, nodeId)) return;
    NS_LOG_INFO("MAC-RX: UID=" << p->GetUid() << " node=" << nodeId << " size=" << p->GetSize());
}

void
PhyTxDropCallback(std::string context, Ptr<const Packet> p)
{
    uint32_t nodeId = 0;
    if (!TryGetNodeIdFromContext(context, &nodeId)) return;
    if (!IsUnicastIpv4Packet(p, nodeId)) return;
    NS_LOG_INFO("PHY-TX-DROP: UID=" << p->GetUid() << " node=" << nodeId);
}

void
PhyRxDropCallback(std::string context, Ptr<const Packet> p, WifiPhyRxfailureReason reason)
{
    uint32_t nodeId = 0;
    if (!TryGetNodeIdFromContext(context, &nodeId)) return;
    if (!IsUnicastIpv4Packet(p, nodeId)) return;
    NS_LOG_INFO("PHY-RX-DROP: UID=" << p->GetUid() << " node=" << nodeId << " reason=" << reason);
}

// === Pre-populate ARP cache for all nodes ===
// This eliminates the need for runtime ARP resolution which can fail
void PopulateArpCache(const NodeContainer& nodes, const NetDeviceContainer& devices)
{
    NS_LOG_INFO("Populating ARP cache for " << nodes.GetN() << " nodes");

    if (devices.GetN() != nodes.GetN())
    {
        NS_LOG_WARN("PopulateArpCache: device count does not match node count");
        return;
    }
    
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<Node> node = nodes.Get(i);
        Ptr<Ipv4L3Protocol> ip = node->GetObject<Ipv4L3Protocol>();
        if (!ip) continue;
        
        Ptr<NetDevice> dev = devices.Get(i);
        if (!dev) continue;

        int32_t ifIndex = ip->GetInterfaceForDevice(dev);
        if (ifIndex < 0) continue;

        Ptr<Ipv4Interface> interface = ip->GetInterface(static_cast<uint32_t>(ifIndex));
        if (!interface) continue;
        
        Ptr<ArpL3Protocol> arp = node->GetObject<ArpL3Protocol>();
        if (!arp) continue;

        // Use the interface's ARP cache; create and bind one if missing.
        Ptr<ArpCache> cache = interface->GetArpCache();
        if (!cache)
        {
            cache = arp->CreateCache(dev, interface);
            interface->SetArpCache(cache);
        }
        
        // Add all other nodes to this node's ARP cache
        for (uint32_t j = 0; j < nodes.GetN(); ++j)
        {
            if (i == j) continue;  // Skip self
            
            Ptr<Node> otherNode = nodes.Get(j);
            Ptr<Ipv4L3Protocol> otherIpv4 = otherNode->GetObject<Ipv4L3Protocol>();
            if (!otherIpv4) continue;

            Ptr<NetDevice> otherDev = devices.Get(j);
            if (!otherDev) continue;

            int32_t otherIfIndex = otherIpv4->GetInterfaceForDevice(otherDev);
            if (otherIfIndex < 0) continue;

            Ipv4Address otherAddr =
                otherIpv4->GetAddress(static_cast<uint32_t>(otherIfIndex), 0).GetLocal();
            if (otherAddr == Ipv4Address::GetZero()) continue;

            Mac48Address otherMac = Mac48Address::ConvertFrom(otherDev->GetAddress());
            
            ArpCache::Entry* entry = cache->Lookup(otherAddr);
            if (!entry)
            {
                entry = cache->Add(otherAddr);
            }
            entry->SetMacAddress(otherMac);
            entry->MarkPermanent();
        }
    }
    NS_LOG_INFO("ARP cache populated for all " << nodes.GetN() << " nodes");
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
    
    // GPSR mode switch
    bool improvedGpsr = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("trafficTime", "The time traffic will be active in seconds", trafficTime);
    cmd.AddValue("packetSize", "packet size in bytes to be used by best effort traffic", udpPacketSize);
    cmd.AddValue("dataRate", "The data rate in kilobits per second for best effort traffic", dataRate);
    cmd.AddValue("testing", "Testing flag for verification", testing);
    cmd.AddValue("improvedGpsr", "Enable improved GPSR (two-hop greedy, adaptive HELLO, DCC)", improvedGpsr);

    // Parse the command line
    cmd.Parse(argc, argv);
    
    // Sync CLI params to global variables for traffic generation
    g_udpPacketSize = udpPacketSize;
    g_dataRateKbps = dataRate;

    // Final simulation time
    Time startupTime = Seconds(2.0);  // Time for SUMO/TraCI to start
    Time finalSimTime = trafficTime + startupTime + Seconds(0.05);

    // Enable GPSR debug logging - main config is below (around line 369)
    //LogComponentEnable("GpsrPositionTable", LOG_LEVEL_DEBUG);  // Disabled to reduce SINR logs

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
    LogComponentEnable("GpsrRoutingProtocol", LOG_LEVEL_INFO);    // Forward/Drop/Delivery traces
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
    NS_LOG_INFO("GPSR mode: " << (improvedGpsr ? "improved" : "original")
                 << " (UseTwoHop=" << (improvedGpsr ? "true" : "false")
                 << ", AdaptiveHello=" << (improvedGpsr ? "true" : "false")
                 << ", DCC=" << (improvedGpsr ? "true" : "false") << ")");
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
    
    // === Connect MAC/PHY Trace Callbacks for Unicast Debugging ===
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
                    MakeCallback(&MacTxCallback));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
                    MakeCallback(&MacTxDropCallback));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                    MakeCallback(&MacRxCallback));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop",
                    MakeCallback(&PhyTxDropCallback));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",
                    MakeCallback(&PhyRxDropCallback));
    NS_LOG_INFO("MAC/PHY trace callbacks connected for unicast debugging");

    // Configure internet with GPSR routing
    GpsrHelper gpsr;
    gpsr.Set("UseTwoHop", BooleanValue(improvedGpsr));
    gpsr.Set("AdaptiveHelloEnabled", BooleanValue(improvedGpsr));
    gpsr.Set("DccEnabled", BooleanValue(improvedGpsr));
    InternetStackHelper internet;
    internet.SetRoutingHelper(gpsr);
    internet.Install(ueNodeContainer);
    stream += internet.AssignStreams(ueNodeContainer, stream);
    Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Drop",
                    MakeCallback(&Ipv4DropCallback));
    Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Tx",
                    MakeCallback(&Ipv4TxCallback));
    bool tcDropOk = Config::ConnectWithoutContextFailSafe(
        "/NodeList/*/$ns3::TrafficControlLayer/TcDrop",
        MakeCallback(&TrafficControlDropCallback));
    bool qdiscDropOk = Config::ConnectFailSafe(
        "/NodeList/*/$ns3::TrafficControlLayer/RootQueueDiscList/*/Drop",
        MakeCallback(&QueueDiscDropCallback));
    bool qdiscDbeOk = Config::ConnectFailSafe(
        "/NodeList/*/$ns3::TrafficControlLayer/RootQueueDiscList/*/DropBeforeEnqueue",
        MakeCallback(&QueueDiscDropReasonCallback));
    bool qdiscDadOk = Config::ConnectFailSafe(
        "/NodeList/*/$ns3::TrafficControlLayer/RootQueueDiscList/*/DropAfterDequeue",
        MakeCallback(&QueueDiscDropReasonCallback));
    bool arpL3DropOk = Config::ConnectWithoutContextFailSafe(
        "/NodeList/*/$ns3::ArpL3Protocol/Drop",
        MakeCallback(&ArpL3DropCallback));
    bool arpCacheDropOk = Config::ConnectWithoutContextFailSafe(
        "/NodeList/*/$ns3::ArpL3Protocol/CacheList/*/Drop",
        MakeCallback(&ArpCacheDropCallback));
    if (!qdiscDropOk && !qdiscDbeOk && !qdiscDadOk)
    {
        NS_LOG_WARN("QueueDisc drop traces not connected (no root queue disc?)");
    }
    if (!tcDropOk)
    {
        NS_LOG_WARN("TrafficControlLayer TcDrop trace not connected");
    }
    if (!arpL3DropOk)
    {
        NS_LOG_WARN("ArpL3Protocol Drop trace not connected");
    }
    if (!arpCacheDropOk)
    {
        NS_LOG_WARN("ArpCache Drop trace not connected");
    }

    // Wire up GPSR m_downTarget callback chain for header insertion
    // This must be called AFTER InternetStackHelper.Install()
    gpsr.Install(ueNodeContainer);
    NS_LOG_INFO("GPSR routing protocol installed on all UEs");
    
    // ========== DCC Initialization ==========
    if (improvedGpsr)
    {
        // Create global MetricSupervisor for CBR monitoring
        g_metricSupervisor = CreateObject<ns3::gpsr::GpsrMetricSupervisor>();
        g_metricSupervisor->SetNodeContainer(ueNodeContainer);
        g_metricSupervisor->SetChannelTechnology("80211p");
        g_metricSupervisor->SetCBRWindow(100);   // 100ms CBR window
        g_metricSupervisor->SetCBRAlpha(0.5);    // Exponential moving average alpha
        g_metricSupervisor->SetSimulationTime(finalSimTime.GetSeconds());
        g_metricSupervisor->StartCheckCBR(-1);   // Monitor all nodes
        NS_LOG_INFO("DCC MetricSupervisor started for CBR monitoring");
        
        // Create DCC instance for each node and link to GPSR
        for (uint32_t i = 0; i < ueNodeContainer.GetN(); ++i)
        {
            Ptr<Node> node = ueNodeContainer.Get(i);
            
            // Create DCC for this node
            Ptr<ns3::gpsr::GpsrDcc> dcc = CreateObject<ns3::gpsr::GpsrDcc>();
            std::string nodeIdStr = std::to_string(node->GetId());
            dcc->SetupDCC(nodeIdStr, g_metricSupervisor, node, "adaptive", 100);  // 100ms DCC interval
            dcc->SetBitRate(6000000);  // 6 Mbps (802.11p OFDM Rate)
            dcc->StartDCC();
            g_dccPerNode[i] = dcc;
            
            // Link DCC to GPSR routing protocol
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
            
            if (gpsrProto)
            {
                // Set DCC and MetricSupervisor in GPSR protocol
                gpsrProto->SetDcc(dcc);
                gpsrProto->SetMetricSupervisor(g_metricSupervisor);
            }
        }
        NS_LOG_INFO("DCC initialized for " << ueNodeContainer.GetN() << " nodes");
    }
    else
    {
        NS_LOG_INFO("GPSR original mode: adaptive HELLO/DCC/two-hop disabled");
    }

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

    // Pre-populate ARP cache for all nodes to avoid ARP resolution delays/failures
    PopulateArpCache(ueNodeContainer, ueNetDev);
    
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
