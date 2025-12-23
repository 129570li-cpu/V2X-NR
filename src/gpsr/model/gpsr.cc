/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Routing Protocol - Complete Implementation
 * Based on original GPSR implementation
 */

#define NS_LOG_APPEND_CONTEXT                                                                      \
    if (m_ipv4)                                                                                    \
    {                                                                                              \
        std::clog << "[node " << m_ipv4->GetObject<Node>()->GetId() << "] ";                       \
    }

#include "gpsr.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/inet-socket-address.h"
#include "ns3/log.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("GpsrRoutingProtocol");

namespace gpsr
{

#define GPSR_LS_GOD 0
#define GPSR_LS_RLS 1

/// Maximum allowed jitter
#define GPSR_MAXJITTER (m_helloInterval.GetSeconds() / 2)

// DeferredRouteOutputTag implementation
NS_OBJECT_ENSURE_REGISTERED(DeferredRouteOutputTag);

TypeId
DeferredRouteOutputTag::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::gpsr::DeferredRouteOutputTag").SetParent<Tag>().SetGroupName("Gpsr");
    return tid;
}

TypeId
DeferredRouteOutputTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
DeferredRouteOutputTag::GetSerializedSize() const
{
    return sizeof(uint32_t);
}

void
DeferredRouteOutputTag::Serialize(TagBuffer i) const
{
    i.WriteU32(m_isCallFromL3);
}

void
DeferredRouteOutputTag::Deserialize(TagBuffer i)
{
    m_isCallFromL3 = i.ReadU32();
}

void
DeferredRouteOutputTag::Print(std::ostream& os) const
{
    os << "DeferredRouteOutputTag: m_isCallFromL3 = " << m_isCallFromL3;
}

// GpsrDataPacketTag implementation
NS_OBJECT_ENSURE_REGISTERED(GpsrDataPacketTag);

TypeId
GpsrDataPacketTag::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::gpsr::GpsrDataPacketTag").SetParent<Tag>().SetGroupName("Gpsr");
    return tid;
}

TypeId
GpsrDataPacketTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
GpsrDataPacketTag::GetSerializedSize() const
{
    return 0;  // No data, just presence indicates GPSR data packet
}

void
GpsrDataPacketTag::Serialize(TagBuffer) const
{
    // No data to serialize
}

void
GpsrDataPacketTag::Deserialize(TagBuffer)
{
    // No data to deserialize
}

void
GpsrDataPacketTag::Print(std::ostream& os) const
{
    os << "GpsrDataPacketTag";
}

// RoutingProtocol implementation
NS_OBJECT_ENSURE_REGISTERED(RoutingProtocol);

const uint32_t RoutingProtocol::GPSR_PORT = 666;

TypeId
RoutingProtocol::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::gpsr::RoutingProtocol")
            .SetParent<Ipv4RoutingProtocol>()
            .SetGroupName("Gpsr")
            .AddConstructor<RoutingProtocol>()
            .AddAttribute("HelloInterval",
                          "HELLO messages emission interval.",
                          TimeValue(Seconds(1)),
                          MakeTimeAccessor(&RoutingProtocol::m_helloInterval),
                          MakeTimeChecker())
            .AddAttribute("LocationServiceName",
                          "Indicates which Location Service to use (0=GOD, 1=RLS)",
                          UintegerValue(GPSR_LS_GOD),
                          MakeUintegerAccessor(&RoutingProtocol::m_locationServiceName),
                          MakeUintegerChecker<uint8_t>())
            .AddAttribute("PerimeterMode",
                          "Indicates if PerimeterMode (recovery mode) is enabled",
                          BooleanValue(true),
                          MakeBooleanAccessor(&RoutingProtocol::m_perimeterMode),
                          MakeBooleanChecker());
    return tid;
}

RoutingProtocol::RoutingProtocol()
    : m_helloInterval(Seconds(1)),
      m_maxQueueLen(64),
      m_maxQueueTime(Seconds(30)),
      m_queue(m_maxQueueLen, m_maxQueueTime),
      m_helloIntervalTimer(Timer::CANCEL_ON_DESTROY),
      m_checkQueueTimer(Timer::CANCEL_ON_DESTROY),
      m_perimeterMode(true),
      m_locationServiceName(GPSR_LS_GOD)
{
    NS_LOG_FUNCTION(this);
}

RoutingProtocol::~RoutingProtocol()
{
    NS_LOG_FUNCTION(this);
}

void
RoutingProtocol::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_ipv4 = nullptr;
    for (auto& s : m_socketAddresses)
    {
        s.first->Close();
    }
    m_socketAddresses.clear();
    Ipv4RoutingProtocol::DoDispose();
}

Ptr<LocationService>
RoutingProtocol::GetLS()
{
    return m_locationService;
}

void
RoutingProtocol::SetLS(Ptr<LocationService> locationService)
{
    m_locationService = locationService;
}

void
RoutingProtocol::Start()
{
    NS_LOG_FUNCTION(this);
    m_queuedAddresses.clear();

    // Initialize location service
    switch (m_locationServiceName)
    {
    case GPSR_LS_GOD:
        NS_LOG_DEBUG("Using GodLocationService");
        m_locationService = CreateObject<GodLocationService>();
        break;
    default:
        NS_LOG_DEBUG("Using GodLocationService (default)");
        m_locationService = CreateObject<GodLocationService>();
        break;
    }
}

void
RoutingProtocol::SetIpv4(Ptr<Ipv4> ipv4)
{
    NS_LOG_FUNCTION(this << ipv4);
    NS_ASSERT(ipv4);
    NS_ASSERT(!m_ipv4);

    m_ipv4 = ipv4;

    // Setup hello timer with jitter
    m_helloIntervalTimer.SetFunction(&RoutingProtocol::HelloTimerExpire, this);

    Ptr<UniformRandomVariable> jitter = CreateObject<UniformRandomVariable>();
    jitter->SetAttribute("Min", DoubleValue(0));
    jitter->SetAttribute("Max", DoubleValue(GPSR_MAXJITTER));

    m_helloIntervalTimer.Schedule(Seconds(jitter->GetValue()));

    // Setup queue check timer
    m_checkQueueTimer.SetFunction(&RoutingProtocol::CheckQueue, this);

    Simulator::ScheduleNow(&RoutingProtocol::Start, this);
}

void
RoutingProtocol::HelloTimerExpire()
{
    NS_LOG_FUNCTION(this);
    SendHello();

    Ptr<UniformRandomVariable> jitter = CreateObject<UniformRandomVariable>();
    jitter->SetAttribute("Min", DoubleValue(-GPSR_MAXJITTER));
    jitter->SetAttribute("Max", DoubleValue(GPSR_MAXJITTER));

    m_helloIntervalTimer.Schedule(m_helloInterval + Seconds(jitter->GetValue()));
}

void
RoutingProtocol::SendHello()
{
    NS_LOG_FUNCTION(this);

    Ptr<MobilityModel> mm = m_ipv4->GetObject<MobilityModel>();
    if (!mm)
    {
        NS_LOG_WARN("No mobility model, cannot send hello");
        return;
    }

    Vector pos = mm->GetPosition();

    for (auto& s : m_socketAddresses)
    {
        Ptr<Socket> socket = s.first;
        Ipv4InterfaceAddress iface = s.second;

        HelloHeader helloHeader((uint64_t)pos.x, (uint64_t)pos.y);

        Ptr<Packet> packet = Create<Packet>();
        packet->AddHeader(helloHeader);
        TypeHeader tHeader(GPSRTYPE_HELLO);
        packet->AddHeader(tHeader);

        Ipv4Address destination;
        if (iface.GetMask() == Ipv4Mask::GetOnes())
        {
            destination = Ipv4Address("255.255.255.255");
        }
        else
        {
            destination = iface.GetBroadcast();
        }

        socket->SendTo(packet, 0, InetSocketAddress(destination, GPSR_PORT));
        NS_LOG_DEBUG("Sent HELLO from " << iface.GetLocal() << " to " << destination);
    }
}

void
RoutingProtocol::RecvGpsr(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    Address sourceAddress;
    Ptr<Packet> packet = socket->RecvFrom(sourceAddress);

    NS_LOG_DEBUG("Received packet size: " << packet->GetSize());

    // Check if packet is large enough for TypeHeader (1 byte)
    if (packet->GetSize() < 1)
    {
        NS_LOG_DEBUG("Packet too small for TypeHeader");
        return;
    }

    TypeHeader tHeader(GPSRTYPE_HELLO);
    packet->RemoveHeader(tHeader);
    if (!tHeader.IsValid())
    {
        NS_LOG_DEBUG("Unknown GPSR message type received: " << (int)tHeader.Get());
        return;
    }

    if (tHeader.Get() == GPSRTYPE_HELLO)
    {
        // Check if remaining packet is large enough for HelloHeader (16 bytes)
        if (packet->GetSize() < 16)
        {
            NS_LOG_DEBUG("Packet too small for HelloHeader, size: " << packet->GetSize());
            return;
        }

        HelloHeader hdr;
        packet->RemoveHeader(hdr);

        Vector pos;
        pos.x = hdr.GetOriginPosx();
        pos.y = hdr.GetOriginPosy();

        InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom(sourceAddress);
        Ipv4Address sender = inetSourceAddr.GetIpv4();
        Ipv4Address receiver = m_socketAddresses[socket].GetLocal();

        NS_LOG_DEBUG("HELLO from " << sender << " position (" << pos.x << ", " << pos.y << ")");
        UpdateRouteToNeighbor(sender, receiver, pos);
    }
}

void
RoutingProtocol::UpdateRouteToNeighbor(Ipv4Address sender, Ipv4Address receiver, Vector pos)
{
    NS_LOG_FUNCTION(this << sender << receiver << pos);
    m_neighbors.AddEntry(sender, pos);
}

void
RoutingProtocol::NotifyInterfaceUp(uint32_t interface)
{
    NS_LOG_FUNCTION(this << interface);

    Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol>();
    if (l3->GetNAddresses(interface) > 1)
    {
        NS_LOG_WARN("GPSR does not work with more than one address per interface");
    }

    Ipv4InterfaceAddress iface = l3->GetAddress(interface, 0);
    if (iface.GetLocal() == Ipv4Address("127.0.0.1"))
    {
        return;
    }

    // Create a socket to listen only on this interface
    Ptr<Socket> socket = Socket::CreateSocket(GetObject<Node>(), UdpSocketFactory::GetTypeId());
    NS_ASSERT(socket);
    socket->SetRecvCallback(MakeCallback(&RoutingProtocol::RecvGpsr, this));
    socket->BindToNetDevice(l3->GetNetDevice(interface));
    socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), GPSR_PORT));
    socket->SetAllowBroadcast(true);
    socket->SetIpRecvTtl(true);  // Enable receiving TTL info
    socket->SetAttribute("IpTtl", UintegerValue(1));
    m_socketAddresses.insert(std::make_pair(socket, iface));

    NS_LOG_DEBUG("Interface " << interface << " (" << iface.GetLocal() << ") is up, socket bound to port " << GPSR_PORT);
}

void
RoutingProtocol::NotifyInterfaceDown(uint32_t interface)
{
    NS_LOG_FUNCTION(this << interface);

    Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol>();
    Ipv4InterfaceAddress iface = l3->GetAddress(interface, 0);

    Ptr<Socket> socket = FindSocketWithInterfaceAddress(iface);
    if (socket)
    {
        socket->Close();
        m_socketAddresses.erase(socket);
    }

    if (m_socketAddresses.empty())
    {
        m_neighbors.Clear();
        if (m_locationService)
        {
            m_locationService->Clear();
        }
    }
}

void
RoutingProtocol::NotifyAddAddress(uint32_t interface, Ipv4InterfaceAddress address)
{
    NS_LOG_FUNCTION(this << interface << address);

    Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol>();
    if (!l3->IsUp(interface))
    {
        return;
    }

    Ipv4InterfaceAddress iface = l3->GetAddress(interface, 0);
    Ptr<Socket> socket = FindSocketWithInterfaceAddress(iface);
    if (!socket)
    {
        if (iface.GetLocal() == Ipv4Address("127.0.0.1"))
        {
            return;
        }
        // Create a socket for this new address
        Ptr<Socket> newSocket = Socket::CreateSocket(GetObject<Node>(), UdpSocketFactory::GetTypeId());
        NS_ASSERT(newSocket);
        newSocket->SetRecvCallback(MakeCallback(&RoutingProtocol::RecvGpsr, this));
        newSocket->BindToNetDevice(l3->GetNetDevice(interface));
        newSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), GPSR_PORT));
        newSocket->SetAllowBroadcast(true);
        newSocket->SetAttribute("IpTtl", UintegerValue(1));
        m_socketAddresses.insert(std::make_pair(newSocket, iface));
        NS_LOG_DEBUG("Added socket for new address " << address.GetLocal());
    }
}

void
RoutingProtocol::NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address)
{
    NS_LOG_FUNCTION(this << interface << address);

    Ptr<Socket> socket = FindSocketWithInterfaceAddress(address);
    if (socket)
    {
        m_socketAddresses.erase(socket);
        socket->Close();
    }
}

Ptr<Socket>
RoutingProtocol::FindSocketWithInterfaceAddress(Ipv4InterfaceAddress addr) const
{
    NS_LOG_FUNCTION(this << addr);
    for (auto& s : m_socketAddresses)
    {
        if (s.second == addr)
        {
            return s.first;
        }
    }
    return nullptr;
}

bool
RoutingProtocol::IsMyOwnAddress(Ipv4Address src)
{
    NS_LOG_FUNCTION(this << src);
    for (auto& s : m_socketAddresses)
    {
        if (s.second.GetLocal() == src)
        {
            return true;
        }
    }
    return false;
}

Ptr<Ipv4Route>
RoutingProtocol::RouteOutput(Ptr<Packet> p,
                             const Ipv4Header& header,
                             Ptr<NetDevice> oif,
                             Socket::SocketErrno& sockerr)
{
    NS_LOG_FUNCTION(this << header << (oif ? oif->GetIfIndex() : 0));

    if (!p)
    {
        return LoopbackRoute(header, oif);
    }

    if (m_socketAddresses.empty())
    {
        sockerr = Socket::ERROR_NOROUTETOHOST;
        NS_LOG_LOGIC("No gpsr interfaces");
        return Ptr<Ipv4Route>();
    }

    sockerr = Socket::ERROR_NOTERROR;
    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    Ipv4Address dst = header.GetDestination();

    // Special handling for broadcast - send directly without routing
    Ipv4Address broadcast = m_ipv4->GetAddress(1, 0).GetBroadcast();
    if (dst == broadcast || dst == Ipv4Address("255.255.255.255"))
    {
        route->SetDestination(dst);
        route->SetSource(m_ipv4->GetAddress(1, 0).GetLocal());
        route->SetGateway(dst);  // Broadcast gateway is the broadcast address itself
        route->SetOutputDevice(m_ipv4->GetNetDevice(1));
        NS_LOG_DEBUG("Broadcast route to " << dst);
        return route;
    }

    Vector dstPos = Vector(1, 0, 0);

    // Get destination position
    dstPos = m_locationService->GetPosition(dst);

    // Check if position is invalid and still searching
    if (CalculateDistance(dstPos, m_locationService->GetInvalidPosition()) == 0 &&
        m_locationService->IsInSearch(dst))
    {
        DeferredRouteOutputTag tag;
        if (!p->PeekPacketTag(tag))
        {
            p->AddPacketTag(tag);
        }
        return LoopbackRoute(header, oif);
    }

    // Get my position
    Ptr<MobilityModel> mm = m_ipv4->GetObject<MobilityModel>();
    if (!mm)
    {
        sockerr = Socket::ERROR_NOROUTETOHOST;
        return Ptr<Ipv4Route>();
    }
    Vector myPos = mm->GetPosition();

    Ipv4Address nextHop;

    // Check if destination is a neighbor
    if (m_neighbors.IsNeighbour(dst))
    {
        nextHop = dst;
    }
    else
    {
        nextHop = m_neighbors.BestNeighbor(dstPos, myPos);
    }

    if (nextHop != Ipv4Address::GetZero())
    {
        NS_LOG_DEBUG("Destination: " << dst);

        route->SetDestination(dst);
        // Check for uninitialized source address (0.0.0.0 or legacy 102.102.102.102)
        Ipv4Address srcAddr = header.GetSource();
        if (srcAddr == Ipv4Address::GetZero() || srcAddr == Ipv4Address("102.102.102.102"))
        {
            route->SetSource(m_ipv4->GetAddress(1, 0).GetLocal());
        }
        else
        {
            route->SetSource(srcAddr);
        }
        route->SetGateway(nextHop);
        
        int32_t ifIndex = m_ipv4->GetInterfaceForAddress(route->GetSource());
        if (ifIndex < 0)
        {
            route->SetOutputDevice(m_ipv4->GetNetDevice(1));
        }
        else
        {
            route->SetOutputDevice(m_ipv4->GetNetDevice(static_cast<uint32_t>(ifIndex)));
        }

        NS_ASSERT(route);
        NS_LOG_DEBUG("Route to " << route->GetDestination() << " from " << route->GetSource());
        NS_LOG_DEBUG("GPSR RouteOutput: NextHop calculated: " << route->GetGateway());

        // Check oif BEFORE returning route
        if (oif && route->GetOutputDevice() != oif)
        {
            NS_LOG_DEBUG("Output device doesn't match. Dropped.");
            sockerr = Socket::ERROR_NOROUTETOHOST;
            return Ptr<Ipv4Route>();
        }

        // Note: GPSR headers are added via the m_downTarget callback chain
        // (UDP -> GPSR::AddHeaders -> IP) configured by GpsrHelper::Install()
        return route;
    }
    else
    {
        // No next hop found, defer and enter recovery mode in RouteInput
        DeferredRouteOutputTag tag;
        if (!p->PeekPacketTag(tag))
        {
            p->AddPacketTag(tag);
        }
        return LoopbackRoute(header, oif);
    }
}

Ptr<Ipv4Route>
RoutingProtocol::LoopbackRoute(const Ipv4Header& hdr, Ptr<NetDevice> oif)
{
    NS_LOG_FUNCTION(this << hdr);

    m_lo = m_ipv4->GetNetDevice(0);
    NS_ASSERT(m_lo);

    Ptr<Ipv4Route> rt = Create<Ipv4Route>();
    rt->SetDestination(hdr.GetDestination());

    auto j = m_socketAddresses.begin();
    if (oif)
    {
        for (j = m_socketAddresses.begin(); j != m_socketAddresses.end(); ++j)
        {
            Ipv4Address addr = j->second.GetLocal();
            int32_t interface = m_ipv4->GetInterfaceForAddress(addr);
            if (oif == m_ipv4->GetNetDevice(static_cast<uint32_t>(interface)))
            {
                rt->SetSource(addr);
                break;
            }
        }
    }
    else
    {
        rt->SetSource(j->second.GetLocal());
    }

    NS_ASSERT_MSG(rt->GetSource() != Ipv4Address(), "Valid GPSR source address not found");
    rt->SetGateway(Ipv4Address("127.0.0.1"));
    rt->SetOutputDevice(m_lo);

    return rt;
}

bool
RoutingProtocol::RouteInput(Ptr<const Packet> p,
                            const Ipv4Header& header,
                            Ptr<const NetDevice> idev,
                            const UnicastForwardCallback& ucb,
                            const MulticastForwardCallback& mcb,
                            const LocalDeliverCallback& lcb,
                            const ErrorCallback& ecb)
{
    NS_LOG_FUNCTION(this << p->GetUid() << header.GetDestination() << idev->GetAddress());

    if (m_socketAddresses.empty())
    {
        NS_LOG_LOGIC("No gpsr interfaces");
        return false;
    }

    NS_ASSERT(m_ipv4);
    NS_ASSERT(p);
    NS_ASSERT(m_ipv4->GetInterfaceForDevice(idev) >= 0);

    int32_t iif = m_ipv4->GetInterfaceForDevice(idev);
    Ipv4Address dst = header.GetDestination();
    Ipv4Address origin = header.GetSource();

    // Check for deferred route output (from loopback)
    DeferredRouteOutputTag tag;
    if (p->PeekPacketTag(tag) && IsMyOwnAddress(origin))
    {
        Ptr<Packet> packet = p->Copy();
        packet->RemovePacketTag(tag);
        DeferredRouteOutput(packet, header, ucb, ecb);
        return true;
    }

    // Local delivery check
    if (m_ipv4->IsDestinationAddress(dst, iif))
    {
        Ptr<Packet> packet = p->Copy();

        // Remove GPSR headers if this is a data packet (not HELLO)
        // Data packets have TypeHeader(GPSRTYPE_POS) + PositionHeader (54+ bytes)
        if (packet->GetSize() >= 54)
        {
            TypeHeader tHeader(GPSRTYPE_POS);
            packet->RemoveHeader(tHeader);
            if (tHeader.IsValid() && tHeader.Get() == GPSRTYPE_POS)
            {
                PositionHeader phdr;
                packet->RemoveHeader(phdr);
                NS_LOG_DEBUG("Removed GPSR headers for local delivery");
            }
            else
            {
                // Not a GPSRTYPE_POS, restore original packet
                packet = p->Copy();
            }
        }

        if (dst != m_ipv4->GetAddress(1, 0).GetBroadcast())
        {
            NS_LOG_LOGIC("Unicast local delivery to " << dst);
        }

        lcb(packet, header, iif);
        return true;
    }

    // Forward the packet
    return Forwarding(p, header, ucb, ecb);
}

bool
RoutingProtocol::Forwarding(Ptr<const Packet> packet,
                            const Ipv4Header& header,
                            const UnicastForwardCallback& ucb,
                            const ErrorCallback& ecb)
{
    NS_LOG_FUNCTION(this);

    Ptr<Packet> p = packet->Copy();
    Ipv4Address dst = header.GetDestination();
    Ipv4Address origin = header.GetSource();

    m_neighbors.Purge();

    // Parse GPSR headers
    TypeHeader tHeader(GPSRTYPE_POS);
    PositionHeader hdr;
    uint32_t updated = 0;
    Vector Position;
    Vector RecPosition;
    uint8_t inRec = 0;

    NS_LOG_DEBUG("Forwarding packet size before RemoveHeader: " << p->GetSize());
    p->RemoveHeader(tHeader);

    if (!tHeader.IsValid())
    {
        NS_LOG_DEBUG("GPSR message " << p->GetUid() << " with unknown type (type byte value was not 1 or 2). Packet size after RemoveHeader: " << p->GetSize() << ". Drop");
        return false;
    }

    if (tHeader.Get() == GPSRTYPE_POS)
    {
        p->RemoveHeader(hdr);
        Position.x = hdr.GetDstPosx();
        Position.y = hdr.GetDstPosy();
        updated = hdr.GetUpdated();
        RecPosition.x = hdr.GetRecPosx();
        RecPosition.y = hdr.GetRecPosy();
        inRec = hdr.GetInRec();
    }

    // Get my position
    Ptr<MobilityModel> mm = m_ipv4->GetObject<MobilityModel>();
    if (!mm)
    {
        NS_LOG_WARN("No mobility model");
        return false;
    }
    Vector myPos = mm->GetPosition();

    // Check if we can exit recovery mode
    if (inRec == 1 && CalculateDistance(myPos, Position) < CalculateDistance(RecPosition, Position))
    {
        inRec = 0;
        hdr.SetInRec(0);
        NS_LOG_LOGIC("No longer in Recovery to " << dst << " at " << myPos);
    }

    // If still in recovery mode, use perimeter forwarding
    if (inRec)
    {
        p->AddHeader(hdr);
        p->AddHeader(tHeader);
        RecoveryMode(dst, p, ucb, header);
        return true;
    }

    // Check if we have a newer position for destination
    uint32_t myUpdated = (uint32_t)m_locationService->GetEntryUpdateTime(dst).GetSeconds();
    if (myUpdated > updated)
    {
        Position = m_locationService->GetPosition(dst);
        updated = myUpdated;
    }

    // Find best neighbor (greedy forwarding)
    Ipv4Address nextHop = m_neighbors.BestNeighbor(Position, myPos);

    if (nextHop != Ipv4Address::GetZero())
    {
        // Greedy forwarding successful
        PositionHeader posHeader(Position.x,
                                 Position.y,
                                 updated,
                                 (uint64_t)0,
                                 (uint64_t)0,
                                 (uint8_t)0,
                                 myPos.x,
                                 myPos.y);
        p->AddHeader(posHeader);
        p->AddHeader(tHeader);

        Ptr<Ipv4Route> route = Create<Ipv4Route>();
        route->SetDestination(dst);
        route->SetSource(header.GetSource());
        route->SetGateway(nextHop);
        route->SetOutputDevice(m_ipv4->GetNetDevice(1));

        NS_LOG_LOGIC("Forwarding to " << dst << " from " << origin << " via " << nextHop);
        ucb(route, p, header);
        return true;
    }

    // No greedy next hop - enter recovery mode
    if (m_perimeterMode)
    {
        hdr.SetInRec(1);
        hdr.SetRecPosx(myPos.x);
        hdr.SetRecPosy(myPos.y);
        hdr.SetLastPosx(Position.x);
        hdr.SetLastPosy(Position.y);

        p->AddHeader(hdr);
        p->AddHeader(tHeader);

        NS_LOG_LOGIC("Entering recovery-mode to " << dst << " at "
                                                  << m_ipv4->GetAddress(1, 0).GetLocal());
        RecoveryMode(dst, p, ucb, header);
        return true;
    }

    NS_LOG_DEBUG("No route to " << dst);
    return false;
}

void
RoutingProtocol::RecoveryMode(Ipv4Address dst,
                              Ptr<Packet> p,
                              const UnicastForwardCallback& ucb,
                              Ipv4Header header)
{
    NS_LOG_FUNCTION(this << dst);

    // Get my position
    Ptr<MobilityModel> mm = m_ipv4->GetObject<MobilityModel>();
    Vector myPos = mm->GetPosition();

    // Parse headers to get previous hop position
    Vector Position;
    Vector previousHop;
    uint32_t updated;
    Vector recPos;

    TypeHeader tHeader(GPSRTYPE_POS);
    p->RemoveHeader(tHeader);
    if (!tHeader.IsValid())
    {
        NS_LOG_DEBUG("GPSR message with unknown type in recovery. Drop");
        return;
    }

    if (tHeader.Get() == GPSRTYPE_POS)
    {
        PositionHeader hdr;
        p->RemoveHeader(hdr);
        Position.x = hdr.GetDstPosx();
        Position.y = hdr.GetDstPosy();
        updated = hdr.GetUpdated();
        recPos.x = hdr.GetRecPosx();
        recPos.y = hdr.GetRecPosy();
        previousHop.x = hdr.GetLastPosx();
        previousHop.y = hdr.GetLastPosy();

        // Update header with current position as last hop
        PositionHeader posHeader(Position.x,
                                 Position.y,
                                 updated,
                                 recPos.x,
                                 recPos.y,
                                 (uint8_t)1,
                                 myPos.x,
                                 myPos.y);
        p->AddHeader(posHeader);
        p->AddHeader(tHeader);
    }

    // Find best angle neighbor (right hand rule)
    Ipv4Address nextHop = m_neighbors.BestAngle(previousHop, myPos);

    if (nextHop == Ipv4Address::GetZero())
    {
        NS_LOG_DEBUG("Recovery mode failed for " << dst);
        return;
    }

    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst);
    route->SetGateway(nextHop);
    route->SetSource(header.GetSource());
    route->SetOutputDevice(m_ipv4->GetNetDevice(1));

    NS_LOG_LOGIC("Recovery forwarding to " << dst << " via " << nextHop);
    ucb(route, p, header);
}

void
RoutingProtocol::DeferredRouteOutput(Ptr<const Packet> p,
                                     const Ipv4Header& header,
                                     UnicastForwardCallback ucb,
                                     ErrorCallback ecb)
{
    NS_LOG_FUNCTION(this << p << header);
    NS_ASSERT(p && p != Ptr<Packet>());

    Ipv4Address dst = header.GetDestination();
    Ptr<Packet> packet = p->Copy();

    // Check if packet already has GPSR headers (e.g., came through AddHeaders path)
    // Only add headers if the packet doesn't already have them
    bool hasGpsrHeaders = false;
    if (packet->GetSize() >= 1)
    {
        uint8_t firstByte;
        packet->CopyData(&firstByte, 1);
        // GPSR TypeHeader has values 1 (HELLO) or 2 (POS)
        if (firstByte == 1 || firstByte == 2)
        {
            Ptr<Packet> testPacket = packet->Copy();
            TypeHeader tTest(GPSRTYPE_POS);
            testPacket->RemoveHeader(tTest);
            if (tTest.IsValid())
            {
                hasGpsrHeaders = true;
                NS_LOG_DEBUG("Packet already has GPSR headers, not adding again");
            }
        }
    }

    if (!hasGpsrHeaders)
    {
        // Get my position
        Ptr<MobilityModel> mm = m_ipv4->GetObject<MobilityModel>();
        Vector myPos = mm->GetPosition();

        // Get destination position
        Vector dstPos = m_locationService->GetPosition(dst);
        uint32_t updated = (uint32_t)m_locationService->GetEntryUpdateTime(dst).GetSeconds();

        // Add position header (not in recovery mode initially)
        PositionHeader posHeader((uint64_t)dstPos.x,
                                 (uint64_t)dstPos.y,
                                 updated,
                                 (uint64_t)0,
                                 (uint64_t)0,
                                 (uint8_t)0,
                                 (uint64_t)myPos.x,
                                 (uint64_t)myPos.y);
        packet->AddHeader(posHeader);
        TypeHeader tHeader(GPSRTYPE_POS);
        packet->AddHeader(tHeader);
    }

    if (m_queue.GetSize() == 0)
    {
        m_checkQueueTimer.Cancel();
        m_checkQueueTimer.Schedule(MilliSeconds(500));
    }

    QueueEntry newEntry(packet, header, ucb, ecb);
    bool result = m_queue.Enqueue(newEntry);

    m_queuedAddresses.push_front(header.GetDestination());
    m_queuedAddresses.unique();

    if (result)
    {
        NS_LOG_LOGIC("Add packet " << p->GetUid() << " to queue. Protocol "
                                   << (uint16_t)header.GetProtocol());
    }
}

void
RoutingProtocol::CheckQueue()
{
    NS_LOG_FUNCTION(this);

    m_checkQueueTimer.Cancel();

    std::list<Ipv4Address> toRemove;

    for (auto& addr : m_queuedAddresses)
    {
        if (SendPacketFromQueue(addr))
        {
            toRemove.push_back(addr);
        }
    }

    for (auto& addr : toRemove)
    {
        m_queuedAddresses.remove(addr);
    }

    if (!m_queuedAddresses.empty())
    {
        m_checkQueueTimer.Schedule(MilliSeconds(500));
    }
}

bool
RoutingProtocol::SendPacketFromQueue(Ipv4Address dst)
{
    NS_LOG_FUNCTION(this << dst);

    bool recovery = false;
    QueueEntry queueEntry;

    if (m_locationService->IsInSearch(dst))
    {
        return false;
    }

    if (!m_locationService->HasPosition(dst))
    {
        m_queue.DropPacketWithDst(dst);
        NS_LOG_LOGIC("Location Service did not find dst. Drop packet to " << dst);
        return true;
    }

    Ptr<MobilityModel> mm = m_ipv4->GetObject<MobilityModel>();
    Vector myPos = mm->GetPosition();
    Ipv4Address nextHop;

    if (m_neighbors.IsNeighbour(dst))
    {
        nextHop = dst;
    }
    else
    {
        Vector dstPos = m_locationService->GetPosition(dst);
        nextHop = m_neighbors.BestNeighbor(dstPos, myPos);

        if (nextHop == Ipv4Address::GetZero())
        {
            NS_LOG_LOGIC("Fallback to recovery-mode for packets to " << dst);
            recovery = true;
        }
    }

    if (recovery && m_perimeterMode)
    {
        // Send packets via recovery mode
        Vector Position;
        uint32_t updated;

        while (m_queue.Dequeue(dst, queueEntry))
        {
            Ptr<Packet> p = ConstCast<Packet>(queueEntry.GetPacket());
            UnicastForwardCallback ucb = queueEntry.GetUnicastForwardCallback();
            Ipv4Header header = queueEntry.GetIpv4Header();

            TypeHeader tHeader(GPSRTYPE_POS);
            p->RemoveHeader(tHeader);
            if (!tHeader.IsValid())
            {
                NS_LOG_DEBUG("GPSR message with unknown type. Drop");
                continue;
            }

            if (tHeader.Get() == GPSRTYPE_POS)
            {
                PositionHeader hdr;
                p->RemoveHeader(hdr);
                Position.x = hdr.GetDstPosx();
                Position.y = hdr.GetDstPosy();
                updated = hdr.GetUpdated();
            }

            PositionHeader posHeader(Position.x,
                                     Position.y,
                                     updated,
                                     myPos.x,
                                     myPos.y,
                                     (uint8_t)1,
                                     Position.x,
                                     Position.y);
            p->AddHeader(posHeader);
            p->AddHeader(tHeader);

            RecoveryMode(dst, p, ucb, header);
        }
        return true;
    }

    // Normal greedy forwarding from queue
    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst);
    route->SetGateway(nextHop);
    route->SetOutputDevice(m_ipv4->GetNetDevice(1));

    while (m_queue.Dequeue(dst, queueEntry))
    {
        Ptr<Packet> p = ConstCast<Packet>(queueEntry.GetPacket());
        Ipv4Header header = queueEntry.GetIpv4Header();

        // Check for uninitialized source address (0.0.0.0 or legacy 102.102.102.102)
        Ipv4Address srcAddr = header.GetSource();
        if (srcAddr == Ipv4Address::GetZero() || srcAddr == Ipv4Address("102.102.102.102"))
        {
            route->SetSource(m_ipv4->GetAddress(1, 0).GetLocal());
            header.SetSource(m_ipv4->GetAddress(1, 0).GetLocal());
        }
        else
        {
            route->SetSource(srcAddr);
        }

        // Packets in queue already have GPSR headers from DeferredRouteOutput
        // Use m_downTarget directly instead of ucb to avoid re-adding headers
        header.SetTtl(header.GetTtl() > 0 ? header.GetTtl() - 1 : 64);
        m_downTarget(p, header.GetSource(), header.GetDestination(), header.GetProtocol(), route);
    }

    return true;
}

void
RoutingProtocol::AddHeaders(Ptr<Packet> p,
                            Ipv4Address source,
                            Ipv4Address destination,
                            uint8_t protocol,
                            Ptr<Ipv4Route> route)
{
    NS_LOG_FUNCTION(this << " source " << source << " destination " << destination);

    Ptr<MobilityModel> mm = m_ipv4->GetObject<MobilityModel>();
    Vector myPos = mm->GetPosition();

    Ipv4Address nextHop;
    if (m_neighbors.IsNeighbour(destination))
    {
        nextHop = destination;
    }
    else
    {
        nextHop = m_neighbors.BestNeighbor(m_locationService->GetPosition(destination), myPos);
    }

    uint64_t positionX = 0;
    uint64_t positionY = 0;
    uint32_t hdrTime = 0;

    if (destination != m_ipv4->GetAddress(1, 0).GetBroadcast())
    {
        Vector dstPos = m_locationService->GetPosition(destination);
        positionX = (uint64_t)dstPos.x;
        positionY = (uint64_t)dstPos.y;
        hdrTime = (uint32_t)m_locationService->GetEntryUpdateTime(destination).GetSeconds();
    }

    PositionHeader posHeader(positionX,
                             positionY,
                             hdrTime,
                             (uint64_t)0,
                             (uint64_t)0,
                             (uint8_t)0,
                             (uint64_t)myPos.x,
                             (uint64_t)myPos.y);
    p->AddHeader(posHeader);
    TypeHeader tHeader(GPSRTYPE_POS);
    p->AddHeader(tHeader);
    NS_LOG_DEBUG("Added GPSR headers to packet " << p->GetUid());

    // Send the packet with headers via the IP layer's down target
    // m_downTarget must be configured by calling GpsrHelper::Install() after InternetStackHelper
    if (m_downTarget.IsNull())
    {
        NS_LOG_WARN("GPSR: m_downTarget is not configured! Call GpsrHelper::Install() after InternetStackHelper.Install()");
        NS_ASSERT_MSG(!m_downTarget.IsNull(), 
            "m_downTarget callback not set. You must call GpsrHelper::Install() after InternetStackHelper.Install()");
        return;
    }
    m_downTarget(p, source, destination, protocol, route);
}

void
RoutingProtocol::SetDownTarget(IpL4Protocol::DownTargetCallback callback)
{
    m_downTarget = callback;
}

IpL4Protocol::DownTargetCallback
RoutingProtocol::GetDownTarget() const
{
    return m_downTarget;
}

void
RoutingProtocol::PrintRoutingTable(Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{
    *stream->GetStream() << "GPSR Routing Protocol - Neighbor Table\n";
    *stream->GetStream() << "(Neighbor positions are dynamic)\n";
}

} // namespace gpsr
} // namespace ns3
