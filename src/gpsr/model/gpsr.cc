/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Routing Protocol - Complete Implementation
 * Based on original GPSR implementation
 */

#include "gpsr.h"
#include "gpsr-dcc.h"
#include "gpsr-metric-supervisor.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/inet-socket-address.h"
#include "ns3/log.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/udp-l4-protocol.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

// Define NS_LOG_APPEND_CONTEXT AFTER all includes to avoid conflicts
// with template functions in included headers (e.g., wifi-phy-state-helper.h)
#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT                                                                      \
    if (m_ipv4)                                                                                    \
    {                                                                                              \
        std::clog << "[node " << m_ipv4->GetObject<Node>()->GetId() << "] ";                       \
    }

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("GpsrRoutingProtocol");

namespace gpsr
{

#define GPSR_LS_GOD 0
#define GPSR_LS_RLS 1

/// Maximum allowed jitter
#define GPSR_MAXJITTER (m_helloInterval.GetSeconds() / 2)

// ============ Geometry Helpers for Perimeter Mode ============
namespace
{

// Calculate squared distance between two points
inline double
CalculateDistanceSq(double x1, double y1, double x2, double y2)
{
    double dx = x1 - x2;
    double dy = y1 - y2;
    return dx * dx + dy * dy;
}

// Check if two line segments (A-B) and (C-D) intersect
// If they intersect, store the intersection point in (outX, outY)
// Returns true if there is a valid intersection point
// Handles collinear case: finds overlap point closest to B (destination)
bool
SegmentIntersect2D(double ax, double ay, double bx, double by,
                   double cx, double cy, double dx, double dy,
                   double& outX, double& outY)
{
    double denom = (bx - ax) * (dy - cy) - (by - ay) * (dx - cx);
    
    // Non-collinear case: standard intersection
    if (std::abs(denom) >= 1e-9)
    {
        double t = ((cx - ax) * (dy - cy) - (cy - ay) * (dx - cx)) / denom;
        double u = ((cx - ax) * (by - ay) - (cy - ay) * (bx - ax)) / denom;

        if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0)
        {
            outX = ax + t * (bx - ax);
            outY = ay + t * (by - ay);
            return true;
        }
        return false;
    }
    
    // Collinear case: check if segments overlap
    // Use Euclidean length for proper normalization
    double abLenSq = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
    double cdLenSq = (dx - cx) * (dx - cx) + (dy - cy) * (dy - cy);
    
    if (abLenSq < 1e-18 || cdLenSq < 1e-18)
        return false;  // Degenerate segment
    
    // Parameterize C and D on line AB
    double tC, tD;
    if (std::abs(bx - ax) > std::abs(by - ay))
    {
        tC = (cx - ax) / (bx - ax);
        tD = (dx - ax) / (bx - ax);
    }
    else
    {
        tC = (cy - ay) / (by - ay);
        tD = (dy - ay) / (by - ay);
    }
    
    // Check collinearity using squared cross-product vs squared length
    // Cross product: (C-A) x (B-A) and (D-A) x (B-A)
    double crossC = (cx - ax) * (by - ay) - (cy - ay) * (bx - ax);
    double crossD = (dx - ax) * (by - ay) - (dy - ay) * (bx - ax);
    // Compare cross^2 < epsilon * length^2 (avoids sqrt)
    double collinearThresholdSq = 1e-12 * abLenSq;
    if (crossC * crossC > collinearThresholdSq || crossD * crossD > collinearThresholdSq)
        return false;  // Not actually collinear
    
    // Find overlap: intersection of [0,1] and [tC,tD]
    double tMin = std::max(0.0, std::min(tC, tD));
    double tMax = std::min(1.0, std::max(tC, tD));
    
    if (tMin > tMax + 1e-9)
        return false;  // No overlap
    
    // Return point closest to B (t=1)
    double tBest = std::min(tMax, 1.0);
    outX = ax + tBest * (bx - ax);
    outY = ay + tBest * (by - ay);
    return true;
}

} // anonymous namespace

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
                          MakeBooleanChecker())
            // ========== Adaptive HELLO attributes ==========
            .AddAttribute("AdaptiveHelloEnabled",
                          "Enable adaptive HELLO interval (ETSI CAM style)",
                          BooleanValue(true),
                          MakeBooleanAccessor(&RoutingProtocol::m_adaptiveHelloEnabled),
                          MakeBooleanChecker())
            .AddAttribute("HelloIntervalMin",
                          "Minimum HELLO interval (adaptive mode)",
                          TimeValue(MilliSeconds(100)),
                          MakeTimeAccessor(&RoutingProtocol::m_helloIntervalMin),
                          MakeTimeChecker())
            .AddAttribute("HelloIntervalMax",
                          "Maximum HELLO interval (adaptive mode)",
                          TimeValue(Seconds(1)),
                          MakeTimeAccessor(&RoutingProtocol::m_helloIntervalMax),
                          MakeTimeChecker())
            .AddAttribute("HeadingThreshold",
                          "Heading change threshold for HELLO trigger (degrees)",
                          DoubleValue(4.0),
                          MakeDoubleAccessor(&RoutingProtocol::m_headingThreshold),
                          MakeDoubleChecker<double>(0.0, 180.0))
            .AddAttribute("PositionThreshold",
                          "Position change threshold for HELLO trigger (meters)",
                          DoubleValue(4.0),
                          MakeDoubleAccessor(&RoutingProtocol::m_positionThreshold),
                          MakeDoubleChecker<double>(0.0, 1000.0))
            .AddAttribute("SpeedThreshold",
                          "Speed change threshold for HELLO trigger (m/s)",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&RoutingProtocol::m_speedThreshold),
                          MakeDoubleChecker<double>(0.0, 100.0))
            .AddAttribute("HelloCheckInterval",
                          "Condition check interval for adaptive HELLO",
                          TimeValue(MilliSeconds(50)),
                          MakeTimeAccessor(&RoutingProtocol::m_helloCheckInterval),
                          MakeTimeChecker())
            // ========== DCC attributes ==========
            .AddAttribute("DccEnabled",
                          "Enable DCC (Decentralized Congestion Control) for HELLO messages",
                          BooleanValue(true),
                          MakeBooleanAccessor(&RoutingProtocol::m_dccEnabled),
                          MakeBooleanChecker())
            .AddAttribute("DccMode",
                          "DCC mode: 'reactive' or 'adaptive'",
                          StringValue("reactive"),
                          MakeStringAccessor(&RoutingProtocol::m_dccMode),
                          MakeStringChecker());
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
    
    // ========== Adaptive HELLO Logic (ETSI EN 302 637-2 style) ==========
    if (!m_adaptiveHelloEnabled)
    {
        // Fixed interval mode (legacy behavior)
        SendHello();
        Ptr<UniformRandomVariable> jitter = CreateObject<UniformRandomVariable>();
        jitter->SetAttribute("Min", DoubleValue(-GPSR_MAXJITTER));
        jitter->SetAttribute("Max", DoubleValue(GPSR_MAXJITTER));
        m_helloIntervalTimer.Schedule(m_helloInterval + Seconds(jitter->GetValue()));
        return;
    }
    
    // ========== FIX 1: min/max consistency check ==========
    Time effectiveMin = std::min(m_helloIntervalMin, m_helloIntervalMax);
    Time effectiveMax = std::max(m_helloIntervalMin, m_helloIntervalMax);
    
    // Get current mobility state
    Ptr<MobilityModel> mm = m_ipv4->GetObject<MobilityModel>();
    if (!mm)
    {
        // ========== FIX 4: No MobilityModel - fallback to fixed interval ==========
        // Without mobility info, adaptive mode is meaningless; use legacy fixed interval
        NS_LOG_DEBUG("No MobilityModel, falling back to fixed interval HELLO");
        SendHello();
        Ptr<UniformRandomVariable> jitter = CreateObject<UniformRandomVariable>();
        jitter->SetAttribute("Min", DoubleValue(-GPSR_MAXJITTER));
        jitter->SetAttribute("Max", DoubleValue(GPSR_MAXJITTER));
        m_helloIntervalTimer.Schedule(m_helloInterval + Seconds(jitter->GetValue()));
        return;
    }
    
    Vector curPos = mm->GetPosition();
    Vector curVel = mm->GetVelocity();
    double curSpeed = std::sqrt(curVel.x * curVel.x + curVel.y * curVel.y);
    
    // Calculate heading from velocity (degrees, 0=East, CCW positive)
    double curHeading = 0.0;
    if (curSpeed > 0.1)  // Only compute heading if moving
    {
        curHeading = std::atan2(curVel.y, curVel.x) * 180.0 / M_PI;
    }
    else if (m_prevHeading > -500.0)  // Use previous heading if stationary
    {
        curHeading = m_prevHeading;
    }
    
    Time now = Simulator::Now();
    Time elapsed = now - m_lastHelloTime;
    bool shouldSend = false;
    std::string triggerReason = "";
    
    // Minimum interval protection (use effective min)
    if (elapsed < effectiveMin)
    {
        // Too soon, schedule next check with jitter
        Ptr<UniformRandomVariable> checkJitter = CreateObject<UniformRandomVariable>();
        checkJitter->SetAttribute("Min", DoubleValue(0.75));
        checkJitter->SetAttribute("Max", DoubleValue(1.25));
        m_helloIntervalTimer.Schedule(m_helloCheckInterval * checkJitter->GetValue());
        return;
    }
    
    // Check trigger conditions (only if previous state is valid)
    if (m_prevHeading > -500.0)  // Valid previous state exists
    {
        // Condition 1a: Heading change > threshold
        double headingDiff = std::abs(curHeading - m_prevHeading);
        if (headingDiff > 180.0)
        {
            headingDiff = 360.0 - headingDiff;
        }
        if (headingDiff > m_headingThreshold)
        {
            shouldSend = true;
            triggerReason = "heading";
        }
        
        // Condition 1b: Position change > threshold
        double posDiff = CalculateDistance(curPos, m_prevPosition);
        if (posDiff > m_positionThreshold)
        {
            shouldSend = true;
            triggerReason = (triggerReason.empty() ? "position" : triggerReason + "+position");
        }
        
        // Condition 1c: Speed change > threshold
        if (m_prevSpeed >= 0.0 && std::abs(curSpeed - m_prevSpeed) > m_speedThreshold)
        {
            shouldSend = true;
            triggerReason = (triggerReason.empty() ? "speed" : triggerReason + "+speed");
        }
    }
    else
    {
        // First HELLO after initialization
        shouldSend = true;
        triggerReason = "init";
    }
    
    // Condition 2: Maximum interval timeout (use effective max)
    if (elapsed >= effectiveMax)
    {
        shouldSend = true;
        if (triggerReason.empty())
        {
            triggerReason = "timeout";
        }
    }
    
    // Send HELLO if triggered
    if (shouldSend)
    {
        // ========== DCC Gate Check ==========
        if (m_dccEnabled && m_dcc)
        {
            int64_t nowMs = Simulator::Now().GetMilliSeconds();
            if (!m_dcc->CheckGateOpen(nowMs))
            {
                NS_LOG_DEBUG("HELLO suppressed by DCC (gate closed), Toff=" 
                             << m_dcc->GetToff() << "ms");
                // Skip sending, but still reschedule
                goto reschedule;
            }
        }
        
        NS_LOG_DEBUG("Adaptive HELLO triggered: " << triggerReason 
                     << " elapsed=" << elapsed.GetMilliSeconds() << "ms"
                     << " heading=" << curHeading << " (prev=" << m_prevHeading << ")"
                     << " speed=" << curSpeed << " (prev=" << m_prevSpeed << ")");
        
        // ========== FIX 3: State update moved to SendHello() ==========
        // SendHello() will update m_lastHelloTime/m_prev* at actual send time
        SendHello();
        
        // Notify DCC of transmission
        if (m_dccEnabled && m_dcc)
        {
            m_dcc->NotifyTx(Simulator::Now().GetMilliSeconds());
        }
    }
    
reschedule:
    
    // Schedule next check with jitter (±25%)
    Ptr<UniformRandomVariable> checkJitter = CreateObject<UniformRandomVariable>();
    checkJitter->SetAttribute("Min", DoubleValue(0.75));
    checkJitter->SetAttribute("Max", DoubleValue(1.25));
    m_helloIntervalTimer.Schedule(m_helloCheckInterval * checkJitter->GetValue());
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
    Vector vel = mm->GetVelocity();

    for (auto& s : m_socketAddresses)
    {
        Ptr<Socket> socket = s.first;
        Ipv4InterfaceAddress iface = s.second;

        HelloHeader helloHeader(pos.x, pos.y);
        
        // Set velocity
        helloHeader.SetVelocity(vel.x, vel.y);
        
        // Set timestamp (milliseconds since simulation start)
        helloHeader.SetTimestamp(static_cast<uint32_t>(Simulator::Now().GetMilliSeconds()));
        
        // Set Top-K neighbor summaries for two-hop routing
        std::vector<NeighborSummary> neighborList = m_neighbors.GetTopKNeighborSummaries(
            HelloHeader::MAX_NEIGHBORS,
            pos);  // Current node position for distance sorting
        helloHeader.SetNeighbors(neighborList);

        Ptr<Packet> packet = Create<Packet>();
        packet->AddHeader(helloHeader);
        TypeHeader tHeader(GPSRTYPE_HELLO);
        packet->AddHeader(tHeader);
        
        // Add tag to mark this packet as having GPSR headers
        GpsrHeaderTag tag(GPSRTYPE_HELLO);
        packet->AddPacketTag(tag);

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
        
        // Update control overhead statistics
        m_ctrlHelloTxPkts++;
        m_ctrlHelloTxBytes += packet->GetSize();  // GPSR+UDP payload (add +28 for IP/UDP headers if needed)
        
        NS_LOG_DEBUG("Sent HELLO from " << iface.GetLocal() << " to " << destination
                     << " with " << neighborList.size() << " neighbors");
    }
    
    // ========== FIX 3: Update state at actual send time ==========
    // This ensures m_prev* reflects the state when HELLO was actually sent
    double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
    double heading = 0.0;
    if (speed > 0.1)
    {
        heading = std::atan2(vel.y, vel.x) * 180.0 / M_PI;
    }
    else if (m_prevHeading > -500.0)
    {
        heading = m_prevHeading;  // Keep previous heading if stationary
    }
    
    m_lastHelloTime = Simulator::Now();
    m_prevPosition = pos;
    m_prevSpeed = speed;
    m_prevHeading = heading;
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
        // Extended HelloHeader: min 37 bytes (16+16+4+1 = position+velocity+timestamp+count)
        if (packet->GetSize() < 37)
        {
            NS_LOG_DEBUG("Packet too small for extended HelloHeader, size: " << packet->GetSize());
            return;
        }

        HelloHeader hdr;
        packet->RemoveHeader(hdr);

        Vector pos;
        pos.x = hdr.GetOriginPosx();
        pos.y = hdr.GetOriginPosy();
        
        Vector vel;
        vel.x = hdr.GetVelocityX();
        vel.y = hdr.GetVelocityY();
        
        uint32_t timestamp = hdr.GetTimestamp();
        const auto& twoHopNeighbors = hdr.GetNeighbors();

        InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom(sourceAddress);
        Ipv4Address sender = inetSourceAddr.GetIpv4();
        Ipv4Address receiver = m_socketAddresses[socket].GetLocal();

        NS_LOG_DEBUG("HELLO from " << sender << " pos(" << pos.x << "," << pos.y << ") "
                     << "vel(" << vel.x << "," << vel.y << ") "
                     << "ts:" << timestamp << " 2hop:" << twoHopNeighbors.size());
        
        // Update 1-hop neighbor with extended info (velocity, two-hop neighbors)
        m_neighbors.AddEntryExtended(sender, pos, vel, twoHopNeighbors);
        
        // Log the full current neighbor list
        NS_LOG_DEBUG("NEIGHBOR LIST: Node " << receiver << " neighbors: " << m_neighbors.GetNeighborList());
        
        // Debug: Log two-hop neighbors
        for (const auto& twoHop : twoHopNeighbors)
        {
            NS_LOG_DEBUG("  2-hop via " << sender << ": " << twoHop.ip 
                         << " pos(" << twoHop.x << "," << twoHop.y << ") lq:" << (int)twoHop.linkQuality);
        }
    }
}

void
RoutingProtocol::UpdateRouteToNeighbor(Ipv4Address sender, Ipv4Address receiver, Vector pos)
{
    NS_LOG_FUNCTION(this << sender << receiver << pos);
    
    // Log neighbor update with detailed information
    NS_LOG_INFO("NEIGHBOR UPDATE: Node " << receiver << " discovered neighbor " 
                << sender << " at position (" << pos.x << ", " << pos.y << ")");
    
    m_neighbors.AddEntry(sender, pos);
    
    // Log the full current neighbor list
    NS_LOG_INFO("NEIGHBOR LIST: Node " << receiver << " neighbors: " << m_neighbors.GetNeighborList());
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

    // FIX: GPSR only supports UDP traffic. Reject non-UDP early to prevent black holes.
    // Non-UDP packets would be dropped later by Forwarding/DeferredRouteOutput anyway.
    if (header.GetProtocol() != UdpL4Protocol::PROT_NUMBER) // Not UDP
    {
        sockerr = Socket::ERROR_NOROUTETOHOST;
        NS_LOG_LOGIC("GPSR only supports UDP. Protocol " << (int)header.GetProtocol() << " rejected.");
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
        Vector myVel = mm->GetVelocity();
        nextHop = m_neighbors.BestNeighborTwoHop(dstPos, myPos, myVel);
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
    NS_LOG_DEBUG("RouteInput-RX: UID=" << p->GetUid() << " rawSize=" << p->GetSize() << " dst=" << header.GetDestination());

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
        // UID-based dedup: prevent duplicate local delivery (same UID within time window)
        const uint64_t uid = p->GetUid();
        const Time now = Simulator::Now();
        auto it = m_localDeliverCache.find(uid);
        if (it != m_localDeliverCache.end() && (now - it->second) <= m_localDeliverWindow)
        {
            NS_LOG_DEBUG("LocalDelivery: duplicate UID " << uid << ", skip");
            return true;  // Already processed, swallow duplicate
        }
        m_localDeliverCache[uid] = now;

        // Optional: clean expired entries
        for (auto iter = m_localDeliverCache.begin(); iter != m_localDeliverCache.end(); )
        {
            if (now - iter->second > m_localDeliverWindow)
                iter = m_localDeliverCache.erase(iter);
            else
                ++iter;
        }

        Ptr<Packet> packet = p->Copy();

        // Remove GPSR headers if this is a UDP data packet (not HELLO, not ICMP)
        // ICMP packets may inherit stale GpsrHeaderTag from original packets due to NS-3 Packet reuse
        // Only UDP packets (protocol 17) should have GPSR headers
        // FIX: Also check for fragmentation. GPSR does not support fragmentation.
        // We must drop ANY fragmented packet (offset != 0 OR not last fragment).
        GpsrHeaderTag gpsrTag;
        GpsrLocalDeliveredTag localDeliveredTag;
        uint32_t minGpsrSize = TypeHeader().GetSerializedSize() + PositionHeader().GetSerializedSize();
        
        // Check if already processed (prevent duplicate local delivery)
        if (packet->PeekPacketTag(localDeliveredTag))
        {
            NS_LOG_DEBUG("LocalDelivery: Already processed (GpsrLocalDeliveredTag present), skipping");
        }
        else if (header.GetProtocol() == UdpL4Protocol::PROT_NUMBER && // UDP only
            header.GetFragmentOffset() == 0 && header.IsLastFragment() && // No fragmentation allowed
            packet->GetSize() >= minGpsrSize && // Sufficient size for GPSR headers
            packet->PeekPacketTag(gpsrTag) && gpsrTag.GetType() == GPSRTYPE_POS)
        {
            // Validate TypeHeader before removing (防止 stale tag 导致错误剥离)
            TypeHeader tHeader(GPSRTYPE_POS);
            uint32_t preSize = packet->GetSize();
            uint32_t peekedBytes = packet->PeekHeader(tHeader);
            
            NS_LOG_DEBUG("LocalDelivery Check: Size=" << preSize 
                         << " UID=" << packet->GetUid()
                         << " PeekTypeHeader=" << peekedBytes
                         << " Type=" << (int)tHeader.Get()
                         << " TypeValid=" << tHeader.IsValid()
                         << " Tag=" << (int)gpsrTag.GetType());
            
            if (peekedBytes > 0 && tHeader.IsValid() && tHeader.Get() == GPSRTYPE_POS)
            {
                packet->RemoveHeader(tHeader);
                PositionHeader phdr;
                packet->RemoveHeader(phdr);
                packet->RemovePacketTag(gpsrTag);
                packet->AddPacketTag(localDeliveredTag);  // Mark as processed
                
                // Read hop count for statistics
                GpsrHopCountTag hopTag;
                uint8_t hopCount = 0;
                if (packet->PeekPacketTag(hopTag))
                {
                    hopCount = hopTag.GetHopCount();
                    packet->RemovePacketTag(hopTag);  // Clean up tag
                }
                NS_LOG_INFO("LocalDelivery: UID=" << packet->GetUid() 
                            << " hops=" << (int)hopCount 
                            << " size=" << packet->GetSize());
                NS_LOG_DEBUG("Removed GPSR headers: pre=" << preSize << " post=" << packet->GetSize());
            }
            else
            {
                // Tag/Type mismatch or corrupted header - just clear tag, don't strip
                NS_LOG_DEBUG("LocalDelivery: Tag/Type mismatch, clearing tag only");
                packet->RemovePacketTag(gpsrTag);
                packet->AddPacketTag(localDeliveredTag);  // Mark as processed
            }
        }
        else if (packet->PeekPacketTag(gpsrTag) && gpsrTag.GetType() == GPSRTYPE_POS)
        {
            // This is a GPSR-routed UDP packet that doesn't meet delivery criteria.
            // Most likely it's a fragment. If we deliver it, the GPSR headers will
            // remain in the payload and corrupt the reassembled data.
            // FIX: DROP any fragmented GPSR packet instead of delivering with headers.
            if (header.GetFragmentOffset() != 0 || !header.IsLastFragment())
            {
                NS_LOG_DEBUG("LocalDelivery: Dropping fragmented GPSR packet (fragOff=" 
                             << header.GetFragmentOffset() << " moreFrags=" << !header.IsLastFragment() << ")");
                return false; // Drop the packet entirely
            }
            // Size too small or other issue - clear tag and continue
            NS_LOG_DEBUG("LocalDelivery: Clearing stale GPSR tag (protocol=" 
                         << (int)header.GetProtocol() << " size=" << packet->GetSize() << ")");
            packet->RemovePacketTag(gpsrTag);
        }
        else if (packet->PeekPacketTag(gpsrTag))
        {
            // Non-POS tag (e.g., HELLO) - just clear it
            packet->RemovePacketTag(gpsrTag);
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

    NS_LOG_DEBUG("Forwarding packet size: " << p->GetSize());

    // FIX: Four-layer protection before RemoveHeader to avoid crash on non-GPSR packets
    // Layer 0: Check protocol - only UDP packets have GPSR headers (ICMP inherits stale tags)
    if (header.GetProtocol() != UdpL4Protocol::PROT_NUMBER) // Not UDP
    {
        NS_LOG_DEBUG("Non-UDP packet (protocol " << (int)header.GetProtocol() << "). Not a GPSR data packet. Drop.");
        // Clear any stale GPSR tag that might have been inherited
        GpsrHeaderTag staleTag;
        if (p->PeekPacketTag(staleTag))
        {
            p->RemovePacketTag(staleTag);
        }
        return false;
    }
    
    // Layer 1: Check fragmentation - GPSR does not support fragmented packets
    // Drop ANY fragmented packet: offset != 0 (not first) OR !IsLastFragment (more fragments follow)
    if (header.GetFragmentOffset() != 0 || !header.IsLastFragment())
    {
        NS_LOG_DEBUG("Fragmented packet (offset=" << header.GetFragmentOffset() 
                     << " moreFrags=" << !header.IsLastFragment() << "). GPSR cannot route fragments. Drop.");
        return false;
    }
    
    // Layer 2: Check Tag - only packets with GPSRTYPE_POS tag have GPSR headers
    GpsrHeaderTag tag;
    if (!p->PeekPacketTag(tag) || tag.GetType() != GPSRTYPE_POS)
    {
        NS_LOG_DEBUG("No GPSR POS tag found. Not a GPSR data packet. Drop.");
        return false;
    }
    
    // Layer 3: Check size - must have at least TypeHeader + PositionHeader
    uint32_t minGpsrSize = TypeHeader().GetSerializedSize() + PositionHeader().GetSerializedSize();
    if (p->GetSize() < minGpsrSize)
    {
        NS_LOG_DEBUG("Packet too small for GPSR headers (size " << p->GetSize() << " < " << minGpsrSize << "). Drop.");
        return false;
    }
    
    // Safe to remove headers now
    p->RemoveHeader(tHeader);

    if (!tHeader.IsValid())
    {
        NS_LOG_DEBUG("GPSR TypeHeader invalid after RemoveHeader. Drop.");
        return false;
    }

    if (tHeader.Get() == GPSRTYPE_POS)
    {
        p->RemoveHeader(hdr);
        NS_LOG_DEBUG("Forwarding-HDR: UID=" << p->GetUid() 
                     << " Size=" << p->GetSize()
                     << " InRec=" << (int)hdr.GetInRec()
                     << " Nhops=" << (int)hdr.GetNhops()
                     << " E0From=" << hdr.GetE0From()
                     << " E0To=" << hdr.GetE0To());
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
        // Sync tag with header
        GpsrHeaderTag tag(GPSRTYPE_POS);
        if (!p->PeekPacketTag(tag)) { p->AddPacketTag(tag); }
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

    // Find best neighbor using two-hop aware scoring
    Vector myVel = mm->GetVelocity();
    Ipv4Address nextHop = m_neighbors.BestNeighborTwoHop(Position, myPos, myVel);

    if (nextHop != Ipv4Address::GetZero())
    {
        // Greedy forwarding successful
        PositionHeader posHeader(Position.x,
                                 Position.y,
                                 updated,
                                 0.0,
                                 0.0,
                                 (uint8_t)0,
                                 myPos.x,
                                 myPos.y);
        p->AddHeader(posHeader);
        p->AddHeader(tHeader);
        // Sync tag with header
        GpsrHeaderTag tag(GPSRTYPE_POS);
        if (!p->PeekPacketTag(tag)) { p->AddPacketTag(tag); }
        
        // Increment hop count for statistics
        GpsrHopCountTag hopTag;
        if (p->PeekPacketTag(hopTag))
        {
            p->RemovePacketTag(hopTag);
            hopTag.Increment();
            p->AddPacketTag(hopTag);
        }
        
        // DEBUG: Verify header was written correctly
        {
            TypeHeader tmpT;
            p->PeekHeader(tmpT);
            PositionHeader tmpP;
            Ptr<Packet> copy = p->Copy();
            copy->RemoveHeader(tmpT);
            copy->PeekHeader(tmpP);
            NS_LOG_DEBUG("Forwarding TX-CHECK: pktSize=" << p->GetSize() 
                         << " nhops=" << (int)tmpP.GetNhops());
        }

        Ptr<Ipv4Route> route = Create<Ipv4Route>();
        route->SetDestination(dst);
        route->SetSource(header.GetSource());
        route->SetGateway(nextHop);
        route->SetOutputDevice(m_ipv4->GetNetDevice(1));

        NS_LOG_LOGIC("Forwarding to " << dst << " from " << origin << " via " << nextHop);
        
        // Add GpsrNextHopTag to pass next-hop info to EpcUeNas for TFT matching
        GpsrNextHopTag existingNhTag;
        if (p->PeekPacketTag(existingNhTag))
        {
            // FIX: Drop packet if TTL has reached 0 (loop prevention)
            uint8_t ttl = existingNhTag.GetTtl();
            if (ttl == 0)
            {
                NS_LOG_DEBUG("GpsrNextHopTag TTL=0, dropping packet to prevent loop");
                return true;  // Silent drop - packet handled (consumed), no error callback
            }
            // Decrement TTL and update next-hop
            p->RemovePacketTag(existingNhTag);
            GpsrNextHopTag nhTag(nextHop, ttl - 1);
            p->AddPacketTag(nhTag);
            NS_LOG_DEBUG("Updated GpsrNextHopTag: nextHop=" << nextHop << " ttl=" << (int)(ttl - 1));
        }
        else
        {
            GpsrNextHopTag nhTag(nextHop, 63);  // First forward, TTL=63
            p->AddPacketTag(nhTag);
            NS_LOG_DEBUG("Added GpsrNextHopTag: nextHop=" << nextHop << " ttl=63");
        }
        
        Ipv4Header newHeader = header;
        newHeader.SetPayloadSize(p->GetSize());
        ucb(route, p, newHeader);
        return true;
    }

    // No greedy next hop - enter recovery mode
    if (m_perimeterMode)
    {
        hdr.SetInRec(1);
        hdr.SetRecPosx(myPos.x);
        hdr.SetRecPosy(myPos.y);
        // Keep original lastPos from packet (incoming edge) - do NOT overwrite

        p->AddHeader(hdr);
        p->AddHeader(tHeader);
        // Sync tag with header
        GpsrHeaderTag tag(GPSRTYPE_POS);
        if (!p->PeekPacketTag(tag)) { p->AddPacketTag(tag); }

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

    // Get my position and IP
    Ptr<MobilityModel> mm = m_ipv4->GetObject<MobilityModel>();
    Vector myPos = mm->GetPosition();
    Ipv4Address myIp = m_ipv4->GetAddress(1, 0).GetLocal();

    // Parse GPSR headers
    TypeHeader tHeader(GPSRTYPE_POS);
    
    GpsrHeaderTag tag;
    if (!p->PeekPacketTag(tag) || tag.GetType() != GPSRTYPE_POS)
    {
        NS_LOG_DEBUG("RecoveryMode: No GPSR POS tag found. Drop.");
        return;
    }
    
    uint32_t minGpsrSize = TypeHeader().GetSerializedSize() + PositionHeader().GetSerializedSize();
    if (p->GetSize() < minGpsrSize)
    {
        NS_LOG_DEBUG("RecoveryMode: Packet too small for GPSR headers. Drop.");
        return;
    }
    
    p->RemoveHeader(tHeader);
    if (!tHeader.IsValid())
    {
        NS_LOG_DEBUG("RecoveryMode: GPSR TypeHeader invalid. Drop");
        return;
    }

    PositionHeader hdr;
    p->RemoveHeader(hdr);
    NS_LOG_DEBUG("RecoveryMode-HDR: UID=" << p->GetUid() 
                 << " Size=" << p->GetSize()
                 << " InRec=" << (int)hdr.GetInRec()
                 << " Nhops=" << (int)hdr.GetNhops()
                 << " E0From=" << hdr.GetE0From()
                 << " E0To=" << hdr.GetE0To());
    
    // Extract header fields
    Vector dstPos(hdr.GetDstPosx(), hdr.GetDstPosy(), 0);
    Vector recPos = Vector(hdr.GetRecPosx(), hdr.GetRecPosy(), 0);   // Lp (Perimeter entry point)
    Vector previousHop(hdr.GetLastPosx(), hdr.GetLastPosy(), 0);
    Vector lfPos = Vector(hdr.GetLfPosx(), hdr.GetLfPosy(), 0);      // Lf (Face crossing point)
    uint32_t e0From = hdr.GetE0From();
    uint32_t e0To = hdr.GetE0To();
    uint32_t lfEdgeFrom = hdr.GetLfEdgeFrom();
    uint32_t lfEdgeTo = hdr.GetLfEdgeTo();
    uint8_t inRec = hdr.GetInRec();
    uint32_t updated = hdr.GetUpdated();

    // === Greedy Return Check ===
    // If current node is closer to D than Lp, return to greedy mode
    double distMyToD = CalculateDistanceSq(myPos.x, myPos.y, dstPos.x, dstPos.y);
    double distLpToD = CalculateDistanceSq(recPos.x, recPos.y, dstPos.x, dstPos.y);
    
    if (inRec == 1 && distMyToD < distLpToD - 1e-9)
    {
        NS_LOG_DEBUG("RecoveryMode: Greedy return! distSq to D: " 
                     << distMyToD << " < " << distLpToD << ". Switching to Greedy.");
        
        // Rebuild header with cleared perimeter state
        PositionHeader greedyHeader(dstPos.x, dstPos.y, updated,
                                    0.0, 0.0, 0,  // Clear Lp and inRec
                                    myPos.x, myPos.y,
                                    0.0, 0.0, 0, 0);  // Clear Lf and e0
        p->AddHeader(greedyHeader);
        p->AddHeader(tHeader);
        GpsrHeaderTag newTag(GPSRTYPE_POS);
        if (!p->PeekPacketTag(newTag)) { p->AddPacketTag(newTag); }
        
        // Call Forwarding (greedy mode) instead of continuing perimeter
        // Note: Forwarding expects (packet, header, ucb, ecb)
        bool greedySuccess = Forwarding(p, header, ucb, 
            [](Ptr<const Packet>, const Ipv4Header&, Socket::SocketErrno){
                // Empty callback - logging not possible in lambda context due to NS_LOG_APPEND_CONTEXT
            });
        if (!greedySuccess)
        {
            NS_LOG_DEBUG("RecoveryMode: Greedy return failed, dropping packet");
        }
        return;  // EXIT RecoveryMode
    }

    // === Right-Hand Rule: Find Next Hop ===
    // NS-2 uses ent_findface ONLY at perimeter ENTRY or loop recovery
    // Regular perimeter hops use ent_next_ccw from ingress neighbor
    Ipv4Address nextHop;
    
    if (e0From == 0 && e0To == 0)
    {
        // First entry into perimeter mode: use FindFace (ent_findface)
        nextHop = m_neighbors.FindFace(dstPos, myPos);
        NS_LOG_DEBUG("RecoveryMode: Entry - using FindFace");
    }
    else
    {
        // Already in perimeter: use NextCCW from ingress neighbor (ent_next_ccw)
        // Ingress neighbor is the last hop in history (where packet came from)
        Ipv4Address ingressNeighbor = Ipv4Address::GetZero();
        PeriHop lastHop;
        if (hdr.GetNhops() > 0)
        {
            lastHop = hdr.GetHop(hdr.GetNhops() - 1);
            ingressNeighbor = Ipv4Address(lastHop.ip);
        }
        
        if (ingressNeighbor != Ipv4Address::GetZero())
        {
            // peri-as-beacon: if ingress not in neighbor table, add it (NS-2 style)
            if (!m_neighbors.IsNeighbour(ingressNeighbor))
            {
                Vector ingressPos(lastHop.x, lastHop.y, lastHop.z);
                m_neighbors.AddEntry(ingressNeighbor, ingressPos);
                NS_LOG_DEBUG("RecoveryMode: peri-as-beacon - added ingress " << ingressNeighbor 
                             << " at (" << lastHop.x << "," << lastHop.y << ")");
            }
            
            nextHop = m_neighbors.NextCCW(ingressNeighbor, myPos);
            NS_LOG_DEBUG("RecoveryMode: Regular - using NextCCW from ingress=" << ingressNeighbor);
            
            // If NextCCW still fails, drop (NS-2 doesn't fallback to FindFace here)
            if (nextHop == Ipv4Address::GetZero())
            {
                NS_LOG_DEBUG("RecoveryMode: NextCCW returned Zero even after peri-as-beacon. Drop.");
                return;
            }
        }
        else
        {
            // No ingress in history - should not happen, drop
            NS_LOG_DEBUG("RecoveryMode: No ingress in hop history. Drop.");
            return;
        }
    }

    if (nextHop == Ipv4Address::GetZero())
    {
        NS_LOG_DEBUG("RecoveryMode: No valid neighbor. Drop.");
        return;
    }

    // Get next hop position for geometry calculations
    Vector nextHopPos = m_neighbors.GetPosition(nextHop);

    // === Add Current Hop to History (NS-2 style) ===
    if (!hdr.AddHop(myIp.Get(), myPos.x, myPos.y))
    {
        NS_LOG_DEBUG("RecoveryMode: Hop history full (MAX_PERI_HOPS reached). Loop detection may be weakened.");
    }
    hdr.SetHasHopList(1);  // 方案A: 标记真正添加了 hop

    // === Perimeter Entry Initialization ===
    // Only enter if truly starting perimeter (not from greedy return)
    if (e0From == 0 && e0To == 0)
    {
        // Entering perimeter mode: initialize Lp, Lf, e0
        inRec = 1;
        recPos = myPos;  // Lp = current position
        e0From = myIp.Get();
        e0To = nextHop.Get();
        
        // Calculate Lf = intersection of (Lp -> D) and (myPos -> nextHop)
        double lfX = recPos.x, lfY = recPos.y;  // Default: Lf = Lp
        SegmentIntersect2D(recPos.x, recPos.y, dstPos.x, dstPos.y,
                           myPos.x, myPos.y, nextHopPos.x, nextHopPos.y,
                           lfX, lfY);
        lfPos = Vector(lfX, lfY, 0);
        // Initialize lfEdge (NS-2 periptip)
        lfEdgeFrom = myIp.Get();
        lfEdgeTo = nextHop.Get();
        
        NS_LOG_DEBUG("RecoveryMode: Entering perimeter. Lp=(" << recPos.x << "," << recPos.y 
                     << ") e0=(" << Ipv4Address(e0From) << "->" << Ipv4Address(e0To) 
                     << ") Lf=(" << lfPos.x << "," << lfPos.y << ") lfEdge=(" << Ipv4Address(lfEdgeFrom) << "->" << Ipv4Address(lfEdgeTo) << ")");
    }
    else
    {
        // === Enhanced Loop Detection (NS-2 style) ===
        uint32_t currentEdgeFrom = myIp.Get();
        uint32_t currentEdgeTo = nextHop.Get();
        
        // Check 1: First-edge loop (return to e0) - DIRECTED comparison (NS-2 style)
        // NS-2 checks: current edge == periptip[1]->periptip[2]
        if (currentEdgeFrom == e0From && currentEdgeTo == e0To)
        {
            double lfDistSq = CalculateDistanceSq(lfPos.x, lfPos.y, hdr.GetLfPosx(), hdr.GetLfPosy());
            if (lfDistSq < 1e-6)  // Lf unchanged
            {
                NS_LOG_DEBUG("RecoveryMode: LOOP DETECTED (first edge, directed). Dropping packet.");
                return;  // Drop packet
            }
        }
        
        // Check 2: Mid-path loop (NS-2 style: revisit any edge in history)
        int loopIdx = hdr.FindEdge(currentEdgeFrom, currentEdgeTo);
        if (loopIdx >= 0 && loopIdx < hdr.GetNhops() - 2)
        {
            NS_LOG_DEBUG("RecoveryMode: Mid-path loop detected at hop " << loopIdx << ". Re-selecting face via FindFace.");
            // Clear hop history
            hdr.ClearHops();
            hdr.AddHop(myIp.Get(), myPos.x, myPos.y);
            hdr.SetHasHopList(1);  // 方案A: 标记真正添加了 hop
            
            // Re-select face via FindFace (NS-2 ent_findface)
            nextHop = m_neighbors.FindFace(dstPos, myPos);
            
            if (nextHop == Ipv4Address::GetZero())
            {
                NS_LOG_DEBUG("RecoveryMode: No valid face after loop recovery. Drop.");
                return;
            }
            
            nextHopPos = m_neighbors.GetPosition(nextHop);
            
            // Re-initialize e0 and lfEdge with new edge (NS-2 periptip update)
            e0From = myIp.Get();
            e0To = nextHop.Get();
            lfEdgeFrom = myIp.Get();
            lfEdgeTo = nextHop.Get();
            
            NS_LOG_DEBUG("RecoveryMode: Loop recovery - face re-selected. nextHop=" << nextHop 
                         << " e0/lfEdge updated");
        }

        // === Face Switching via CCW Chain (NS-2 style) ===
        // Only traverse edges on the current face using NextCCW, not all neighbors
        // Find if current edge or CCW neighbors have closer intersection with Lp->D
        double bestDistSq = CalculateDistanceSq(lfPos.x, lfPos.y, dstPos.x, dstPos.y);
        Ipv4Address bestNextHop = nextHop;
        Vector bestLf = lfPos;
        bool faceChanged = false;

        // Start from current nextHop and traverse CCW
        // Get the previous hop (edge we came from) to start CCW traversal
        Ipv4Address prevEdge = Ipv4Address(hdr.GetE0From());
        if (prevEdge == Ipv4Address::GetZero() || prevEdge == myIp)
        {
            // Use current nextHop as starting point if no valid prev edge
            prevEdge = nextHop;
        }

        // Helper: check if edge is Lf edge (undirected, NS-2 closer_pt skips periptip)
        auto isLfEdge = [&](uint32_t from, uint32_t to) {
            return (from == lfEdgeFrom && to == lfEdgeTo) ||
                   (from == lfEdgeTo && to == lfEdgeFrom);
        };

        // Check current edge first (but skip if it's the Lf edge)
        if (!isLfEdge(myIp.Get(), nextHop.Get()))
        {
            double ix, iy;
            if (SegmentIntersect2D(recPos.x, recPos.y, dstPos.x, dstPos.y,
                                   myPos.x, myPos.y, nextHopPos.x, nextHopPos.y,
                                   ix, iy))
            {
                double distSq = CalculateDistanceSq(ix, iy, dstPos.x, dstPos.y);
                if (distSq < bestDistSq - 1e-9)
                {
                    bestDistSq = distSq;
                    bestNextHop = nextHop;
                    bestLf = Vector(ix, iy, 0);
                    faceChanged = true;
                }
            }
        }

        // Traverse CCW chain from nextHop (NS-2 closer_pt style)
        // NS-2 traverses until back to starting edge or Zero (no iteration limit)
        Ipv4Address ccwNeighbor = m_neighbors.NextCCW(nextHop, myPos);
        while (ccwNeighbor != Ipv4Address::GetZero() && 
               ccwNeighbor != nextHop)
        {
            // Skip lfEdge (undirected, NS-2 closer_pt skips periptip edge)
            if (isLfEdge(myIp.Get(), ccwNeighbor.Get()))
            {
                ccwNeighbor = m_neighbors.NextCCW(ccwNeighbor, myPos);
                continue;
            }
            Vector ccwPos = m_neighbors.GetPosition(ccwNeighbor);
            
            double ix, iy;
            if (SegmentIntersect2D(recPos.x, recPos.y, dstPos.x, dstPos.y,
                                   myPos.x, myPos.y, ccwPos.x, ccwPos.y,
                                   ix, iy))
            {
                double distSq = CalculateDistanceSq(ix, iy, dstPos.x, dstPos.y);
                if (distSq < bestDistSq - 1e-9)
                {
                    bestDistSq = distSq;
                    bestNextHop = ccwNeighbor;
                    bestLf = Vector(ix, iy, 0);
                    faceChanged = true;
                }
            }
            
            ccwNeighbor = m_neighbors.NextCCW(ccwNeighbor, myPos);
        }

        if (faceChanged)
        {
            nextHop = bestNextHop;
            nextHopPos = m_neighbors.GetPosition(nextHop);
            lfPos = bestLf;
            // Update lfEdge to new Lf edge (NS-2 periptip)
            lfEdgeFrom = myIp.Get();
            lfEdgeTo = nextHop.Get();
            e0From = myIp.Get();
            e0To = nextHop.Get();
            NS_LOG_DEBUG("RecoveryMode: Face switch via CCW! Best Lf=(" << lfPos.x << "," << lfPos.y 
                         << ") e0=(" << Ipv4Address(e0From) << "->" << Ipv4Address(e0To) << ") lfEdge updated");
        }
    }

    // === Build Updated Header ===
    PositionHeader posHeader(dstPos.x, dstPos.y, updated,
                             recPos.x, recPos.y, inRec,
                             myPos.x, myPos.y,
                             lfPos.x, lfPos.y,
                             e0From, e0To);
    posHeader.SetLfEdge(lfEdgeFrom, lfEdgeTo);
    // 方案A: 继承 hasHopList 标志
    posHeader.SetHasHopList(hdr.GetHasHopList());
    // Copy hop history from old header (with z coordinate)
    for (uint8_t h = 0; h < hdr.GetNhops(); h++)
    {
        PeriHop hop = hdr.GetHop(h);
        posHeader.AddHop(hop.ip, hop.x, hop.y, hop.z);
    }
    p->AddHeader(posHeader);
    p->AddHeader(tHeader);
    GpsrHeaderTag newTag(GPSRTYPE_POS);
    if (!p->PeekPacketTag(newTag)) { p->AddPacketTag(newTag); }
    
    // Increment hop count for statistics
    GpsrHopCountTag hopTag;
    if (p->PeekPacketTag(hopTag))
    {
        p->RemovePacketTag(hopTag);
        hopTag.Increment();
        p->AddPacketTag(hopTag);
    }
    
    // 诊断日志：发送时的包大小和 header 详情
    NS_LOG_DEBUG("RecoveryMode-TX: UID=" << p->GetUid()
                 << " pktSize=" << p->GetSize()
                 << " hasHopList=" << (int)posHeader.GetHasHopList()
                 << " nhops=" << (int)posHeader.GetNhops()
                 << " inRec=" << (int)inRec
                 << " hdrSize=" << posHeader.GetSerializedSize());

    // === Forward Packet ===
    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst);
    route->SetGateway(nextHop);
    route->SetSource(header.GetSource());
    route->SetOutputDevice(m_ipv4->GetNetDevice(1));

    NS_LOG_LOGIC("Recovery forwarding to " << dst << " via " << nextHop);
    
    // TTL Management
    GpsrNextHopTag existingNhTag;
    if (p->PeekPacketTag(existingNhTag))
    {
        uint8_t ttl = existingNhTag.GetTtl();
        if (ttl == 0)
        {
            NS_LOG_DEBUG("RecoveryMode: GpsrNextHopTag TTL=0, dropping packet to prevent loop");
            return;
        }
        p->RemovePacketTag(existingNhTag);
        GpsrNextHopTag nhTag(nextHop, ttl - 1);
        p->AddPacketTag(nhTag);
    }
    else
    {
        GpsrNextHopTag nhTag(nextHop, 63);
        p->AddPacketTag(nhTag);
    }
    
    Ipv4Header newHeader = header;
    newHeader.SetPayloadSize(p->GetSize());
    ucb(route, p, newHeader);
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

    // FIX: GPSR only processes UDP packets.
    // Non-UDP packets (e.g., ICMP) should not have GPSR headers added.
    if (header.GetProtocol() != UdpL4Protocol::PROT_NUMBER) // Not UDP
    {
        NS_LOG_DEBUG("DeferredRouteOutput: Non-UDP packet (protocol " << (int)header.GetProtocol() << "). GPSR only handles UDP. Dropping.");
        // Clear any stale tag that might have been inherited
        GpsrHeaderTag staleTag;
        if (packet->PeekPacketTag(staleTag))
        {
            packet->RemovePacketTag(staleTag);
        }
        return; // Don't add headers, don't queue
    }

    // For UDP packets: Check if GPSR headers are already present
    GpsrHeaderTag existingTag;
    bool hasGpsrHeaders = packet->PeekPacketTag(existingTag) && existingTag.GetType() == GPSRTYPE_POS;
    
    if (hasGpsrHeaders)
    {
        NS_LOG_DEBUG("UDP packet already has GPSR POS tag, not adding headers again");
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
        PositionHeader posHeader(dstPos.x,
                                 dstPos.y,
                                 updated,
                                 0.0,
                                 0.0,
                                 (uint8_t)0,
                                 myPos.x,
                                 myPos.y);
        packet->AddHeader(posHeader);
        TypeHeader tHeader(GPSRTYPE_POS);
        packet->AddHeader(tHeader);
        // Sync tag with header
        GpsrHeaderTag tag(GPSRTYPE_POS);
        if (!packet->PeekPacketTag(tag)) { packet->AddPacketTag(tag); }
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
        Vector myVel = mm->GetVelocity();
        nextHop = m_neighbors.BestNeighborTwoHop(dstPos, myPos, myVel);

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
            
            // FIX: Check Tag and Size before RemoveHeader
            GpsrHeaderTag tag;
            if (!p->PeekPacketTag(tag) || tag.GetType() != GPSRTYPE_POS)
            {
                NS_LOG_DEBUG("SendPacketFromQueue: No GPSR POS tag. Drop.");
                continue;
            }
            
            uint32_t minGpsrSize = TypeHeader().GetSerializedSize() + PositionHeader().GetSerializedSize();
            if (p->GetSize() < minGpsrSize)
            {
                NS_LOG_DEBUG("SendPacketFromQueue: Packet too small. Drop.");
                continue;
            }
            
            p->RemoveHeader(tHeader);
            if (!tHeader.IsValid())
            {
                NS_LOG_DEBUG("SendPacketFromQueue: Invalid TypeHeader. Drop.");
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
            // Sync tag with header
            GpsrHeaderTag newTag(GPSRTYPE_POS);
            if (!p->PeekPacketTag(newTag)) { p->AddPacketTag(newTag); }

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
        
        // Add/Update GpsrNextHopTag with the calculated next-hop
        GpsrNextHopTag existingNhTag;
        if (p->PeekPacketTag(existingNhTag))
        {
            // FIX: Drop packet if TTL has reached 0 (loop prevention)
            uint8_t ttl = existingNhTag.GetTtl();
            if (ttl == 0)
            {
                NS_LOG_DEBUG("SendPacketFromQueue: GpsrNextHopTag TTL=0, dropping packet");
                continue;  // Skip this packet, process next in queue
            }
            p->RemovePacketTag(existingNhTag);
            GpsrNextHopTag nhTag(nextHop, ttl - 1);
            p->AddPacketTag(nhTag);
        }
        else
        {
            GpsrNextHopTag nhTag(nextHop, 64);
            p->AddPacketTag(nhTag);
        }
        
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

    // Check if packet already has a GpsrHeaderTag (e.g., HELLO packets)
    // If so, skip adding POS headers - this packet is already a GPSR control packet
    GpsrHeaderTag existingTag;
    if (p->PeekPacketTag(existingTag))
    {
        NS_LOG_DEBUG("Packet already has GpsrHeaderTag (type " << (int)existingTag.GetType() 
                     << "), skipping AddHeaders");
        // Just pass the packet through without adding POS headers
        if (!m_downTarget.IsNull())
        {
            m_downTarget(p, source, destination, protocol, route);
        }
        return;
    }

    Ptr<MobilityModel> mm = m_ipv4->GetObject<MobilityModel>();
    Vector myPos = mm->GetPosition();

    // FIX: Use route->GetGateway() instead of recalculating nextHop
    // This ensures consistency with the routing decision made in RouteOutput()
    // If route is null or gateway is zero, fall back to recalculating
    Ipv4Address nextHop = Ipv4Address::GetZero();
    if (route && route->GetGateway() != Ipv4Address::GetZero())
    {
        nextHop = route->GetGateway();
        NS_LOG_DEBUG("AddHeaders: using route gateway " << nextHop);
    }
    else if (m_neighbors.IsNeighbour(destination))
    {
        nextHop = destination;
        NS_LOG_DEBUG("AddHeaders: destination is neighbor, using " << nextHop);
    }
    else
    {
        Vector myVel = mm->GetVelocity();
        nextHop = m_neighbors.BestNeighborTwoHop(m_locationService->GetPosition(destination), myPos, myVel);
        NS_LOG_DEBUG("AddHeaders: calculated best neighbor " << nextHop);
    }

    double positionX = 0.0;
    double positionY = 0.0;
    uint32_t hdrTime = 0;

    if (destination != m_ipv4->GetAddress(1, 0).GetBroadcast())
    {
        Vector dstPos = m_locationService->GetPosition(destination);
        positionX = dstPos.x;
        positionY = dstPos.y;
        hdrTime = (uint32_t)m_locationService->GetEntryUpdateTime(destination).GetSeconds();
    }

    PositionHeader posHeader(positionX,
                             positionY,
                             hdrTime,
                             0.0,
                             0.0,
                             (uint8_t)0,
                             myPos.x,
                             myPos.y);
    p->AddHeader(posHeader);
    TypeHeader tHeader(GPSRTYPE_POS);
    p->AddHeader(tHeader);
    
    // Add tag to mark this packet as having GPSR headers (only if not already present)
    GpsrHeaderTag tag(GPSRTYPE_POS);
    if (!p->PeekPacketTag(tag))
    {
        p->AddPacketTag(tag);
    }
    
    // Add hop count tag for statistics (initial hop = 0)
    GpsrHopCountTag hopTag(0);
    p->AddPacketTag(hopTag);
    
    // Add GpsrNextHopTag to pass next-hop info to EpcUeNas for TFT matching
    // Remove any existing tag first (in case of re-routing)
    GpsrNextHopTag existingNhTag;
    if (p->PeekPacketTag(existingNhTag))
    {
        p->RemovePacketTag(existingNhTag);
    }
    GpsrNextHopTag nhTag(nextHop, 64);  // Default TTL = 64
    p->AddPacketTag(nhTag);
    
    // DEBUG: Verify tag was added successfully
    GpsrNextHopTag verifyTag;
    bool hasNhTag = p->PeekPacketTag(verifyTag);
    NS_LOG_DEBUG("AddHeaders: packet UID=" << p->GetUid() 
                 << " nextHop=" << nextHop 
                 << " tagAdded=" << hasNhTag 
                 << " verifyNextHop=" << (hasNhTag ? verifyTag.GetNextHop() : Ipv4Address::GetZero()));
    
    // DEBUG: Verify header was written correctly
    {
        TypeHeader tmpT;
        p->PeekHeader(tmpT);
        PositionHeader tmpP;
        Ptr<Packet> copy = p->Copy();
        copy->RemoveHeader(tmpT);
        copy->PeekHeader(tmpP);
        NS_LOG_DEBUG("AddHeaders TX-CHECK: pktSize=" << p->GetSize() 
                     << " nhops=" << (int)tmpP.GetNhops());
    }

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
