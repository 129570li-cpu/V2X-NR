/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Packet Header Implementation
 * Modified: Use double for coordinates to support negative values
 */

#include "gpsr-packet.h"

#include "ns3/address-utils.h"
#include "ns3/log.h"
#include "ns3/packet.h"

#include <cstring>
#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("GpsrPacket");

namespace gpsr
{

// Helper functions for serializing double as uint64_t (bit-level, preserves sign and precision)
static uint64_t
DoubleToUint64(double val)
{
    uint64_t result;
    std::memcpy(&result, &val, sizeof(double));
    return result;
}

static double
Uint64ToDouble(uint64_t val)
{
    double result;
    std::memcpy(&result, &val, sizeof(double));
    return result;
}

NS_OBJECT_ENSURE_REGISTERED(TypeHeader);

TypeHeader::TypeHeader(MessageType t)
    : m_type(t),
      m_valid(true)
{
}

TypeId
TypeHeader::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::gpsr::TypeHeader").SetParent<Header>().AddConstructor<TypeHeader>();
    return tid;
}

TypeId
TypeHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
TypeHeader::GetSerializedSize() const
{
    return 1;
}

void
TypeHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteU8((uint8_t)m_type);
}

uint32_t
TypeHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    uint8_t type = i.ReadU8();
    m_valid = true;
    switch (type)
    {
    case GPSRTYPE_HELLO:
    case GPSRTYPE_POS: {
        m_type = (MessageType)type;
        break;
    }
    default:
        m_valid = false;
    }
    uint32_t dist = i.GetDistanceFrom(start);
    NS_ASSERT(dist == GetSerializedSize());
    return dist;
}

void
TypeHeader::Print(std::ostream& os) const
{
    switch (m_type)
    {
    case GPSRTYPE_HELLO: {
        os << "HELLO";
        break;
    }
    case GPSRTYPE_POS: {
        os << "POSITION";
        break;
    }
    default:
        os << "UNKNOWN_TYPE";
    }
}

bool
TypeHeader::operator==(const TypeHeader& o) const
{
    return (m_type == o.m_type && m_valid == o.m_valid);
}

std::ostream&
operator<<(std::ostream& os, const TypeHeader& h)
{
    h.Print(os);
    return os;
}

// HelloHeader

NS_OBJECT_ENSURE_REGISTERED(HelloHeader);

HelloHeader::HelloHeader(double originPosx, double originPosy)
    : m_originPosx(originPosx),
      m_originPosy(originPosy),
      m_velocityX(0.0),
      m_velocityY(0.0),
      m_timestamp(0)
{
}

TypeId
HelloHeader::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::gpsr::HelloHeader").SetParent<Header>().AddConstructor<HelloHeader>();
    return tid;
}

TypeId
HelloHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
HelloHeader::GetSerializedSize() const
{
    // Position: 2 * 8 = 16 bytes
    // Velocity: 2 * 8 = 16 bytes  
    // Timestamp: 4 bytes
    // Sequence number: 2 bytes (LDT)
    // Neighbor count: 1 byte
    // Neighbors: count * 21 bytes
    return 16 + 16 + 4 + 2 + 1 + (m_neighbors.size() * NeighborSummary::GetSerializedSize());
}

void
HelloHeader::Serialize(Buffer::Iterator i) const
{
    // Position
    i.WriteHtonU64(DoubleToUint64(m_originPosx));
    i.WriteHtonU64(DoubleToUint64(m_originPosy));
    // Velocity
    i.WriteHtonU64(DoubleToUint64(m_velocityX));
    i.WriteHtonU64(DoubleToUint64(m_velocityY));
    // Timestamp
    i.WriteHtonU32(m_timestamp);
    // Sequence number (LDT)
    i.WriteHtonU16(m_seq);
    // Neighbor count
    uint8_t count = static_cast<uint8_t>(std::min(m_neighbors.size(), 
                                                   static_cast<size_t>(MAX_NEIGHBORS)));
    i.WriteU8(count);
    // Neighbors
    for (uint8_t n = 0; n < count; ++n)
    {
        m_neighbors[n].Serialize(i);
    }
}

uint32_t
HelloHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    // Position
    m_originPosx = Uint64ToDouble(i.ReadNtohU64());
    m_originPosy = Uint64ToDouble(i.ReadNtohU64());
    // Velocity
    m_velocityX = Uint64ToDouble(i.ReadNtohU64());
    m_velocityY = Uint64ToDouble(i.ReadNtohU64());
    // Timestamp
    m_timestamp = i.ReadNtohU32();
    // Sequence number (LDT)
    m_seq = i.ReadNtohU16();
    // Neighbor count with bounds and buffer validation
    uint8_t originalCount = i.ReadU8();
    
    // Validate buffer has enough bytes for all neighbors
    uint32_t neighborSize = NeighborSummary::GetSerializedSize();
    uint32_t requiredBytes = originalCount * neighborSize;
    uint32_t remainingBytes = i.GetRemainingSize();
    
    if (requiredBytes > remainingBytes)
    {
        NS_LOG_WARN("HelloHeader: buffer too short (need " << requiredBytes 
                    << ", have " << remainingBytes << "), truncating to fit");
        originalCount = static_cast<uint8_t>(remainingBytes / neighborSize);
    }
    
    uint8_t count = originalCount;
    if (count > MAX_NEIGHBORS)
    {
        NS_LOG_WARN("HelloHeader: neighbor count " << (int)originalCount 
                    << " exceeds MAX_NEIGHBORS " << (int)MAX_NEIGHBORS << ", truncating");
        count = MAX_NEIGHBORS;
    }
    // Neighbors
    m_neighbors.clear();
    m_neighbors.reserve(count);
    for (uint8_t n = 0; n < count; ++n)
    {
        NeighborSummary ns;
        ns.Deserialize(i);
        m_neighbors.push_back(ns);
    }
    // Skip excess neighbor bytes to maintain correct packet parsing
    if (originalCount > count)
    {
        uint8_t skip = originalCount - count;
        i.Next(skip * neighborSize);
    }
    uint32_t dist = i.GetDistanceFrom(start);
    // Note: dist will include skipped bytes, matching the actual serialized size
    return dist;
}

void
HelloHeader::Print(std::ostream& os) const
{
    os << "Pos:(" << m_originPosx << "," << m_originPosy << ") "
       << "Vel:(" << m_velocityX << "," << m_velocityY << ") "
       << "TS:" << m_timestamp << " "
       << "Seq:" << m_seq << " "
       << "Neighbors:" << m_neighbors.size();
}

bool
HelloHeader::operator==(const HelloHeader& o) const
{
    return (m_originPosx == o.m_originPosx && m_originPosy == o.m_originPosy &&
            m_velocityX == o.m_velocityX && m_velocityY == o.m_velocityY &&
            m_timestamp == o.m_timestamp && m_seq == o.m_seq && 
            m_neighbors.size() == o.m_neighbors.size());
}

void
HelloHeader::SetNeighbors(const std::vector<NeighborSummary>& neighbors)
{
    // Only store up to MAX_NEIGHBORS
    m_neighbors.clear();
    size_t count = std::min(neighbors.size(), static_cast<size_t>(MAX_NEIGHBORS));
    for (size_t i = 0; i < count; ++i)
    {
        m_neighbors.push_back(neighbors[i]);
    }
}

std::ostream&
operator<<(std::ostream& os, const HelloHeader& h)
{
    h.Print(os);
    return os;
}

// PositionHeader

NS_OBJECT_ENSURE_REGISTERED(PositionHeader);

PositionHeader::PositionHeader(double dstPosx,
                               double dstPosy,
                               uint32_t updated,
                               double recPosx,
                               double recPosy,
                               uint8_t inRec,
                               double lastPosx,
                               double lastPosy,
                               double lfPosx,
                               double lfPosy,
                               uint32_t e0From,
                               uint32_t e0To)
    : m_dstPosx(dstPosx),
      m_dstPosy(dstPosy),
      m_updated(updated),
      m_recPosx(recPosx),
      m_recPosy(recPosy),
      m_inRec(inRec),
      m_lastPosx(lastPosx),
      m_lastPosy(lastPosy),
      m_lfPosx(lfPosx),
      m_lfPosy(lfPosy),
      m_e0From(e0From),
      m_e0To(e0To)
{
}

TypeId
PositionHeader::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::gpsr::PositionHeader").SetParent<Header>().AddConstructor<PositionHeader>();
    return tid;
}

TypeId
PositionHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
PositionHeader::GetSerializedSize() const
{
    // Base: 8*8 (doubles) + 4 (updated) + 1 (inRec) + 8 (e0From+e0To) + 8 (lfEdgeFrom+To) = 85 bytes
    // + 1 (hasHopList) + 1 (nhops) + 1 (forwardType) + nhops * 28
    return 88 + m_nhops * 28;
}

void
PositionHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteHtonU64(DoubleToUint64(m_dstPosx));
    i.WriteHtonU64(DoubleToUint64(m_dstPosy));
    i.WriteHtonU32(m_updated);
    i.WriteHtonU64(DoubleToUint64(m_recPosx));
    i.WriteHtonU64(DoubleToUint64(m_recPosy));
    i.WriteU8(m_inRec);
    i.WriteHtonU64(DoubleToUint64(m_lastPosx));
    i.WriteHtonU64(DoubleToUint64(m_lastPosy));
    i.WriteHtonU64(DoubleToUint64(m_lfPosx));
    i.WriteHtonU64(DoubleToUint64(m_lfPosy));
    i.WriteHtonU32(m_e0From);
    i.WriteHtonU32(m_e0To);
    i.WriteHtonU32(m_lfEdgeFrom);
    i.WriteHtonU32(m_lfEdgeTo);
    // hasHopList flag (方案A)
    i.WriteU8(m_hasHopList);
    // forwardType (PA-GPSR: R/L/G)
    i.WriteU8(m_forwardType);
    // Hop history
    i.WriteU8(m_nhops);
    for (uint8_t h = 0; h < m_nhops; h++)
    {
        i.WriteHtonU32(m_hops[h].ip);
        i.WriteHtonU64(DoubleToUint64(m_hops[h].x));
        i.WriteHtonU64(DoubleToUint64(m_hops[h].y));
        i.WriteHtonU64(DoubleToUint64(m_hops[h].z));
    }
}

uint32_t
PositionHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_dstPosx = Uint64ToDouble(i.ReadNtohU64());
    m_dstPosy = Uint64ToDouble(i.ReadNtohU64());
    m_updated = i.ReadNtohU32();
    m_recPosx = Uint64ToDouble(i.ReadNtohU64());
    m_recPosy = Uint64ToDouble(i.ReadNtohU64());
    m_inRec = i.ReadU8();
    m_lastPosx = Uint64ToDouble(i.ReadNtohU64());
    m_lastPosy = Uint64ToDouble(i.ReadNtohU64());
    m_lfPosx = Uint64ToDouble(i.ReadNtohU64());
    m_lfPosy = Uint64ToDouble(i.ReadNtohU64());
    m_e0From = i.ReadNtohU32();
    m_e0To = i.ReadNtohU32();
    m_lfEdgeFrom = i.ReadNtohU32();
    m_lfEdgeTo = i.ReadNtohU32();
    // hasHopList flag (方案A)
    m_hasHopList = i.ReadU8();
    // forwardType (PA-GPSR: R/L/G)
    m_forwardType = i.ReadU8();
    // Hop history
    uint8_t packetNhops = i.ReadU8();  // Actual count in packet
    uint8_t originalNhops = packetNhops;
    
    // Safety check: verify buffer has enough remaining data for stated hops
    const uint32_t hopSize = 28;  // 4 + 8 + 8 + 8
    uint32_t remainingBytes = i.GetRemainingSize();
    uint32_t maxFit = remainingBytes / hopSize;
    
    if (packetNhops > maxFit)
    {
        NS_LOG_WARN("PositionHeader: nhops=" << (int)originalNhops
                    << " > maxFit=" << maxFit
                    << " remaining=" << remainingBytes
                    << ", clamp");
        packetNhops = static_cast<uint8_t>(maxFit);
    }
    
    // 只有在确实处于周边模式时才允许解析 hop list
    if (m_hasHopList != 1 || m_inRec == 0 || m_e0From == 0 || m_e0To == 0)
    {
        packetNhops = 0;
    }
    
    // Clamp to storage limit
    m_nhops = std::min(packetNhops, MAX_PERI_HOPS);
    
    // Read hops we can store
    for (uint8_t h = 0; h < m_nhops; h++)
    {
        m_hops[h].ip = i.ReadNtohU32();
        m_hops[h].x = Uint64ToDouble(i.ReadNtohU64());
        m_hops[h].y = Uint64ToDouble(i.ReadNtohU64());
        m_hops[h].z = Uint64ToDouble(i.ReadNtohU64());
    }
    // Skip excess hops we can't store (28 bytes each) - use clamped packetNhops
    for (uint8_t h = m_nhops; h < packetNhops; h++)
    {
        i.ReadNtohU32();   // ip
        i.ReadNtohU64();   // x
        i.ReadNtohU64();   // y
        i.ReadNtohU64();   // z
    }
    uint32_t dist = i.GetDistanceFrom(start);
    NS_LOG_DEBUG("PositionHeader::Deserialize: hasHopList=" << (int)m_hasHopList
                 << " origNhops=" << (int)originalNhops
                 << " pktNhops=" << (int)packetNhops
                 << " m_nhops=" << (int)m_nhops
                 << " bytesRead=" << dist
                 << " remaining=" << remainingBytes);
    return dist;
}

void
PositionHeader::Print(std::ostream& os) const
{
    os << "DstPos: (" << m_dstPosx << "," << m_dstPosy << ") "
       << "Updated: " << m_updated << " "
       << "RecPos: (" << m_recPosx << "," << m_recPosy << ") "
       << "InRec: " << (int)m_inRec << " "
       << "LastPos: (" << m_lastPosx << "," << m_lastPosy << ") "
       << "LfPos: (" << m_lfPosx << "," << m_lfPosy << ") "
       << "e0: (" << m_e0From << "->" << m_e0To << ") "
       << "lfEdge: (" << m_lfEdgeFrom << "->" << m_lfEdgeTo << ") "
       << "hasHopList=" << (int)m_hasHopList << " "
       << "fwdType=" << (char)m_forwardType << " "
       << "Hops[" << (int)m_nhops << "]: ";
    for (uint8_t h = 0; h < m_nhops; h++)
    {
        os << m_hops[h].ip << "@(" << m_hops[h].x << "," << m_hops[h].y << "," << m_hops[h].z << ")";
        if (h < m_nhops - 1) os << "->";
    }
}

bool
PositionHeader::operator==(const PositionHeader& o) const
{
    if (m_dstPosx != o.m_dstPosx || m_dstPosy != o.m_dstPosy || m_updated != o.m_updated ||
        m_recPosx != o.m_recPosx || m_recPosy != o.m_recPosy || m_inRec != o.m_inRec ||
        m_lastPosx != o.m_lastPosx || m_lastPosy != o.m_lastPosy ||
        m_lfPosx != o.m_lfPosx || m_lfPosy != o.m_lfPosy ||
        m_e0From != o.m_e0From || m_e0To != o.m_e0To ||
        m_lfEdgeFrom != o.m_lfEdgeFrom || m_lfEdgeTo != o.m_lfEdgeTo ||
        m_hasHopList != o.m_hasHopList ||
        m_forwardType != o.m_forwardType ||
        m_nhops != o.m_nhops)
    {
        return false;
    }
    for (uint8_t h = 0; h < m_nhops; h++)
    {
        if (m_hops[h].ip != o.m_hops[h].ip ||
            m_hops[h].x != o.m_hops[h].x ||
            m_hops[h].y != o.m_hops[h].y ||
            m_hops[h].z != o.m_hops[h].z)
        {
            return false;
        }
    }
    return true;
}

std::ostream&
operator<<(std::ostream& os, const PositionHeader& h)
{
    h.Print(os);
    return os;
}

} // namespace gpsr
} // namespace ns3
