/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Packet Header Implementation
 */

#include "gpsr-packet.h"

#include "ns3/address-utils.h"
#include "ns3/log.h"
#include "ns3/packet.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("GpsrPacket");

namespace gpsr
{

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

HelloHeader::HelloHeader(uint64_t originPosx, uint64_t originPosy)
    : m_originPosx(originPosx),
      m_originPosy(originPosy)
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
    return 16; // 2 * uint64_t
}

void
HelloHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteHtonU64(m_originPosx);
    i.WriteHtonU64(m_originPosy);
}

uint32_t
HelloHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_originPosx = i.ReadNtohU64();
    m_originPosy = i.ReadNtohU64();
    uint32_t dist = i.GetDistanceFrom(start);
    NS_ASSERT(dist == GetSerializedSize());
    return dist;
}

void
HelloHeader::Print(std::ostream& os) const
{
    os << "OriginPosition: " << m_originPosx << " " << m_originPosy;
}

bool
HelloHeader::operator==(const HelloHeader& o) const
{
    return (m_originPosx == o.m_originPosx && m_originPosy == o.m_originPosy);
}

std::ostream&
operator<<(std::ostream& os, const HelloHeader& h)
{
    h.Print(os);
    return os;
}

// PositionHeader

NS_OBJECT_ENSURE_REGISTERED(PositionHeader);

PositionHeader::PositionHeader(uint64_t dstPosx,
                               uint64_t dstPosy,
                               uint32_t updated,
                               uint64_t recPosx,
                               uint64_t recPosy,
                               uint8_t inRec,
                               uint64_t lastPosx,
                               uint64_t lastPosy)
    : m_dstPosx(dstPosx),
      m_dstPosy(dstPosy),
      m_updated(updated),
      m_recPosx(recPosx),
      m_recPosy(recPosy),
      m_inRec(inRec),
      m_lastPosx(lastPosx),
      m_lastPosy(lastPosy)
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
    return 53; // 6*uint64_t + uint32_t + uint8_t
}

void
PositionHeader::Serialize(Buffer::Iterator i) const
{
    i.WriteHtonU64(m_dstPosx);
    i.WriteHtonU64(m_dstPosy);
    i.WriteHtonU32(m_updated);
    i.WriteHtonU64(m_recPosx);
    i.WriteHtonU64(m_recPosy);
    i.WriteU8(m_inRec);
    i.WriteHtonU64(m_lastPosx);
    i.WriteHtonU64(m_lastPosy);
}

uint32_t
PositionHeader::Deserialize(Buffer::Iterator start)
{
    Buffer::Iterator i = start;
    m_dstPosx = i.ReadNtohU64();
    m_dstPosy = i.ReadNtohU64();
    m_updated = i.ReadNtohU32();
    m_recPosx = i.ReadNtohU64();
    m_recPosy = i.ReadNtohU64();
    m_inRec = i.ReadU8();
    m_lastPosx = i.ReadNtohU64();
    m_lastPosy = i.ReadNtohU64();
    uint32_t dist = i.GetDistanceFrom(start);
    NS_ASSERT(dist == GetSerializedSize());
    return dist;
}

void
PositionHeader::Print(std::ostream& os) const
{
    os << "DstPos: (" << m_dstPosx << "," << m_dstPosy << ") "
       << "Updated: " << m_updated << " "
       << "RecPos: (" << m_recPosx << "," << m_recPosy << ") "
       << "InRec: " << (int)m_inRec << " "
       << "LastPos: (" << m_lastPosx << "," << m_lastPosy << ")";
}

bool
PositionHeader::operator==(const PositionHeader& o) const
{
    return (m_dstPosx == o.m_dstPosx && m_dstPosy == o.m_dstPosy && m_updated == o.m_updated &&
            m_recPosx == o.m_recPosx && m_recPosy == o.m_recPosy && m_inRec == o.m_inRec &&
            m_lastPosx == o.m_lastPosx && m_lastPosy == o.m_lastPosy);
}

std::ostream&
operator<<(std::ostream& os, const PositionHeader& h)
{
    h.Print(os);
    return os;
}

} // namespace gpsr
} // namespace ns3
