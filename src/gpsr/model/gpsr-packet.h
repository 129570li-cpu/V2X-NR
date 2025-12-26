/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Packet Header Definitions
 * Modified: Use double for coordinates to support negative values
 */

#ifndef GPSR_PACKET_H
#define GPSR_PACKET_H

#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/tag.h"

#include <cstring>
#include <iostream>
#include <vector>

namespace ns3
{
namespace gpsr
{

/**
 * \ingroup gpsr
 * \brief GPSR packet types
 */
enum MessageType
{
    GPSRTYPE_HELLO = 1,
    GPSRTYPE_POS = 2
};

/**
 * \ingroup gpsr
 * \brief GPSR Type Header
 */
class TypeHeader : public Header
{
  public:
    TypeHeader(MessageType t = GPSRTYPE_HELLO);

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    MessageType Get() const
    {
        return m_type;
    }

    bool IsValid() const
    {
        return m_valid;
    }

    bool operator==(const TypeHeader& o) const;

  private:
    MessageType m_type;
    bool m_valid;
};

std::ostream& operator<<(std::ostream& os, const TypeHeader& h);

/**
 * \ingroup gpsr
 * \brief Compact neighbor summary for Hello piggybacking
 * 
 * Contains essential neighbor info for two-hop routing decisions.
 * Uses explicit serialization to avoid padding issues.
 * Size: 4(IP) + 4(x) + 4(y) + 4(vx) + 4(vy) + 1(lq) = 21 bytes
 */
struct NeighborSummary
{
    Ipv4Address ip;      ///< Neighbor IP address
    float x;             ///< Position X coordinate
    float y;             ///< Position Y coordinate  
    float vx;            ///< Velocity X component
    float vy;            ///< Velocity Y component
    uint8_t linkQuality; ///< Quantized link quality (0-255, mapped from SINR)
    
    /// Default constructor
    NeighborSummary()
        : ip(Ipv4Address::GetZero()), x(0), y(0), vx(0), vy(0), linkQuality(0)
    {
    }
    
    /// Constructor with all fields
    NeighborSummary(Ipv4Address addr, float px, float py, float velx, float vely, uint8_t lq)
        : ip(addr), x(px), y(py), vx(velx), vy(vely), linkQuality(lq)
    {
    }
    
    /// Get serialized size (fixed 21 bytes)
    static uint32_t GetSerializedSize()
    {
        return 4 + 4 + 4 + 4 + 4 + 1; // 21 bytes
    }
    
    /// Serialize to buffer (explicit byte-packing, no padding)
    void Serialize(Buffer::Iterator& i) const
    {
        i.WriteHtonU32(ip.Get());
        // Write floats as raw bytes (IEEE 754)
        uint32_t xBits, yBits, vxBits, vyBits;
        std::memcpy(&xBits, &x, sizeof(float));
        std::memcpy(&yBits, &y, sizeof(float));
        std::memcpy(&vxBits, &vx, sizeof(float));
        std::memcpy(&vyBits, &vy, sizeof(float));
        i.WriteHtonU32(xBits);
        i.WriteHtonU32(yBits);
        i.WriteHtonU32(vxBits);
        i.WriteHtonU32(vyBits);
        i.WriteU8(linkQuality);
    }
    
    /// Deserialize from buffer
    void Deserialize(Buffer::Iterator& i)
    {
        ip = Ipv4Address(i.ReadNtohU32());
        uint32_t xBits = i.ReadNtohU32();
        uint32_t yBits = i.ReadNtohU32();
        uint32_t vxBits = i.ReadNtohU32();
        uint32_t vyBits = i.ReadNtohU32();
        std::memcpy(&x, &xBits, sizeof(float));
        std::memcpy(&y, &yBits, sizeof(float));
        std::memcpy(&vx, &vxBits, sizeof(float));
        std::memcpy(&vy, &vyBits, sizeof(float));
        linkQuality = i.ReadU8();
    }
};

/**
 * \ingroup gpsr
 * \brief Extended GPSR Hello Header for Two-Hop Routing
 * 
 * Carries sender's position, velocity, timestamp, and Top-K neighbor summaries.
 * Uses explicit serialization to avoid padding issues.
 */
class HelloHeader : public Header
{
  public:
    /// Maximum number of neighbors to piggyback (Top-K)
    static constexpr uint8_t MAX_NEIGHBORS = 5;
    
    HelloHeader(double originPosx = 0.0, double originPosy = 0.0);

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    // Position accessors (existing)
    void SetOriginPosx(double posx) { m_originPosx = posx; }
    double GetOriginPosx() const { return m_originPosx; }
    void SetOriginPosy(double posy) { m_originPosy = posy; }
    double GetOriginPosy() const { return m_originPosy; }
    
    // Velocity accessors (new)
    void SetVelocity(double vx, double vy) { m_velocityX = vx; m_velocityY = vy; }
    double GetVelocityX() const { return m_velocityX; }
    double GetVelocityY() const { return m_velocityY; }
    
    // Timestamp accessor (new)
    void SetTimestamp(uint32_t ts) { m_timestamp = ts; }
    uint32_t GetTimestamp() const { return m_timestamp; }
    
    // Neighbor list accessors (new)
    void SetNeighbors(const std::vector<NeighborSummary>& neighbors);
    const std::vector<NeighborSummary>& GetNeighbors() const { return m_neighbors; }
    uint8_t GetNeighborCount() const { return static_cast<uint8_t>(m_neighbors.size()); }

    bool operator==(const HelloHeader& o) const;

  private:
    double m_originPosx;      ///< Sender's X position
    double m_originPosy;      ///< Sender's Y position
    double m_velocityX;       ///< Sender's X velocity component
    double m_velocityY;       ///< Sender's Y velocity component
    uint32_t m_timestamp;     ///< Timestamp (ms since epoch or simulation start)
    std::vector<NeighborSummary> m_neighbors; ///< Top-K neighbor summaries
};

std::ostream& operator<<(std::ostream& os, const HelloHeader& h);

/**
 * \ingroup gpsr
 * \brief GPSR Position Header
 * Uses double for coordinates to support negative values and decimals
 */
class PositionHeader : public Header
{
  public:
    PositionHeader(double dstPosx = 0.0,
                   double dstPosy = 0.0,
                   uint32_t updated = 0,
                   double recPosx = 0.0,
                   double recPosy = 0.0,
                   uint8_t inRec = 0,
                   double lastPosx = 0.0,
                   double lastPosy = 0.0);

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    void SetDstPosx(double posx)
    {
        m_dstPosx = posx;
    }

    double GetDstPosx() const
    {
        return m_dstPosx;
    }

    void SetDstPosy(double posy)
    {
        m_dstPosy = posy;
    }

    double GetDstPosy() const
    {
        return m_dstPosy;
    }

    void SetUpdated(uint32_t updated)
    {
        m_updated = updated;
    }

    uint32_t GetUpdated() const
    {
        return m_updated;
    }

    void SetRecPosx(double posx)
    {
        m_recPosx = posx;
    }

    double GetRecPosx() const
    {
        return m_recPosx;
    }

    void SetRecPosy(double posy)
    {
        m_recPosy = posy;
    }

    double GetRecPosy() const
    {
        return m_recPosy;
    }

    void SetInRec(uint8_t inRec)
    {
        m_inRec = inRec;
    }

    uint8_t GetInRec() const
    {
        return m_inRec;
    }

    void SetLastPosx(double posx)
    {
        m_lastPosx = posx;
    }

    double GetLastPosx() const
    {
        return m_lastPosx;
    }

    void SetLastPosy(double posy)
    {
        m_lastPosy = posy;
    }

    double GetLastPosy() const
    {
        return m_lastPosy;
    }

    bool operator==(const PositionHeader& o) const;

  private:
    double m_dstPosx;
    double m_dstPosy;
    uint32_t m_updated;
    double m_recPosx;
    double m_recPosy;
    uint8_t m_inRec;
    double m_lastPosx;
    double m_lastPosy;
};

std::ostream& operator<<(std::ostream& os, const PositionHeader& h);

/**
 * \ingroup gpsr
 * \brief Tag to mark packets with GPSR headers
 * 
 * This tag is added when GPSR headers are added to a packet.
 * It allows safe detection of GPSR headers before calling RemoveHeader,
 * avoiding NS-3 metadata errors when packet content accidentally matches
 * GPSR header format.
 */
class GpsrHeaderTag : public Tag
{
  public:
    GpsrHeaderTag()
        : m_type(GPSRTYPE_POS)
    {
    }

    GpsrHeaderTag(MessageType type)
        : m_type(type)
    {
    }

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::gpsr::GpsrHeaderTag")
                                .SetParent<Tag>()
                                .SetGroupName("Gpsr")
                                .AddConstructor<GpsrHeaderTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return 1;  // Just store the type byte
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU8(static_cast<uint8_t>(m_type));
    }

    void Deserialize(TagBuffer i) override
    {
        m_type = static_cast<MessageType>(i.ReadU8());
    }

    void Print(std::ostream& os) const override
    {
        os << "GpsrHeaderTag type=" << static_cast<int>(m_type);
    }

    MessageType GetType() const
    {
        return m_type;
    }

  private:
    MessageType m_type;
};

/**
 * \ingroup gpsr
 * \brief Tag to pass next-hop information from GPSR to EpcUeNas
 * 
 * This tag is used internally within a node to communicate the GPSR-selected
 * next-hop IP address to the NAS layer for correct TFT matching in NR Sidelink.
 * It also carries TTL for hop counting.
 */
class GpsrNextHopTag : public Tag
{
  public:
    GpsrNextHopTag()
        : m_nextHop(Ipv4Address::GetAny()),
          m_ttl(64)
    {
    }

    GpsrNextHopTag(Ipv4Address nextHop, uint8_t ttl = 64)
        : m_nextHop(nextHop),
          m_ttl(ttl)
    {
    }

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::gpsr::GpsrNextHopTag")
                                .SetParent<Tag>()
                                .SetGroupName("Gpsr")
                                .AddConstructor<GpsrNextHopTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return 4 + 1;  // IPv4 address (4 bytes) + TTL (1 byte)
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU32(m_nextHop.Get());
        i.WriteU8(m_ttl);
    }

    void Deserialize(TagBuffer i) override
    {
        m_nextHop = Ipv4Address(i.ReadU32());
        m_ttl = i.ReadU8();
    }

    void Print(std::ostream& os) const override
    {
        os << "GpsrNextHopTag nextHop=" << m_nextHop << " ttl=" << (int)m_ttl;
    }

    Ipv4Address GetNextHop() const
    {
        return m_nextHop;
    }

    void SetNextHop(Ipv4Address nextHop)
    {
        m_nextHop = nextHop;
    }

    uint8_t GetTtl() const
    {
        return m_ttl;
    }

    void SetTtl(uint8_t ttl)
    {
        m_ttl = ttl;
    }

    void DecrementTtl()
    {
        if (m_ttl > 0)
        {
            m_ttl--;
        }
    }

  private:
    Ipv4Address m_nextHop;
    uint8_t m_ttl;
};

} // namespace gpsr
} // namespace ns3

#endif /* GPSR_PACKET_H */
