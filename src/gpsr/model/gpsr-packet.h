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

#include <iostream>

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
 * \brief GPSR Hello Header
 * Uses double for coordinates to support negative values and decimals
 */
class HelloHeader : public Header
{
  public:
    HelloHeader(double originPosx = 0.0, double originPosy = 0.0);

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    void SetOriginPosx(double posx)
    {
        m_originPosx = posx;
    }

    double GetOriginPosx() const
    {
        return m_originPosx;
    }

    void SetOriginPosy(double posy)
    {
        m_originPosy = posy;
    }

    double GetOriginPosy() const
    {
        return m_originPosy;
    }

    bool operator==(const HelloHeader& o) const;

  private:
    double m_originPosx;
    double m_originPosy;
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

} // namespace gpsr
} // namespace ns3

#endif /* GPSR_PACKET_H */
