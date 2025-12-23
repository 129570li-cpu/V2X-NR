/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Packet Header Definitions
 */

#ifndef GPSR_PACKET_H
#define GPSR_PACKET_H

#include "ns3/header.h"
#include "ns3/ipv4-address.h"

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
 */
class HelloHeader : public Header
{
  public:
    HelloHeader(uint64_t originPosx = 0, uint64_t originPosy = 0);

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    void SetOriginPosx(uint64_t posx)
    {
        m_originPosx = posx;
    }

    uint64_t GetOriginPosx() const
    {
        return m_originPosx;
    }

    void SetOriginPosy(uint64_t posy)
    {
        m_originPosy = posy;
    }

    uint64_t GetOriginPosy() const
    {
        return m_originPosy;
    }

    bool operator==(const HelloHeader& o) const;

  private:
    uint64_t m_originPosx;
    uint64_t m_originPosy;
};

std::ostream& operator<<(std::ostream& os, const HelloHeader& h);

/**
 * \ingroup gpsr
 * \brief GPSR Position Header
 */
class PositionHeader : public Header
{
  public:
    PositionHeader(uint64_t dstPosx = 0,
                   uint64_t dstPosy = 0,
                   uint32_t updated = 0,
                   uint64_t recPosx = 0,
                   uint64_t recPosy = 0,
                   uint8_t inRec = 0,
                   uint64_t lastPosx = 0,
                   uint64_t lastPosy = 0);

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    void SetDstPosx(uint64_t posx)
    {
        m_dstPosx = posx;
    }

    uint64_t GetDstPosx() const
    {
        return m_dstPosx;
    }

    void SetDstPosy(uint64_t posy)
    {
        m_dstPosy = posy;
    }

    uint64_t GetDstPosy() const
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

    void SetRecPosx(uint64_t posx)
    {
        m_recPosx = posx;
    }

    uint64_t GetRecPosx() const
    {
        return m_recPosx;
    }

    void SetRecPosy(uint64_t posy)
    {
        m_recPosy = posy;
    }

    uint64_t GetRecPosy() const
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

    void SetLastPosx(uint64_t posx)
    {
        m_lastPosx = posx;
    }

    uint64_t GetLastPosx() const
    {
        return m_lastPosx;
    }

    void SetLastPosy(uint64_t posy)
    {
        m_lastPosy = posy;
    }

    uint64_t GetLastPosy() const
    {
        return m_lastPosy;
    }

    bool operator==(const PositionHeader& o) const;

  private:
    uint64_t m_dstPosx;
    uint64_t m_dstPosy;
    uint32_t m_updated;
    uint64_t m_recPosx;
    uint64_t m_recPosy;
    uint8_t m_inRec;
    uint64_t m_lastPosx;
    uint64_t m_lastPosy;
};

std::ostream& operator<<(std::ostream& os, const PositionHeader& h);

} // namespace gpsr
} // namespace ns3

#endif /* GPSR_PACKET_H */
