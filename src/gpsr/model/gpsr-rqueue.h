/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Request Queue
 */

#ifndef GPSR_RQUEUE_H
#define GPSR_RQUEUE_H

#include "ns3/ipv4-header.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"

#include <vector>

namespace ns3
{
namespace gpsr
{

/**
 * \ingroup gpsr
 * \brief Queue entry for deferred packet forwarding
 */
class QueueEntry
{
  public:
    typedef Ipv4RoutingProtocol::UnicastForwardCallback UnicastForwardCallback;
    typedef Ipv4RoutingProtocol::ErrorCallback ErrorCallback;

    QueueEntry(Ptr<const Packet> pa = nullptr,
               const Ipv4Header& h = Ipv4Header(),
               UnicastForwardCallback ucb = UnicastForwardCallback(),
               ErrorCallback ecb = ErrorCallback())
        : m_packet(pa),
          m_header(h),
          m_ucb(ucb),
          m_ecb(ecb),
          m_expire(Seconds(0))
    {
    }

    bool operator==(const QueueEntry& o) const
    {
        return ((m_packet == o.m_packet) &&
                (m_header.GetDestination() == o.m_header.GetDestination()) &&
                (m_expire == o.m_expire));
    }

    Ptr<const Packet> GetPacket() const
    {
        return m_packet;
    }

    void SetPacket(Ptr<const Packet> p)
    {
        m_packet = p;
    }

    Ipv4Header GetIpv4Header() const
    {
        return m_header;
    }

    void SetIpv4Header(Ipv4Header h)
    {
        m_header = h;
    }

    UnicastForwardCallback GetUnicastForwardCallback() const
    {
        return m_ucb;
    }

    void SetUnicastForwardCallback(UnicastForwardCallback ucb)
    {
        m_ucb = ucb;
    }

    ErrorCallback GetErrorCallback() const
    {
        return m_ecb;
    }

    void SetErrorCallback(ErrorCallback ecb)
    {
        m_ecb = ecb;
    }

    Time GetExpireTime() const
    {
        return m_expire;
    }

    void SetExpireTime(Time exp)
    {
        m_expire = exp;
    }

  private:
    Ptr<const Packet> m_packet;
    Ipv4Header m_header;
    UnicastForwardCallback m_ucb;
    ErrorCallback m_ecb;
    Time m_expire;
};

/**
 * \ingroup gpsr
 * \brief Packet queue for deferred forwarding
 */
class RequestQueue
{
  public:
    RequestQueue(uint32_t maxLen = 64, Time maxTime = Seconds(30));

    /**
     * \brief Push entry to queue
     * \return True if successful
     */
    bool Enqueue(QueueEntry& entry);

    /**
     * \brief Remove and return first entry for destination
     * \param dst Destination address
     * \param entry Output entry
     * \return True if found
     */
    bool Dequeue(Ipv4Address dst, QueueEntry& entry);

    /**
     * \brief Drop all packets for destination
     * \param dst Destination address
     */
    void DropPacketWithDst(Ipv4Address dst);

    /**
     * \brief Find entry for destination
     * \param dst Destination address
     * \param entry Output entry
     * \return True if found
     */
    bool Find(Ipv4Address dst, QueueEntry& entry);

    /**
     * \brief Get queue size
     */
    uint32_t GetSize() const
    {
        return m_queue.size();
    }

  private:
    std::vector<QueueEntry> m_queue;
    uint32_t m_maxLen;
    Time m_queueTimeout;

    /**
     * \brief Remove expired entries
     */
    void Purge();

    /**
     * \brief Drop packet and call error callback
     * \param entry Queue entry to drop
     * \param reason Reason for dropping
     */
    void Drop(QueueEntry& entry, std::string reason);
};

} // namespace gpsr
} // namespace ns3

#endif /* GPSR_RQUEUE_H */
