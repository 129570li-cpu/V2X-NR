/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Request Queue Implementation
 */

#include "gpsr-rqueue.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("GpsrRequestQueue");

namespace gpsr
{

RequestQueue::RequestQueue(uint32_t maxLen, Time maxTime)
    : m_maxLen(maxLen),
      m_queueTimeout(maxTime)
{
}

void
RequestQueue::Purge()
{
    NS_LOG_FUNCTION(this);
    
    // Drop and notify for expired packets
    for (auto it = m_queue.begin(); it != m_queue.end();)
    {
        if (it->GetExpireTime() < Simulator::Now())
        {
            Drop(*it, "Drop outdated packet");
            it = m_queue.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void
RequestQueue::Drop(QueueEntry& entry, std::string reason)
{
    NS_LOG_LOGIC(reason << " " << entry.GetPacket()->GetUid() 
                 << " " << entry.GetIpv4Header().GetDestination());
    
    QueueEntry::ErrorCallback ecb = entry.GetErrorCallback();
    if (!ecb.IsNull())
    {
        ecb(entry.GetPacket(), entry.GetIpv4Header(), Socket::ERROR_NOROUTETOHOST);
    }
}

bool
RequestQueue::Enqueue(QueueEntry& entry)
{
    NS_LOG_FUNCTION(this);

    Purge();

    // Check for duplicate
    for (const auto& e : m_queue)
    {
        if ((e.GetPacket()->GetUid() == entry.GetPacket()->GetUid()) &&
            (e.GetIpv4Header().GetDestination() == entry.GetIpv4Header().GetDestination()))
        {
            return false;
        }
    }

    entry.SetExpireTime(Simulator::Now() + m_queueTimeout);

    // If queue is full, drop the oldest packet (front) and notify via error callback
    if (m_queue.size() >= m_maxLen)
    {
        Drop(m_queue.front(), "Drop the most aged packet");
        m_queue.erase(m_queue.begin());
    }

    m_queue.push_back(entry);
    NS_LOG_DEBUG("Enqueued packet for " << entry.GetIpv4Header().GetDestination());
    return true;
}

bool
RequestQueue::Dequeue(Ipv4Address dst, QueueEntry& entry)
{
    NS_LOG_FUNCTION(this << dst);

    Purge();

    for (auto it = m_queue.begin(); it != m_queue.end(); ++it)
    {
        if (it->GetIpv4Header().GetDestination() == dst)
        {
            entry = *it;
            m_queue.erase(it);
            NS_LOG_DEBUG("Dequeued packet for " << dst);
            return true;
        }
    }

    return false;
}

void
RequestQueue::DropPacketWithDst(Ipv4Address dst)
{
    NS_LOG_FUNCTION(this << dst);

    m_queue.erase(std::remove_if(m_queue.begin(),
                                  m_queue.end(),
                                  [dst](const QueueEntry& e) {
                                      return (e.GetIpv4Header().GetDestination() == dst);
                                  }),
                  m_queue.end());
}

bool
RequestQueue::Find(Ipv4Address dst, QueueEntry& entry)
{
    NS_LOG_FUNCTION(this << dst);

    Purge();

    for (auto& e : m_queue)
    {
        if (e.GetIpv4Header().GetDestination() == dst)
        {
            entry = e;
            return true;
        }
    }

    return false;
}

} // namespace gpsr
} // namespace ns3
