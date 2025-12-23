/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * God Location Service Implementation
 */

#define NS_LOG_APPEND_CONTEXT                                                                      \
    if (m_ipv4)                                                                                    \
    {                                                                                              \
        std::clog << "[node " << m_ipv4->GetObject<Node>()->GetId() << "] ";                       \
    }

#include "god.h"

#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("GodLocationService");

NS_OBJECT_ENSURE_REGISTERED(GodLocationService);

TypeId
GodLocationService::GetTypeId()
{
    static TypeId tid = TypeId("ns3::GodLocationService")
                            .SetParent<LocationService>()
                            .SetGroupName("LocationService")
                            .AddConstructor<GodLocationService>();
    return tid;
}

GodLocationService::GodLocationService(Time tableLifeTime)
    : m_ipv4(nullptr)
{
    NS_LOG_FUNCTION(this << tableLifeTime);
}

GodLocationService::GodLocationService()
    : m_ipv4(nullptr)
{
    NS_LOG_FUNCTION(this);
}

GodLocationService::~GodLocationService()
{
    NS_LOG_FUNCTION(this);
}

void
GodLocationService::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_ipv4 = nullptr;
}

void
GodLocationService::Start()
{
    NS_LOG_FUNCTION(this);
}

Vector
GodLocationService::GetPosition(Ipv4Address adr)
{
    NS_LOG_FUNCTION(this << adr);

    uint32_t n = NodeList::GetNNodes();

    for (uint32_t i = 0; i < n; i++)
    {
        Ptr<Node> node = NodeList::GetNode(i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();

        if (ipv4)
        {
            // Check all interfaces
            for (uint32_t j = 1; j < ipv4->GetNInterfaces(); j++)
            {
                for (uint32_t k = 0; k < ipv4->GetNAddresses(j); k++)
                {
                    if (ipv4->GetAddress(j, k).GetLocal() == adr)
                    {
                        Ptr<MobilityModel> mm = node->GetObject<MobilityModel>();
                        if (mm)
                        {
                            return mm->GetPosition();
                        }
                    }
                }
            }
        }
    }

    NS_LOG_WARN("Could not find position for " << adr);
    return GetInvalidPosition();
}

bool
GodLocationService::HasPosition(Ipv4Address adr)
{
    NS_LOG_FUNCTION(this << adr);
    // God knows everything - always returns true
    return true;
}

bool
GodLocationService::IsInSearch(Ipv4Address adr)
{
    NS_LOG_FUNCTION(this << adr);
    // God knows everything instantly - never searching
    return false;
}

void
GodLocationService::SetIpv4(Ptr<Ipv4> ipv4)
{
    NS_LOG_FUNCTION(this << ipv4);
    m_ipv4 = ipv4;
}

Vector
GodLocationService::GetInvalidPosition()
{
    return Vector(-1, -1, 0);
}

Time
GodLocationService::GetEntryUpdateTime(Ipv4Address id)
{
    NS_LOG_FUNCTION(this << id);
    // God has real-time knowledge
    return Simulator::Now();
}

void
GodLocationService::AddEntry(Ipv4Address id, Vector position)
{
    NS_LOG_FUNCTION(this << id << position);
    // God doesn't need to store entries - it knows all positions
}

void
GodLocationService::DeleteEntry(Ipv4Address id)
{
    NS_LOG_FUNCTION(this << id);
    // Nothing to delete in God mode
}

void
GodLocationService::Purge()
{
    NS_LOG_FUNCTION(this);
    // Nothing to purge in God mode
}

void
GodLocationService::Clear()
{
    NS_LOG_FUNCTION(this);
    // Nothing to clear in God mode
}

} // namespace ns3
