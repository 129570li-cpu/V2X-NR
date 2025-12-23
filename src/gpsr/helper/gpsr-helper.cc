/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Helper Implementation
 */

#include "gpsr-helper.h"

#include "ns3/gpsr.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/node-container.h"
#include "ns3/node-list.h"
#include "ns3/udp-l4-protocol.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("GpsrHelper");

GpsrHelper::GpsrHelper()
{
    m_agentFactory.SetTypeId("ns3::gpsr::RoutingProtocol");
}

GpsrHelper::~GpsrHelper()
{
}

GpsrHelper*
GpsrHelper::Copy() const
{
    return new GpsrHelper(*this);
}

Ptr<Ipv4RoutingProtocol>
GpsrHelper::Create(Ptr<Node> node) const
{
    NS_LOG_FUNCTION(this << node);
    Ptr<gpsr::RoutingProtocol> agent = m_agentFactory.Create<gpsr::RoutingProtocol>();
    node->AggregateObject(agent);
    return agent;
}

void
GpsrHelper::Set(std::string name, const AttributeValue& value)
{
    m_agentFactory.Set(name, value);
}

void
GpsrHelper::Install(NodeContainer nodes) const
{
    NS_LOG_FUNCTION(this);

    for (auto it = nodes.Begin(); it != nodes.End(); ++it)
    {
        Ptr<Node> node = *it;
        Ptr<gpsr::RoutingProtocol> gpsr = node->GetObject<gpsr::RoutingProtocol>();

        // Only create and install if GPSR doesn't already exist on this node
        // (e.g., it wasn't already installed via InternetStackHelper)
        if (!gpsr)
        {
            Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
            if (ipv4)
            {
                Ptr<Ipv4RoutingProtocol> proto = Create(node);
                ipv4->SetRoutingProtocol(proto);
                NS_LOG_DEBUG("Installed GPSR on node " << node->GetId());
            }
        }
    }

    // Wire up the m_downTarget callback chain for all nodes
    // This must be done after all nodes have GPSR and UDP installed
    for (auto it = nodes.Begin(); it != nodes.End(); ++it)
    {
        Ptr<Node> node = *it;
        Ptr<UdpL4Protocol> udp = node->GetObject<UdpL4Protocol>();
        Ptr<gpsr::RoutingProtocol> gpsr = node->GetObject<gpsr::RoutingProtocol>();

        if (udp && gpsr)
        {
            // Save UDP's original down target (IP layer send)
            gpsr->SetDownTarget(udp->GetDownTarget());
            // Intercept UDP's down target to add GPSR headers
            udp->SetDownTarget(MakeCallback(&gpsr::RoutingProtocol::AddHeaders, gpsr));
            NS_LOG_DEBUG("Wired up GPSR m_downTarget on node " << node->GetId());
        }
    }
}

void
GpsrHelper::InstallAll() const
{
    Install(NodeContainer::GetGlobal());
}

} // namespace ns3

