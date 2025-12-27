/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Helper
 */

#ifndef GPSR_HELPER_H
#define GPSR_HELPER_H

#include "ns3/gpsr.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/node-container.h"
#include "ns3/node.h"
#include "ns3/object-factory.h"

namespace ns3
{

/**
 * \ingroup gpsr
 * \brief Helper class for GPSR routing protocol
 */
class GpsrHelper : public Ipv4RoutingHelper
{
  public:
    GpsrHelper();
    ~GpsrHelper() override;

    /**
     * \returns pointer to clone of this GpsrHelper
     */
    GpsrHelper* Copy() const override;

    /**
     * \param node the node on which the routing protocol will run
     * \returns a newly-created routing protocol
     */
    Ptr<Ipv4RoutingProtocol> Create(Ptr<Node> node) const override;

    /**
     * \param name the name of the attribute to set
     * \param value the value of the attribute to set
     */
    void Set(std::string name, const AttributeValue& value);

    /**
     * \brief Install GPSR on all nodes in a container
     * \param nodes Node container
     */
    void Install(NodeContainer nodes) const;

    /**
     * \brief Install GPSR on all nodes globally
     */
    void InstallAll() const;

  private:
    ObjectFactory m_agentFactory;
};

} // namespace ns3

#endif /* GPSR_HELPER_H */
