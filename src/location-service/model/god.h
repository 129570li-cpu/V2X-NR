/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * God Location Service - Omniscient location service for simulation
 */

#ifndef GOD_LOCATION_SERVICE_H
#define GOD_LOCATION_SERVICE_H

#include "location-service.h"

#include "ns3/ipv4.h"
#include "ns3/nstime.h"

namespace ns3
{

/**
 * \ingroup location-service
 * \brief God (omniscient) location service
 *
 * This location service has global knowledge of all node positions
 * in the simulation. It directly queries the MobilityModel of nodes
 * to get their positions.
 */
class GodLocationService : public LocationService
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * \brief Constructor with table lifetime
     * \param tableLifeTime Lifetime of table entries (not used in God mode)
     */
    GodLocationService(Time tableLifeTime);

    /**
     * \brief Default constructor
     */
    GodLocationService();

    /**
     * \brief Destructor
     */
    virtual ~GodLocationService();

    /**
     * \brief Dispose of the object
     */
    virtual void DoDispose();

    // Inherited from LocationService
    Vector GetPosition(Ipv4Address adr) override;
    bool HasPosition(Ipv4Address adr) override;
    bool IsInSearch(Ipv4Address adr) override;
    void SetIpv4(Ptr<Ipv4> ipv4) override;
    Vector GetInvalidPosition() override;
    Time GetEntryUpdateTime(Ipv4Address id) override;
    void AddEntry(Ipv4Address id, Vector position) override;
    void DeleteEntry(Ipv4Address id) override;
    void Purge() override;
    void Clear() override;

  private:
    Ptr<Ipv4> m_ipv4; //!< IPv4 protocol (optional, for context)

    /**
     * \brief Start the location service
     */
    void Start();
};

} // namespace ns3

#endif /* GOD_LOCATION_SERVICE_H */
