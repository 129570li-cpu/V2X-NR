/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 University of Washington
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef LOCATION_SERVICE_H
#define LOCATION_SERVICE_H

#include "ns3/ipv4.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/vector.h"

namespace ns3
{

/**
 * \ingroup location-service
 * \brief Abstract base class for location services
 *
 * This class provides an interface for location services used by
 * geographic routing protocols like GPSR.
 */
class LocationService : public Object
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    LocationService();
    virtual ~LocationService();

    /**
     * \brief Get the position of a node by its IP address
     * \param adr The IPv4 address of the node
     * \return The position vector of the node
     */
    virtual Vector GetPosition(Ipv4Address adr) = 0;

    /**
     * \brief Check if the location service has the position of a node
     * \param adr The IPv4 address of the node
     * \return True if position is known
     */
    virtual bool HasPosition(Ipv4Address adr) = 0;

    /**
     * \brief Check if a search for the node is in progress
     * \param adr The IPv4 address of the node
     * \return True if search is in progress
     */
    virtual bool IsInSearch(Ipv4Address adr) = 0;

    /**
     * \brief Set the IPv4 protocol for this location service
     * \param ipv4 The IPv4 protocol
     */
    virtual void SetIpv4(Ptr<Ipv4> ipv4) = 0;

    /**
     * \brief Get an invalid position marker
     * \return Vector representing invalid position
     */
    virtual Vector GetInvalidPosition() = 0;

    /**
     * \brief Get the last update time for an entry
     * \param id The IPv4 address
     * \return Time of last update
     */
    virtual Time GetEntryUpdateTime(Ipv4Address id) = 0;

    /**
     * \brief Add or update an entry in the location table
     * \param id The IPv4 address
     * \param position The position
     */
    virtual void AddEntry(Ipv4Address id, Vector position) = 0;

    /**
     * \brief Delete an entry from the location table
     * \param id The IPv4 address
     */
    virtual void DeleteEntry(Ipv4Address id) = 0;

    /**
     * \brief Remove expired entries
     */
    virtual void Purge() = 0;

    /**
     * \brief Clear all entries
     */
    virtual void Clear() = 0;
};

} // namespace ns3

#endif /* LOCATION_SERVICE_H */
