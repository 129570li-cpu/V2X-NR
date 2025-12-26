/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Position Table (Neighbor Table) - Complete Implementation
 */

#ifndef GPSR_PTABLE_H
#define GPSR_PTABLE_H

#include "ns3/callback.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/vector.h"
#include "ns3/wifi-mac-header.h"

#include <map>

namespace ns3
{
namespace gpsr
{

/**
 * \ingroup gpsr
 * \brief Position table used by GPSR to store neighbor positions
 */
class PositionTable
{
  public:
    PositionTable();

    /**
     * \brief Gets the last time the entry was updated
     * \param id IPv4 address to get time of update from
     * \return Time of last update to the position
     */
    Time GetEntryUpdateTime(Ipv4Address id);

    /**
     * \brief Adds/updates entry in position table
     * \param id IPv4 address of the neighbor
     * \param position Position of the neighbor
     */
    void AddEntry(Ipv4Address id, Vector position);

    /**
     * \brief Deletes entry in position table
     * \param id IPv4 address to delete
     */
    void DeleteEntry(Ipv4Address id);

    /**
     * \brief Gets position from position table (uses God mode)
     * \param id IPv4 address to get position from
     * \return Position of that id
     */
    Vector GetPosition(Ipv4Address id);

    /**
     * \brief Checks if a node is a neighbor
     * \param id IPv4 address of the node to check
     * \return True if the node is a neighbor
     */
    bool IsNeighbour(Ipv4Address id);

    /**
     * \brief Remove entries with expired lifetime
     */
    void Purge();

    /**
     * \brief Clear all entries
     */
    void Clear();

    /**
     * \brief Gets best neighbor for greedy forwarding
     * \param dstPosition Position of the destination
     * \param nodePos Position of the current node
     * \return IPv4 address of the best neighbor, GetZero() if none found
     */
    Ipv4Address BestNeighbor(Vector dstPosition, Vector nodePos);

    /**
     * \brief Gets best neighbor for perimeter forwarding (right-hand rule)
     * \param previousHop Position of the previous hop
     * \param nodePos Position of the current node
     * \return IPv4 address of the best neighbor
     */
    Ipv4Address BestAngle(Vector previousHop, Vector nodePos);

    /**
     * \brief Calculate angle between vectors (counterclockwise)
     * \param centrePos Center position
     * \param refPos Reference position
     * \param node Node position
     * \return Angle in degrees
     */
    double GetAngle(Vector centrePos, Vector refPos, Vector node);

    /**
     * \brief Get invalid position marker
     * \return Vector representing invalid position
     */
    static Vector GetInvalidPosition()
    {
        return Vector(-1, -1, 0);
    }

    /**
     * \brief Check if position is valid
     * \param pos Position to check
     * \return True if valid
     */
    static bool IsPositionValid(Vector pos)
    {
        return !(pos.x == -1 && pos.y == -1);
    }

    /**
     * \brief Check if search is in progress
     */
    bool IsInSearch(Ipv4Address id);

    /**
     * \brief Check if position is known
     */
    bool HasPosition(Ipv4Address id);

    /**
     * \brief Get formatted string of all neighbors for logging
     * \return String with all neighbor IPs and positions
     */
    std::string GetNeighborList();

    /**
     * \brief Get TX error callback
     */
    Callback<void, WifiMacHeader const&> GetTxErrorCallback() const;

    /**
     * \brief Update SINR for an EXISTING neighbor only.
     *        If neighbor does not exist, this update is ignored.
     * \param id IPv4 address of the neighbor
     * \param sinr The measured linear SINR value
     */
    void UpdateSinr(Ipv4Address id, double sinr);

    /**
     * \brief Neighbor entry structure with position and SINR data
     */
    struct NeighborEntry
    {
        Vector position;
        Time lastUpdate;                  // Position update time from HELLO
        double sinr = -1.0;               // Linear SINR. -1.0 = unknown/invalid
        Time lastSinrUpdate = Seconds(0); // Time of last valid SINR update
    };

  private:
    Time m_entryLifeTime;
    std::map<Ipv4Address, NeighborEntry> m_table;

    // TX error callback
    Callback<void, WifiMacHeader const&> m_txErrorCallback;

    /**
     * \brief Process layer 2 TX error notification
     */
    void ProcessTxError(WifiMacHeader const&);

    /**
     * \brief Calculate distance between two positions
     */
    double CalculateDistance(Vector a, Vector b);
};

} // namespace gpsr
} // namespace ns3

#endif /* GPSR_PTABLE_H */
