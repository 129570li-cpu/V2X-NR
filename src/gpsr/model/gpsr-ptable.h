/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Position Table (Neighbor Table) - Complete Implementation
 */

#ifndef GPSR_PTABLE_H
#define GPSR_PTABLE_H

#include "gpsr-packet.h"

#include "ns3/callback.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/vector.h"
#include "ns3/wifi-mac-header.h"

#include <map>
#include <vector>

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
     * \brief Adds/updates entry with extended info (velocity, two-hop neighbors)
     * \param id IPv4 address of the neighbor
     * \param position Position of the neighbor
     * \param velocity Velocity vector of the neighbor
     * \param twoHopNeighbors List of two-hop neighbors from this neighbor
     */
    void AddEntryExtended(Ipv4Address id, 
                          Vector position, 
                          Vector velocity,
                          const std::vector<NeighborSummary>& twoHopNeighbors);

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
     * \brief Gets best neighbor using two-hop aware scoring
     * Uses composite score: Progress + LinkQuality + LinkDuration
     * Falls back to standard greedy if no good candidates
     * \param dstPosition Position of the destination
     * \param nodePos Position of the current node
     * \param nodeVel Velocity of the current node
     * \return IPv4 address of the best neighbor, GetZero() if none found
     */
    Ipv4Address BestNeighborTwoHop(Vector dstPosition, Vector nodePos, Vector nodeVel);

    /**
     * \brief Gets best neighbor for perimeter forwarding (right-hand rule)
     * \param previousHop Position of the previous hop
     * \param nodePos Position of the current node
     * \param excludeIp Optional IP to exclude (use instead of position threshold)
     * \return IPv4 address of the best neighbor
     */
    Ipv4Address BestAngle(Vector previousHop, Vector nodePos, Ipv4Address excludeIp = Ipv4Address::GetZero());

    /**
     * \brief Find face toward destination (NS-2 ent_findface equivalent)
     * Returns the GG neighbor with bearing closest to destination bearing
     * \param dstPos Destination position
     * \param nodePos Current node position
     * \return IPv4 address of the face entry neighbor
     */
    Ipv4Address FindFace(Vector dstPos, Vector nodePos);

    /**
     * \brief Get next neighbor counter-clockwise from given neighbor (NS-2 ent_next_ccw)
     * Only considers GG edges in planarized graph
     * \param inNeighbor Current neighbor (incoming edge)
     * \param nodePos Current node position
     * \return Next CCW neighbor on current face
     */
    Ipv4Address NextCCW(Ipv4Address inNeighbor, Vector nodePos);

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
     * \brief Get Top-K neighbor summaries for Hello piggybacking
     * \param k Maximum number of neighbors to return
     * \param selfPos Current node position (for distance-based selection)
     * \return Vector of NeighborSummary structs sorted by link quality
     */
    std::vector<NeighborSummary> GetTopKNeighborSummaries(uint8_t k, Vector selfPos);

    /**
     * \brief Calculate Link Duration (RET) based on relative motion
     * Solves |r + v*t| = R for smallest positive t
     * \param pos1 Position of node 1
     * \param vel1 Velocity of node 1
     * \param pos2 Position of node 2
     * \param vel2 Velocity of node 2
     * \param commRange Communication range R
     * \return Predicted time until link breaks (seconds), or large value if stable
     */
    static double CalculateLinkDuration(Vector pos1, Vector vel1, 
                                        Vector pos2, Vector vel2, 
                                        double commRange);

    /**
     * \brief Calculate two-hop aware score for a neighbor
     * Score = w1*Progress + w2*LinkQuality + w3*LinkDuration
     * \param neighborId The 1-hop neighbor to score
     * \param dstPos Destination position
     * \param selfPos Current node position
     * \param selfVel Current node velocity
     * \return Composite score (higher is better), or -1 if invalid
     */
    double CalculateTwoHopScore(Ipv4Address neighborId,
                                Vector dstPos,
                                Vector selfPos,
                                Vector selfVel);

    /**
     * \brief Neighbor entry structure with position, velocity, SINR, and two-hop data
     *        Extended with Local Digital Twin fields for prediction and quality tracking
     */
    struct NeighborEntry
    {
        // === 基础字段（已有） ===
        Vector position;                   // 1-hop neighbor position
        Vector velocity;                   // 1-hop neighbor velocity vector
        Time lastUpdate;                   // Position update time from HELLO
        double sinr = -1.0;                // Linear SINR. -1.0 = unknown/invalid
        double smoothedSinr = -1.0;        // EWMA-filtered SINR for stability
        Time lastSinrUpdate = Seconds(0);  // Time of last valid SINR update
        std::vector<NeighborSummary> twoHopNeighbors; // 2-hop neighbors via this 1-hop

        // === Local Digital Twin 扩展字段 ===
        double prr = 1.0;                  // Packet Reception Ratio (EWMA), default optimistic
        Time lastPrrUpdate = Seconds(0);   // Time of last PRR update
        Vector predPosition;               // Predicted position at predTime
        Time predTime = Seconds(0);        // Time point for prediction
        double confidence = 1.0;           // Data freshness confidence [0,1]
        
        // === HELLO 序号跟踪字段（LDT PRR）===
        bool hasHelloSeq = false;          // 是否已收到过 HELLO
        uint16_t lastHelloSeq = 0;         // 上次收到的 HELLO 序号
    };

    /// Typedef for table iterator
    typedef std::map<Ipv4Address, NeighborEntry>::iterator TableIterator;
    typedef std::map<Ipv4Address, NeighborEntry>::const_iterator TableConstIterator;

    // ========== Local Digital Twin (LDT) 接口 ==========
    
    /**
     * \brief Update PRR for a neighbor using EWMA
     * \param id Neighbor IPv4 address
     * \param success Whether the packet was successfully received
     */
    void UpdatePrr(Ipv4Address id, bool success);

    /**
     * \brief Update PRR with multiple consecutive misses (closed-form EWMA)
     * Uses: prr = prr * (1-α)^miss, then prr = α + (1-α)*prr for success
     * \param id Neighbor IPv4 address
     * \param misses Number of missed HELLO packets
     */
    void UpdatePrrMisses(Ipv4Address id, uint16_t misses);

    /**
     * \brief Get the last update time for a neighbor entry
     * \param id Neighbor IPv4 address
     * \return Last update time, or Seconds(0) if not found
     */
    Time GetEntryUpdateTime(Ipv4Address id) const;

    /**
     * \brief Update PRR from HELLO sequence number (replaces inline loop in RecvGpsr)
     * Handles: first HELLO, expired entry reset, gap detection with clamping
     * \param id Neighbor IPv4 address
     * \param curSeq Current HELLO sequence number
     * \param expired Whether the entry was considered expired before this HELLO
     */
    void UpdatePrrFromHello(Ipv4Address id, uint16_t curSeq, bool expired);

    /**
     * \brief Predict neighbor position using constant velocity model
     * \param entry The neighbor entry
     * \param now Current time
     * \param predWindow Prediction window (how far into the future)
     * \return Predicted position
     */
    static Vector PredictPosition(const NeighborEntry& entry, Time now, Time predWindow);

    /**
     * \brief Calculate confidence based on time decay
     * conf = exp(-(now - lastUpdate) / tau)
     * \param entry The neighbor entry
     * \param now Current time
     * \param tau Time constant for decay (default 1.5s)
     * \return Confidence value [0, 1]
     */
    static double GetConfidence(const NeighborEntry& entry, Time now, double tau = 1.5);

    /**
     * \brief Get predicted entry state (combines position prediction + confidence)
     * \param id Neighbor IPv4 address
     * \param now Current time
     * \param predWindow Prediction window
     * \param tau Confidence decay constant
     * \param[out] predPos Predicted position
     * \param[out] conf Confidence value
     * \return True if neighbor exists and entry is valid
     */
    bool GetPredictedEntry(Ipv4Address id, Time now, Time predWindow, double tau,
                           Vector& predPos, double& conf);
    
    /**
     * \brief Get iterator to beginning of neighbor table
     */
    TableIterator GetTableBegin() { return m_table.begin(); }
    
    /**
     * \brief Get iterator to end of neighbor table
     */
    TableIterator GetTableEnd() { return m_table.end(); }

    // ========== LDT 参数 setter ==========
    void SetTwinEnabled(bool enabled) { m_twinEnabled = enabled; }
    void SetTwinMinConf(double val) { m_twinMinConf = val; }
    void SetTwinMinPrr(double val) { m_twinMinPrr = val; }
    void SetTwinMinRet(double val) { m_twinMinRet = val; }
    void SetTwinPredWindow(double val) { m_twinPredWindow = val; }
    void SetTwinConfTau(double val) { m_twinConfTau = val; }
    void SetTwinUsePrr(bool val) { m_twinUsePrr = val; }
    Time GetEntryLifeTime() const { return m_entryLifeTime; }

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
    
    // ========== LDT 参数成员 ==========
    bool m_twinEnabled{true};
    double m_twinMinConf{0.3};
    double m_twinMinPrr{0.5};
    double m_twinMinRet{0.5};
    double m_twinPredWindow{0.5};
    double m_twinConfTau{1.5};
    bool m_twinUsePrr{true};
};

} // namespace gpsr
} // namespace ns3

#endif /* GPSR_PTABLE_H */
