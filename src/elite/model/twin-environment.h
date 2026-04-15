/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef TWIN_ENVIRONMENT_H
#define TWIN_ENVIRONMENT_H

#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/vector.h"

#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace ns3
{

/**
 * \ingroup elite
 * \brief Cached state of a vehicle mirrored in the digital twin.
 */
struct TwinVehicleState
{
    uint64_t vehicleId{0};
    Vector position{0.0, 0.0, 0.0};
    Vector velocity{0.0, 0.0, 0.0};
    double headingDeg{0.0};
    std::string roadId;
    std::string laneId;
    std::string junctionId;
    double bufferOccupancyBytes{0.0};
    Time lastUpdate{Seconds(0)};
};

/**
 * \ingroup elite
 * \brief Static junction information mirrored from the SUMO road graph.
 */
struct TwinJunction
{
    std::string junctionId;
    Vector position{0.0, 0.0, 0.0};
    std::string type;
};

/**
 * \ingroup elite
 * \brief Static road segment information mirrored from the SUMO road graph.
 */
struct TwinRoadSegment
{
    std::string roadId;
    std::string fromJunctionId;
    std::string toJunctionId;
    double lengthMeters{0.0};
    double maxSpeedMetersPerSecond{0.0};
    uint32_t laneCount{0};
};

/**
 * \ingroup elite
 * \brief A mirrored network snapshot at a given simulated time.
 */
struct TwinSnapshot
{
    Time timestamp{Seconds(0)};
    std::vector<TwinVehicleState> vehicles;
};

/**
 * \ingroup elite
 * \brief Minimal digital twin state manager for ELITE experiments.
 *
 * This class keeps the current mirrored vehicle state and an indexed set of
 * future snapshots. The first version is intentionally limited to state
 * storage and retrieval, so the controller and policy logic can be layered
 * on top later.
 */
class TwinEnvironment : public Object
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    TwinEnvironment();
    ~TwinEnvironment() override;

    /**
     * \brief Initialize the mirrored road graph from a SUMO net.xml file.
     * \param fileName path to the SUMO net.xml file
     * \return true on success
     */
    bool InitializeRoadGraphFromSumoNetXml(const std::string& fileName);

    /**
     * \brief Replace or insert the current mirrored state of a vehicle.
     * \param state the state to cache
     */
    void UpdateVehicleState(const TwinVehicleState& state);

    /**
     * \brief Remove a mirrored vehicle state.
     * \param vehicleId the vehicle identifier
     * \return true if the state existed and was removed
     */
    bool RemoveVehicleState(uint64_t vehicleId);

    /**
     * \brief Remove all current and future mirrored states.
     */
    void Clear();

    /**
     * \brief Clear only the stored future snapshots.
     */
    void ClearFutureSnapshots();

    /**
     * \brief Remove all mirrored road graph state.
     */
    void ClearRoadGraph();

    /**
     * \brief Check whether a vehicle state is cached in the current snapshot.
     * \param vehicleId the vehicle identifier
     * \return true if a state exists
     */
    bool HasVehicleState(uint64_t vehicleId) const;

    /**
     * \brief Get a pointer to the current mirrored state of a vehicle.
     * \param vehicleId the vehicle identifier
     * \return pointer to the state or nullptr if absent
     */
    const TwinVehicleState* GetVehicleState(uint64_t vehicleId) const;

    /**
     * \brief Get all current mirrored vehicle states.
     * \return const reference to the internal state table
     */
    const std::unordered_map<uint64_t, TwinVehicleState>& GetVehicleStates() const;

    /**
     * \brief Add or replace a junction in the mirrored road graph.
     * \param junction the junction description
     */
    void AddJunction(const TwinJunction& junction);

    /**
     * \brief Add or replace a road segment in the mirrored road graph.
     * \param road the road segment description
     */
    void AddRoadSegment(const TwinRoadSegment& road);

    /**
     * \brief Check whether a mirrored junction exists.
     * \param junctionId the junction identifier
     * \return true if present
     */
    bool HasJunction(const std::string& junctionId) const;

    /**
     * \brief Check whether a mirrored road segment exists.
     * \param roadId the road identifier
     * \return true if present
     */
    bool HasRoadSegment(const std::string& roadId) const;

    /**
     * \brief Get a mirrored junction.
     * \param junctionId the junction identifier
     * \return pointer to the junction or nullptr if absent
     */
    const TwinJunction* GetJunction(const std::string& junctionId) const;

    /**
     * \brief Get a mirrored road segment.
     * \param roadId the road identifier
     * \return pointer to the road segment or nullptr if absent
     */
    const TwinRoadSegment* GetRoadSegment(const std::string& roadId) const;

    /**
     * \brief Find the road segment connecting two adjacent junctions.
     * \param fromJunctionId source junction identifier
     * \param toJunctionId destination junction identifier
     * \return pointer to the matching road segment or nullptr if absent
     */
    const TwinRoadSegment* GetConnectingRoadSegment(const std::string& fromJunctionId,
                                                    const std::string& toJunctionId) const;

    /**
     * \brief Get the full mirrored junction table.
     * \return const reference to junctions
     */
    const std::unordered_map<std::string, TwinJunction>& GetJunctions() const;

    /**
     * \brief Get the full mirrored road segment table.
     * \return const reference to road segments
     */
    const std::unordered_map<std::string, TwinRoadSegment>& GetRoadSegments() const;

    /**
     * \brief Get the outgoing adjacent junctions of a junction.
     * \param junctionId the source junction identifier
     * \return pointer to adjacency vector or nullptr if absent
     */
    const std::vector<std::string>* GetAdjacentJunctions(const std::string& junctionId) const;

    /**
     * \brief Get the adjacency table for mirrored junction graph.
     * \return const reference to adjacency table
     */
    const std::unordered_map<std::string, std::vector<std::string>>& GetAdjacency() const;

    /**
     * \brief Count vehicles currently mirrored on a road segment.
     * \param roadId road identifier
     * \param offset optional future snapshot offset
     * \return number of mirrored vehicles on the road
     */
    uint32_t CountVehiclesOnRoad(const std::string& roadId,
                                 Time offset = Seconds(0)) const;

    /**
     * \brief Get the current mirrored snapshot assembled from cached states.
     * \return the snapshot value
     */
    TwinSnapshot GetCurrentSnapshot() const;

    /**
     * \brief Add or replace a future snapshot keyed by its offset from now.
     * \param offset relative time offset
     * \param snapshot future snapshot data
     */
    void AddFutureSnapshot(Time offset, const TwinSnapshot& snapshot);

    /**
     * \brief Check whether a future snapshot exists at the given offset.
     * \param offset relative time offset
     * \return true if present
     */
    bool HasFutureSnapshot(Time offset) const;

    /**
     * \brief Get a pointer to a future snapshot.
     * \param offset relative time offset
     * \return pointer to the snapshot or nullptr if absent
     */
    const TwinSnapshot* GetFutureSnapshot(Time offset) const;

    /**
     * \brief Get all future snapshots.
     * \return const reference to the snapshot table
     */
    const std::map<Time, TwinSnapshot>& GetFutureSnapshots() const;

    /**
     * \brief Set the maximum prediction horizon maintained by the twin.
     * \param horizon the prediction horizon
     */
    void SetPredictionHorizon(Time horizon);

    /**
     * \brief Get the current prediction horizon.
     * \return the prediction horizon
     */
    Time GetPredictionHorizon() const;

    /**
     * \brief Set the sampling interval for future snapshots.
     * \param interval the snapshot interval
     */
    void SetSnapshotInterval(Time interval);

    /**
     * \brief Get the sampling interval for future snapshots.
     * \return the snapshot interval
     */
    Time GetSnapshotInterval() const;

  private:
    std::unordered_map<uint64_t, TwinVehicleState> m_vehicleStates;
    std::unordered_map<std::string, TwinJunction> m_junctions;
    std::unordered_map<std::string, TwinRoadSegment> m_roads;
    std::unordered_map<std::string, std::vector<std::string>> m_adjacency;
    std::map<Time, TwinSnapshot> m_futureSnapshots;
    Time m_predictionHorizon;
    Time m_snapshotInterval;
};

} // namespace ns3

#endif /* TWIN_ENVIRONMENT_H */
