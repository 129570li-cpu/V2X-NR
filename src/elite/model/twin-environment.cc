/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "twin-environment.h"

#include "sumo-road-graph-loader.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TwinEnvironment");

NS_OBJECT_ENSURE_REGISTERED(TwinEnvironment);

TypeId
TwinEnvironment::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TwinEnvironment")
            .SetParent<Object>()
            .SetGroupName("Elite")
            .AddConstructor<TwinEnvironment>();
    return tid;
}

TwinEnvironment::TwinEnvironment()
    : m_predictionHorizon(Seconds(5)),
      m_snapshotInterval(MilliSeconds(100))
{
    NS_LOG_FUNCTION(this);
}

TwinEnvironment::~TwinEnvironment()
{
    NS_LOG_FUNCTION(this);
}

bool
TwinEnvironment::InitializeRoadGraphFromSumoNetXml(const std::string& fileName)
{
    NS_LOG_FUNCTION(this << fileName);
    return SumoRoadGraphLoader::LoadNetXml(fileName, *this);
}

void
TwinEnvironment::UpdateVehicleState(const TwinVehicleState& state)
{
    NS_LOG_FUNCTION(this << state.vehicleId);
    m_vehicleStates[state.vehicleId] = state;
}

bool
TwinEnvironment::RemoveVehicleState(uint64_t vehicleId)
{
    NS_LOG_FUNCTION(this << vehicleId);
    return m_vehicleStates.erase(vehicleId) > 0;
}

void
TwinEnvironment::Clear()
{
    NS_LOG_FUNCTION(this);
    m_vehicleStates.clear();
    m_junctions.clear();
    m_roads.clear();
    m_adjacency.clear();
    m_futureSnapshots.clear();
}

void
TwinEnvironment::ClearFutureSnapshots()
{
    NS_LOG_FUNCTION(this);
    m_futureSnapshots.clear();
}

void
TwinEnvironment::ClearRoadGraph()
{
    NS_LOG_FUNCTION(this);
    m_junctions.clear();
    m_roads.clear();
    m_adjacency.clear();
}

bool
TwinEnvironment::HasVehicleState(uint64_t vehicleId) const
{
    return m_vehicleStates.find(vehicleId) != m_vehicleStates.end();
}

const TwinVehicleState*
TwinEnvironment::GetVehicleState(uint64_t vehicleId) const
{
    auto it = m_vehicleStates.find(vehicleId);
    if (it == m_vehicleStates.end())
    {
        return nullptr;
    }

    return &it->second;
}

const std::unordered_map<uint64_t, TwinVehicleState>&
TwinEnvironment::GetVehicleStates() const
{
    return m_vehicleStates;
}

void
TwinEnvironment::AddJunction(const TwinJunction& junction)
{
    NS_LOG_FUNCTION(this << junction.junctionId);
    m_junctions[junction.junctionId] = junction;
    (void)m_adjacency[junction.junctionId];
}

void
TwinEnvironment::AddRoadSegment(const TwinRoadSegment& road)
{
    NS_LOG_FUNCTION(this << road.roadId);
    m_roads[road.roadId] = road;

    auto& adjacency = m_adjacency[road.fromJunctionId];
    if (std::find(adjacency.begin(), adjacency.end(), road.toJunctionId) == adjacency.end())
    {
        adjacency.push_back(road.toJunctionId);
    }

    (void)m_adjacency[road.toJunctionId];
}

bool
TwinEnvironment::HasJunction(const std::string& junctionId) const
{
    return m_junctions.find(junctionId) != m_junctions.end();
}

bool
TwinEnvironment::HasRoadSegment(const std::string& roadId) const
{
    return m_roads.find(roadId) != m_roads.end();
}

const TwinJunction*
TwinEnvironment::GetJunction(const std::string& junctionId) const
{
    auto it = m_junctions.find(junctionId);
    if (it == m_junctions.end())
    {
        return nullptr;
    }

    return &it->second;
}

const TwinRoadSegment*
TwinEnvironment::GetRoadSegment(const std::string& roadId) const
{
    auto it = m_roads.find(roadId);
    if (it == m_roads.end())
    {
        return nullptr;
    }

    return &it->second;
}

const TwinRoadSegment*
TwinEnvironment::GetConnectingRoadSegment(const std::string& fromJunctionId,
                                          const std::string& toJunctionId) const
{
    for (const auto& [roadId, road] : m_roads)
    {
        (void)roadId;
        if (road.fromJunctionId == fromJunctionId && road.toJunctionId == toJunctionId)
        {
            return &road;
        }
    }

    return nullptr;
}

const std::unordered_map<std::string, TwinJunction>&
TwinEnvironment::GetJunctions() const
{
    return m_junctions;
}

const std::unordered_map<std::string, TwinRoadSegment>&
TwinEnvironment::GetRoadSegments() const
{
    return m_roads;
}

const std::vector<std::string>*
TwinEnvironment::GetAdjacentJunctions(const std::string& junctionId) const
{
    auto it = m_adjacency.find(junctionId);
    if (it == m_adjacency.end())
    {
        return nullptr;
    }

    return &it->second;
}

const std::unordered_map<std::string, std::vector<std::string>>&
TwinEnvironment::GetAdjacency() const
{
    return m_adjacency;
}

uint32_t
TwinEnvironment::CountVehiclesOnRoad(const std::string& roadId, Time offset) const
{
    uint32_t count = 0;

    if (offset.IsStrictlyPositive())
    {
        const auto* snapshot = GetFutureSnapshot(offset);
        if (snapshot == nullptr)
        {
            return 0;
        }

        for (const auto& vehicle : snapshot->vehicles)
        {
            if (vehicle.roadId == roadId)
            {
                count++;
            }
        }
        return count;
    }

    for (const auto& [vehicleId, vehicle] : m_vehicleStates)
    {
        (void)vehicleId;
        if (vehicle.roadId == roadId)
        {
            count++;
        }
    }

    return count;
}

TwinSnapshot
TwinEnvironment::GetCurrentSnapshot() const
{
    TwinSnapshot snapshot;
    snapshot.timestamp = Simulator::Now();
    snapshot.vehicles.reserve(m_vehicleStates.size());

    for (const auto& [vehicleId, state] : m_vehicleStates)
    {
        (void)vehicleId;
        snapshot.vehicles.push_back(state);
    }

    return snapshot;
}

void
TwinEnvironment::AddFutureSnapshot(Time offset, const TwinSnapshot& snapshot)
{
    NS_LOG_FUNCTION(this << offset.GetSeconds());
    if (offset.IsNegative())
    {
        NS_LOG_WARN("Ignoring negative twin snapshot offset");
        return;
    }

    if (offset > m_predictionHorizon)
    {
        NS_LOG_WARN("Ignoring twin snapshot beyond prediction horizon");
        return;
    }

    m_futureSnapshots[offset] = snapshot;
}

bool
TwinEnvironment::HasFutureSnapshot(Time offset) const
{
    return m_futureSnapshots.find(offset) != m_futureSnapshots.end();
}

const TwinSnapshot*
TwinEnvironment::GetFutureSnapshot(Time offset) const
{
    auto it = m_futureSnapshots.find(offset);
    if (it == m_futureSnapshots.end())
    {
        return nullptr;
    }

    return &it->second;
}

const std::map<Time, TwinSnapshot>&
TwinEnvironment::GetFutureSnapshots() const
{
    return m_futureSnapshots;
}

void
TwinEnvironment::SetPredictionHorizon(Time horizon)
{
    NS_LOG_FUNCTION(this << horizon.GetSeconds());
    m_predictionHorizon = horizon;
}

Time
TwinEnvironment::GetPredictionHorizon() const
{
    return m_predictionHorizon;
}

void
TwinEnvironment::SetSnapshotInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval.GetSeconds());
    m_snapshotInterval = interval;
}

Time
TwinEnvironment::GetSnapshotInterval() const
{
    return m_snapshotInterval;
}

} // namespace ns3
