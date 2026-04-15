/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "elite-reward-model.h"

#include "twin-environment.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

namespace
{

constexpr double kMinPositive = 1e-6;

} // namespace

EliteRewardModel::EliteRewardModel()
    : m_objective(EliteTrainingObjective::HOP_COUNT),
      m_communicationRangeMeters(300.0)
{
}

void
EliteRewardModel::SetObjective(EliteTrainingObjective objective)
{
    m_objective = objective;
}

EliteTrainingObjective
EliteRewardModel::GetObjective() const
{
    return m_objective;
}

void
EliteRewardModel::SetCommunicationRange(double communicationRangeMeters)
{
    m_communicationRangeMeters = communicationRangeMeters;
}

double
EliteRewardModel::GetCommunicationRange() const
{
    return m_communicationRangeMeters;
}

double
EliteRewardModel::ComputeReward(const TwinEnvironment& environment,
                                const std::string& currentJunction,
                                const std::string& nextJunction,
                                const std::string& destinationJunction,
                                Time offset) const
{
    (void)destinationJunction;

    switch (m_objective)
    {
    case EliteTrainingObjective::PDR:
        return ComputePdrReward(environment, currentJunction, nextJunction, offset);
    case EliteTrainingObjective::DELAY:
        return ComputeDelayReward(environment, currentJunction, nextJunction);
    case EliteTrainingObjective::HOP_COUNT:
        return ComputeHopCountReward(environment, currentJunction, nextJunction);
    case EliteTrainingObjective::ROUTING_COST:
        return ComputeRoutingCostReward(environment, currentJunction, nextJunction, offset);
    }

    return 0.0;
}

double
EliteRewardModel::ComputePdrReward(const TwinEnvironment& environment,
                                   const std::string& currentJunction,
                                   const std::string& nextJunction,
                                   Time offset) const
{
    const auto* road = environment.GetConnectingRoadSegment(currentJunction, nextJunction);
    if (road == nullptr)
    {
        return 0.0;
    }

    const double laneCount = std::max(1u, road->laneCount);
    const double vehicleCount = environment.CountVehiclesOnRoad(road->roadId, offset);
    const double densityScore = std::min(1.0, vehicleCount / (laneCount * 5.0));

    return densityScore;
}

double
EliteRewardModel::ComputeDelayReward(const TwinEnvironment& environment,
                                     const std::string& currentJunction,
                                     const std::string& nextJunction) const
{
    const auto* road = environment.GetConnectingRoadSegment(currentJunction, nextJunction);
    if (road == nullptr)
    {
        return 0.0;
    }

    const double speed = std::max(kMinPositive, road->maxSpeedMetersPerSecond);
    const double travelTime = road->lengthMeters / speed;
    return 1.0 / (1.0 + travelTime);
}

double
EliteRewardModel::ComputeHopCountReward(const TwinEnvironment& environment,
                                        const std::string& currentJunction,
                                        const std::string& nextJunction) const
{
    const auto* road = environment.GetConnectingRoadSegment(currentJunction, nextJunction);
    if (road == nullptr)
    {
        return 0.0;
    }

    const double normalized = road->lengthMeters / std::max(kMinPositive, m_communicationRangeMeters);
    return std::exp(-normalized);
}

double
EliteRewardModel::ComputeRoutingCostReward(const TwinEnvironment& environment,
                                           const std::string& currentJunction,
                                           const std::string& nextJunction,
                                           Time offset) const
{
    const auto* road = environment.GetConnectingRoadSegment(currentJunction, nextJunction);
    if (road == nullptr)
    {
        return 0.0;
    }

    const double expectedRelayCount =
        std::max(1.0, std::ceil(road->lengthMeters / std::max(kMinPositive, m_communicationRangeMeters)));
    const double vehicleCount = environment.CountVehiclesOnRoad(road->roadId, offset);
    const double controlCost = expectedRelayCount + 0.1 * vehicleCount;

    return 1.0 / (1.0 + controlCost);
}

} // namespace ns3
