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

    // 以每车道5辆为满载基准，归一化到 [0, 1]
    const double laneCount = std::max(1u, road->laneCount);
    const double vehicleCount = environment.CountVehiclesOnRoad(road->roadId, offset);
    const double densityScore = std::min(1.0, vehicleCount / (laneCount * 5.0));

    // 使用倒U型高斯函数：密度在0.5时奖励最高（连通性好但不过载）
    // reward = exp(-4 * (d - 0.5)^2)，峰值约为1.0，在d=0和d=1时约为0.018
    const double shifted = densityScore - 0.5;
    return std::exp(-4.0 * shifted * shifted);
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

    // 估算路段实际平均速度：
    // 统计在该路段上所有车辆的速度模长，取均值；若无车，则退回限速。
    double sumSpeed = 0.0;
    uint32_t count = 0;
    for (const auto& kv : environment.GetVehicleStates())
    {
        if (kv.second.roadId == road->roadId)
        {
            const auto& v = kv.second.velocity;
            sumSpeed += std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            ++count;
        }
    }
    const double actualSpeed = (count > 0)
                                   ? std::max(kMinPositive, sumSpeed / count)
                                   : std::max(kMinPositive, road->maxSpeedMetersPerSecond);

    const double travelTime = road->lengthMeters / actualSpeed;
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

    // 预计中继跳数（路段长度 / 通信半径，向上取整，至少为1）
    const double expectedRelayCount =
        std::max(1.0, std::ceil(road->lengthMeters / std::max(kMinPositive, m_communicationRangeMeters)));

    // 将车辆数归一化为车辆密度（辆/百米），避免与跳数直接相加产生量纲混乱
    const double vehicleCount = environment.CountVehiclesOnRoad(road->roadId, offset);
    const double densityPer100m =
        vehicleCount / std::max(kMinPositive, road->lengthMeters / 100.0);

    // 控制开销 = 跳数 × (1 + 密度惩罚因子)，密度越高控制包越多
    const double controlCost = expectedRelayCount * (1.0 + 0.1 * densityPer100m);

    return 1.0 / (1.0 + controlCost);
}

} // namespace ns3
