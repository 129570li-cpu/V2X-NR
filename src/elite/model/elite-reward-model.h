/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef ELITE_REWARD_MODEL_H
#define ELITE_REWARD_MODEL_H

#include "ns3/nstime.h"

#include <string>

namespace ns3
{

class TwinEnvironment;

/**
 * \ingroup elite
 * \brief Supported single-target training objectives in ELITE.
 */
enum class EliteTrainingObjective
{
    PDR,
    DELAY,
    HOP_COUNT,
    ROUTING_COST
};

/**
 * \ingroup elite
 * \brief Reward calculator for ELITE single-target training.
 */
class EliteRewardModel
{
  public:
    EliteRewardModel();

    void SetObjective(EliteTrainingObjective objective);
    EliteTrainingObjective GetObjective() const;

    void SetCommunicationRange(double communicationRangeMeters);
    double GetCommunicationRange() const;

    /**
     * \brief Compute the reward for traversing a junction-to-junction action.
     * \param environment twin environment providing graph and mirrored state
     * \param currentJunction current junction
     * \param nextJunction chosen next junction
     * \param destinationJunction target junction
     * \param offset optional future snapshot offset
     * \return reward value
     */
    double ComputeReward(const TwinEnvironment& environment,
                         const std::string& currentJunction,
                         const std::string& nextJunction,
                         const std::string& destinationJunction,
                         Time offset = Seconds(0)) const;

  private:
    double ComputePdrReward(const TwinEnvironment& environment,
                            const std::string& currentJunction,
                            const std::string& nextJunction,
                            Time offset) const;
    double ComputeDelayReward(const TwinEnvironment& environment,
                              const std::string& currentJunction,
                              const std::string& nextJunction) const;
    double ComputeHopCountReward(const TwinEnvironment& environment,
                                 const std::string& currentJunction,
                                 const std::string& nextJunction) const;
    double ComputeRoutingCostReward(const TwinEnvironment& environment,
                                    const std::string& currentJunction,
                                    const std::string& nextJunction,
                                    Time offset) const;

    EliteTrainingObjective m_objective;
    double m_communicationRangeMeters;
};

} // namespace ns3

#endif /* ELITE_REWARD_MODEL_H */
