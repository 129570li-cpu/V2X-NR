/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef ELITE_TRAINER_H
#define ELITE_TRAINER_H

#include "ns3/nstime.h"
#include "ns3/ptr.h"

#include <string>
#include <vector>

namespace ns3
{

class EliteQTable;
class EliteRewardModel;
class TwinEnvironment;
class UniformRandomVariable;

/**
 * \ingroup elite
 * \brief Minimal tabular trainer for ELITE single-target policies.
 */
class EliteTrainer
{
  public:
    EliteTrainer();

    void SetEnvironment(TwinEnvironment* environment);
    void SetQTable(EliteQTable* qTable);
    void SetRewardModel(EliteRewardModel* rewardModel);

    void SetLearningRate(double learningRate);
    double GetLearningRate() const;

    void SetDiscountFactor(double discountFactor);
    double GetDiscountFactor() const;

    void SetMaxSteps(uint32_t maxSteps);
    uint32_t GetMaxSteps() const;

    /**
     * \brief Configure the three action selection probabilities.
     * \param exploitationProbability probability of exploiting the Q-table
     * \param greedyProbability probability of taking the geometric greedy action
     * \param explorationProbability probability of random exploration
     */
    void SetSelectionProbabilities(double exploitationProbability,
                                   double greedyProbability,
                                   double explorationProbability);

    /**
     * \brief Check whether the trainer has all required dependencies.
     * \return true if environment, Q table and reward model are attached
     */
    bool IsReady() const;

    /**
     * \brief Execute and train on a single episode.
     * \param sourceJunction episode source junction
     * \param destinationJunction episode destination junction
     * \param offset optional future snapshot offset
     * \return traversed junction sequence
     */
    std::vector<std::string> TrainEpisode(const std::string& sourceJunction,
                                          const std::string& destinationJunction,
                                          Time offset = Seconds(0));

  private:
    enum class ActionMode
    {
        EXPLOIT,
        GREEDY,
        EXPLORE
    };

    std::vector<std::string> GetCandidateActions(const std::string& currentJunction) const;
    ActionMode SelectMode() const;
    std::string SelectAction(const std::string& currentJunction,
                             const std::string& destinationJunction) const;
    std::string SelectGreedyDistanceAction(const std::string& currentJunction,
                                           const std::string& destinationJunction,
                                           const std::vector<std::string>& candidates) const;
    std::string SelectRandomAction(const std::vector<std::string>& candidates) const;
    void UpdateEpisode(const std::vector<std::string>& path,
                       const std::string& destinationJunction,
                       Time offset);

    TwinEnvironment* m_environment;
    EliteQTable* m_qTable;
    EliteRewardModel* m_rewardModel;
    double m_learningRate;
    double m_discountFactor;
    double m_exploitationProbability;
    double m_greedyProbability;
    double m_explorationProbability;
    uint32_t m_maxSteps;
    Ptr<UniformRandomVariable> m_uniformRv;
};

} // namespace ns3

#endif /* ELITE_TRAINER_H */
