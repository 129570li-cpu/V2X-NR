/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "elite-trainer.h"

#include "elite-q-table.h"
#include "elite-reward-model.h"
#include "twin-environment.h"

#include "ns3/log.h"
#include "ns3/random-variable-stream.h"

#include <algorithm>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("EliteTrainer");

EliteTrainer::EliteTrainer()
    : m_environment(nullptr),
      m_qTable(nullptr),
      m_rewardModel(nullptr),
      m_learningRate(0.9),
      m_discountFactor(0.1),
      m_exploitationProbability(0.5),
      m_greedyProbability(0.2),
      m_explorationProbability(0.3),
      m_maxSteps(64),
      m_uniformRv(CreateObject<UniformRandomVariable>())
{
}

void
EliteTrainer::SetEnvironment(TwinEnvironment* environment)
{
    m_environment = environment;
}

void
EliteTrainer::SetQTable(EliteQTable* qTable)
{
    m_qTable = qTable;
}

void
EliteTrainer::SetRewardModel(EliteRewardModel* rewardModel)
{
    m_rewardModel = rewardModel;
}

void
EliteTrainer::SetLearningRate(double learningRate)
{
    m_learningRate = learningRate;
}

double
EliteTrainer::GetLearningRate() const
{
    return m_learningRate;
}

void
EliteTrainer::SetDiscountFactor(double discountFactor)
{
    m_discountFactor = discountFactor;
}

double
EliteTrainer::GetDiscountFactor() const
{
    return m_discountFactor;
}

void
EliteTrainer::SetMaxSteps(uint32_t maxSteps)
{
    m_maxSteps = maxSteps;
}

uint32_t
EliteTrainer::GetMaxSteps() const
{
    return m_maxSteps;
}

void
EliteTrainer::SetSelectionProbabilities(double exploitationProbability,
                                        double greedyProbability,
                                        double explorationProbability)
{
    m_exploitationProbability = exploitationProbability;
    m_greedyProbability = greedyProbability;
    m_explorationProbability = explorationProbability;
}

bool
EliteTrainer::IsReady() const
{
    return m_environment != nullptr && m_qTable != nullptr && m_rewardModel != nullptr;
}

std::vector<std::string>
EliteTrainer::TrainEpisode(const std::string& sourceJunction,
                           const std::string& destinationJunction,
                           Time offset)
{
    std::vector<std::string> path;
    if (!IsReady())
    {
        NS_LOG_WARN("EliteTrainer is not ready");
        return path;
    }

    if (!m_environment->HasJunction(sourceJunction) || !m_environment->HasJunction(destinationJunction))
    {
        NS_LOG_WARN("Source or destination junction is missing from the twin");
        return path;
    }

    std::string current = sourceJunction;
    path.push_back(current);

    for (uint32_t step = 0; step < m_maxSteps && current != destinationJunction; ++step)
    {
        const std::string next = SelectAction(current, destinationJunction);
        if (next.empty())
        {
            break;
        }

        if (std::find(path.begin(), path.end(), next) != path.end())
        {
            path.push_back(next);
            break;
        }

        path.push_back(next);
        current = next;
    }

    if (!path.empty() && path.back() == destinationJunction)
    {
        UpdateEpisode(path, destinationJunction, offset);
    }

    return path;
}

std::vector<std::string>
EliteTrainer::GetCandidateActions(const std::string& currentJunction) const
{
    if (m_environment == nullptr)
    {
        return {};
    }

    const auto* adjacency = m_environment->GetAdjacentJunctions(currentJunction);
    if (adjacency == nullptr)
    {
        return {};
    }

    return *adjacency;
}

EliteTrainer::ActionMode
EliteTrainer::SelectMode() const
{
    const double total = m_exploitationProbability + m_greedyProbability + m_explorationProbability;
    if (total <= 0.0)
    {
        return ActionMode::EXPLORE;
    }

    const double draw = m_uniformRv->GetValue(0.0, total);
    if (draw < m_exploitationProbability)
    {
        return ActionMode::EXPLOIT;
    }
    if (draw < m_exploitationProbability + m_greedyProbability)
    {
        return ActionMode::GREEDY;
    }

    return ActionMode::EXPLORE;
}

std::string
EliteTrainer::SelectAction(const std::string& currentJunction,
                           const std::string& destinationJunction) const
{
    const auto candidates = GetCandidateActions(currentJunction);
    if (candidates.empty())
    {
        return "";
    }

    if (std::find(candidates.begin(), candidates.end(), destinationJunction) != candidates.end())
    {
        return destinationJunction;
    }

    switch (SelectMode())
    {
    case ActionMode::EXPLOIT:
    {
        const std::string bestAction =
            m_qTable->GetBestAction(destinationJunction, currentJunction, candidates);
        if (!bestAction.empty())
        {
            return bestAction;
        }
        [[fallthrough]];
    }
    case ActionMode::GREEDY:
    {
        const std::string greedyAction =
            SelectGreedyDistanceAction(currentJunction, destinationJunction, candidates);
        if (!greedyAction.empty())
        {
            return greedyAction;
        }
        [[fallthrough]];
    }
    case ActionMode::EXPLORE:
        return SelectRandomAction(candidates);
    }

    return "";
}

std::string
EliteTrainer::SelectGreedyDistanceAction(const std::string& currentJunction,
                                         const std::string& destinationJunction,
                                         const std::vector<std::string>& candidates) const
{
    (void)currentJunction;

    if (m_environment == nullptr)
    {
        return "";
    }

    const auto* destination = m_environment->GetJunction(destinationJunction);
    if (destination == nullptr)
    {
        return "";
    }

    std::string bestAction;
    double bestDistance = std::numeric_limits<double>::max();

    for (const auto& candidate : candidates)
    {
        const auto* junction = m_environment->GetJunction(candidate);
        if (junction == nullptr)
        {
            continue;
        }

        const double dx = junction->position.x - destination->position.x;
        const double dy = junction->position.y - destination->position.y;
        const double distance = dx * dx + dy * dy;

        if (distance < bestDistance)
        {
            bestDistance = distance;
            bestAction = candidate;
        }
    }

    return bestAction;
}

std::string
EliteTrainer::SelectRandomAction(const std::vector<std::string>& candidates) const
{
    if (candidates.empty())
    {
        return "";
    }

    const uint32_t index = m_uniformRv->GetInteger(0, candidates.size() - 1);
    return candidates[index];
}

void
EliteTrainer::UpdateEpisode(const std::vector<std::string>& path,
                            const std::string& destinationJunction,
                            Time offset)
{
    if (path.size() < 2)
    {
        return;
    }

    for (int32_t i = static_cast<int32_t>(path.size()) - 2; i >= 0; --i)
    {
        const std::string& current = path[i];
        const std::string& next = path[i + 1];
        const auto nextCandidates = GetCandidateActions(next);

        const double reward =
            m_rewardModel->ComputeReward(*m_environment, current, next, destinationJunction, offset);
        const double maxNextQ = m_qTable->GetMaxQ(destinationJunction, next, nextCandidates);
        const double currentQ = m_qTable->GetQ(destinationJunction, current, next);
        const double updatedQ =
            currentQ + m_learningRate * (reward + m_discountFactor * maxNextQ - currentQ);

        m_qTable->SetQ(destinationJunction, current, next, updatedQ);
    }
}

} // namespace ns3
