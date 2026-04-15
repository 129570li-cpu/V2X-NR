/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "elite-q-table.h"

#include <functional>

namespace ns3
{

namespace
{

inline void
HashCombine(std::size_t& seed, const std::string& value)
{
    seed ^= std::hash<std::string>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

} // namespace

EliteQTable::EliteQTable() = default;

bool
EliteQTable::StateActionKey::operator==(const StateActionKey& other) const
{
    return destination == other.destination && current == other.current && next == other.next;
}

std::size_t
EliteQTable::StateActionKeyHash::operator()(const StateActionKey& key) const
{
    std::size_t seed = 0;
    HashCombine(seed, key.destination);
    HashCombine(seed, key.current);
    HashCombine(seed, key.next);
    return seed;
}

void
EliteQTable::Clear()
{
    m_qValues.clear();
}

bool
EliteQTable::HasEntry(const std::string& destination,
                      const std::string& current,
                      const std::string& next) const
{
    return m_qValues.find({destination, current, next}) != m_qValues.end();
}

double
EliteQTable::GetQ(const std::string& destination,
                  const std::string& current,
                  const std::string& next) const
{
    auto it = m_qValues.find({destination, current, next});
    if (it == m_qValues.end())
    {
        return 0.0;
    }

    return it->second;
}

void
EliteQTable::SetQ(const std::string& destination,
                  const std::string& current,
                  const std::string& next,
                  double value)
{
    m_qValues[{destination, current, next}] = value;
}

double
EliteQTable::GetMaxQ(const std::string& destination,
                     const std::string& current,
                     const std::vector<std::string>& candidates) const
{
    double maxValue = 0.0;
    bool hasCandidate = false;

    for (const auto& next : candidates)
    {
        const double qValue = GetQ(destination, current, next);
        if (!hasCandidate || qValue > maxValue)
        {
            maxValue = qValue;
            hasCandidate = true;
        }
    }

    return hasCandidate ? maxValue : 0.0;
}

std::string
EliteQTable::GetBestAction(const std::string& destination,
                           const std::string& current,
                           const std::vector<std::string>& candidates) const
{
    std::string bestAction;
    double bestValue = 0.0;
    bool hasCandidate = false;

    for (const auto& next : candidates)
    {
        const double qValue = GetQ(destination, current, next);
        if (!hasCandidate || qValue > bestValue)
        {
            bestAction = next;
            bestValue = qValue;
            hasCandidate = true;
        }
    }

    return bestAction;
}

} // namespace ns3
