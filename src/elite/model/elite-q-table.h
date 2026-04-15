/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef ELITE_Q_TABLE_H
#define ELITE_Q_TABLE_H

#include <string>
#include <unordered_map>
#include <vector>

namespace ns3
{

/**
 * \ingroup elite
 * \brief Tabular Q-value store for junction-level ELITE training.
 */
class EliteQTable
{
  public:
    EliteQTable();

    /**
     * \brief Remove all learned Q-values.
     */
    void Clear();

    /**
     * \brief Check whether a Q entry exists.
     * \param destination destination junction
     * \param current current junction
     * \param next next-hop junction
     * \return true if the entry exists
     */
    bool HasEntry(const std::string& destination,
                  const std::string& current,
                  const std::string& next) const;

    /**
     * \brief Get a Q value.
     * \param destination destination junction
     * \param current current junction
     * \param next next-hop junction
     * \return stored Q value or 0 if absent
     */
    double GetQ(const std::string& destination,
                const std::string& current,
                const std::string& next) const;

    /**
     * \brief Set a Q value.
     * \param destination destination junction
     * \param current current junction
     * \param next next-hop junction
     * \param value Q value
     */
    void SetQ(const std::string& destination,
              const std::string& current,
              const std::string& next,
              double value);

    /**
     * \brief Get the maximum Q value among candidate actions.
     * \param destination destination junction
     * \param current current junction
     * \param candidates candidate next-hop junctions
     * \return maximum Q value or 0 if no candidate exists
     */
    double GetMaxQ(const std::string& destination,
                   const std::string& current,
                   const std::vector<std::string>& candidates) const;

    /**
     * \brief Get the best action among candidates according to the table.
     * \param destination destination junction
     * \param current current junction
     * \param candidates candidate next-hop junctions
     * \return best candidate or empty string if none exists
     */
    std::string GetBestAction(const std::string& destination,
                              const std::string& current,
                              const std::vector<std::string>& candidates) const;

  private:
    struct StateActionKey
    {
        std::string destination;
        std::string current;
        std::string next;

        bool operator==(const StateActionKey& other) const;
    };

    struct StateActionKeyHash
    {
        std::size_t operator()(const StateActionKey& key) const;
    };

    std::unordered_map<StateActionKey, double, StateActionKeyHash> m_qValues;
};

} // namespace ns3

#endif /* ELITE_Q_TABLE_H */
