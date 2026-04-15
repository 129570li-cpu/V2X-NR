/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef ELITE_CONTROLLER_H
#define ELITE_CONTROLLER_H

#include "elite-q-table.h"
#include "elite-reward-model.h"
#include "elite-trainer.h"
#include "twin-environment.h"

#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ns3
{

/**
 * \ingroup elite
 * \brief Message priority classes used for policy fusion.
 *
 * The controller selects the policy fusion weights based on the
 * declared priority of each routing request.
 */
enum class EliteMessageType : uint8_t
{
    SAFETY     = 0, ///< Safety-critical messages (BSM, DENM). Prioritise PDR and delay.
    DATA       = 1, ///< General data traffic. Balanced weights.
    STREAMING  = 2, ///< Video / infotainment. Prioritise cost efficiency.
    CONTROL    = 3, ///< SDN control channel. Prioritise delay and cost.
};

/// Number of message types supported by the fusion module.
constexpr uint8_t ELITE_MSG_TYPE_COUNT = 4;

/// Number of Q-learning objectives (must match EliteTrainingObjective count).
constexpr uint8_t ELITE_OBJECTIVE_COUNT = 4;

/**
 * \ingroup elite
 * \brief Logical SDN controller for the ELITE hierarchical routing scheme.
 *
 * EliteController acts as a logical global object (not attached to any
 * physical ns-3 Node). Vehicle routing protocols hold a Ptr<EliteController>
 * and invoke RequestPath() directly (zero network overhead). If control-plane
 * latency needs to be modelled, the caller should introduce a simulated delay
 * via Simulator::Schedule before acting on the returned path.
 *
 * Internal operation (per training cycle):
 *  1. SyncPhysicalToTwin() – mirror all vehicle states from NodeList into the
 *     TwinEnvironment.
 *  2. RunTrainingCycle()   – drive each of the four EliteTrainers for a fixed
 *     number of episodes using the refreshed twin state.
 *  3. On each routing request: fuse the four Q-tables using per-message-type
 *     weights and compute a junction-level path via a greedy walk.
 */
class EliteController : public Object
{
  public:
    /**
     * \brief Get the ns-3 TypeId for attribute binding.
     * \return TypeId
     */
    static TypeId GetTypeId();

    EliteController();
    ~EliteController() override;

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    /**
     * \brief Attach the digital twin environment used by all trainers.
     * \param twin pointer to an initialised TwinEnvironment
     */
    void SetTwinEnvironment(Ptr<TwinEnvironment> twin);

    /**
     * \brief Return the attached twin environment.
     */
    Ptr<TwinEnvironment> GetTwinEnvironment() const;

    /**
     * \brief Set the interval at which the controller syncs physical state
     *        and drives a training cycle.
     * \param interval simulation time between cycles
     */
    void SetCycleInterval(Time interval);

    /**
     * \brief Return the current cycle interval.
     */
    Time GetCycleInterval() const;

    /**
     * \brief Set the number of training episodes per objective per cycle.
     * \param episodes episode count (≥ 1)
     */
    void SetEpisodesPerCycle(uint32_t episodes);

    /**
     * \brief Return the number of episodes per objective per cycle.
     */
    uint32_t GetEpisodesPerCycle() const;

    /**
     * \brief Override the fusion weight table for a specific message type.
     *
     * \param type    message type index
     * \param weights array of ELITE_OBJECTIVE_COUNT weights (PDR, DELAY, HOP, COST)
     *
     * Weights are automatically normalised to sum to 1.0.
     */
    void SetFusionWeights(EliteMessageType type,
                          const std::array<double, ELITE_OBJECTIVE_COUNT>& weights);

    // -------------------------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------------------------

    /**
     * \brief Initialise trainers and schedule the first training cycle.
     *
     * Must be called after SetTwinEnvironment() and, if needed, after
     * TwinEnvironment::InitializeRoadGraphFromSumoNetXml().
     */
    void Start();

    /**
     * \brief Cancel all pending scheduled events.
     */
    void Stop();

    // -------------------------------------------------------------------------
    // Routing interface (called by vehicle routing protocols)
    // -------------------------------------------------------------------------

    /**
     * \brief Request a junction-level path between two vehicles.
     *
     * The controller maps each vehicle ID to the nearest junction, then walks
     * the fused Q-table to build a junction sequence.  Returns an empty vector
     * if no path can be found.
     *
     * \param srcVehicleId  source vehicle identifier
     * \param dstVehicleId  destination vehicle identifier
     * \param type          message priority class (drives fusion weights)
     * \return ordered list of junction IDs from source to destination
     */
    std::vector<std::string> RequestPath(uint64_t srcVehicleId,
                                         uint64_t dstVehicleId,
                                         EliteMessageType type);

    /**
     * \brief Request a path between two known junction IDs directly.
     *
     * Use this variant when the caller has already resolved junction IDs.
     *
     * \param srcJunction  source junction identifier
     * \param dstJunction  destination junction identifier
     * \param type         message priority class
     * \return ordered list of junction IDs
     */
    std::vector<std::string> RequestPathByJunction(const std::string& srcJunction,
                                                    const std::string& dstJunction,
                                                    EliteMessageType type);

    /**
     * \brief Map a vehicle to the closest junction in the twin road graph.
     *
     * Returns an empty string if the vehicle is unknown or the twin has no
     * junctions.
     *
     * \param vehicleId vehicle identifier
     */
    std::string VehicleToJunction(uint64_t vehicleId) const;

    // -------------------------------------------------------------------------
    // Training cycle (public for testing; normally driven by scheduler)
    // -------------------------------------------------------------------------

    /**
     * \brief Mirror all ns-3 node positions/velocities into the twin.
     *
     * Iterates NodeList and queries each node's MobilityModel.  Nodes without
     * a MobilityModel are skipped.
     */
    void SyncPhysicalToTwin();

    /**
     * \brief Run one round of training across all four objectives.
     *
     * For each objective, m_episodesPerCycle episode pairs are sampled
     * uniformly from the junctions in the twin and forwarded to the
     * corresponding EliteTrainer.
     */
    void RunTrainingCycle();

  private:
    // -------------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------------

    /// Build the fused Q-table for a given message type.
    EliteQTable BuildFusedQTable(EliteMessageType type) const;

    /// Walk the fused Q-table greedily to produce a junction path.
    std::vector<std::string> WalkFusedTable(const EliteQTable& fused,
                                             const std::string& src,
                                             const std::string& dst) const;

    /// Schedule the next SyncAndTrain event.
    void ScheduleNextCycle();

    /// Combined sync + train callback invoked by the scheduler.
    void OnCycleTick();

    // -------------------------------------------------------------------------
    // Members
    // -------------------------------------------------------------------------

    Ptr<TwinEnvironment> m_twin;  ///< Digital twin state manager

    /// One trainer and one Q-table per objective (PDR, DELAY, HOP, COST).
    std::array<EliteTrainer, ELITE_OBJECTIVE_COUNT>  m_trainers;
    std::array<EliteQTable,  ELITE_OBJECTIVE_COUNT>  m_qTables;
    std::array<EliteRewardModel, ELITE_OBJECTIVE_COUNT> m_rewardModels;

    /// Fusion weight table: m_fusionWeights[msgType][objective].
    std::array<std::array<double, ELITE_OBJECTIVE_COUNT>, ELITE_MSG_TYPE_COUNT>
        m_fusionWeights;

    Time     m_cycleInterval;    ///< How often to sync + train
    uint32_t m_episodesPerCycle; ///< Episodes per objective per cycle
    bool     m_running;          ///< Whether the periodic loop is active
};

} // namespace ns3

#endif /* ELITE_CONTROLLER_H */
