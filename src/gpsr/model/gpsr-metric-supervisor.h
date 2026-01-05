/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Metric Supervisor - CBR Monitoring for DCC
 * Adapted from VaN3Twin MetricSupervisor
 * 
 * Original authors:
 *  Marco Malinverno, Politecnico di Torino
 *  Francesco Raviglione, Politecnico di Torino
 *  Diego Gasco, Politecnico di Torino
 *  Carlos Mateo Risma Carletti, Politecnico di Torino
 */

#ifndef GPSR_METRIC_SUPERVISOR_H
#define GPSR_METRIC_SUPERVISOR_H

#include <list>
#include <unordered_map>
#include <string>
#include <vector>
#include "ns3/event-id.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/object.h"

// Forward declaration - wifi-phy-state.h included only in .cc file
// to avoid NS_LOG_APPEND_CONTEXT conflicts

// TraCI support (optional)
#ifdef HAS_TRACI
#include "ns3/traci-client.h"
#endif

namespace ns3
{
namespace gpsr
{

/**
 * \ingroup gpsr
 * \brief CBR (Channel Busy Ratio) monitoring for GPSR DCC
 *
 * This class monitors the channel busy ratio for 802.11p networks
 * to support Decentralized Congestion Control (DCC) in GPSR.
 */
class GpsrMetricSupervisor : public Object
{
public:
    static TypeId GetTypeId();

    GpsrMetricSupervisor();
    virtual ~GpsrMetricSupervisor();

    /**
     * \brief Set the NodeContainer for CBR monitoring
     * \param nc NodeContainer with all nodes to monitor
     */
    void SetNodeContainer(NodeContainer nc) { m_nodeContainer = nc; }

#ifdef HAS_TRACI
    /**
     * \brief Set the TraCI client pointer
     * \param traci_ptr Pointer to TraCI client
     */
    void SetTraCIClient(Ptr<TraciClient> traci_ptr) { m_traciPtr = traci_ptr; }
#endif

    /**
     * \brief Start CBR monitoring
     * \param numNodes Number of nodes to monitor (-1 for all)
     */
    void StartCheckCBR(int numNodes = -1);

    /**
     * \brief Get CBR for a specific node
     * \param nodeId Node ID (as string)
     * \return CBR value (0.0-1.0), or -1.0 if not available
     */
    double GetCBRPerNode(const std::string& nodeId);

    /**
     * \brief Get average CBR across all nodes
     * \return Average CBR value (0.0-1.0)
     */
    float GetAverageCBROverall();

    /**
     * \brief Get all CBR values
     * \return Map of node ID to CBR history
     */
    std::unordered_map<std::string, std::vector<double>> GetCBRValues();

    // Configuration setters
    void SetCBRWindow(float windowMs) { m_cbrWindow = windowMs; }
    void SetCBRAlpha(float alpha) { m_cbrAlpha = alpha; }
    void SetSimulationTime(float simTime) { m_simulationTime = simTime; }
    void SetChannelTechnology(const std::string& tech);

    // Verbose output
    void EnableCBRVerbose() { m_cbrVerbose = true; }
    void DisableCBRVerbose() { m_cbrVerbose = false; }

private:
    /**
     * \brief Periodic CBR computation
     */
    void CheckCBR();

    /**
     * \brief Log final CBR values
     */
    void LogLastCBRs();

    // Node management
    NodeContainer m_nodeContainer;

#ifdef HAS_TRACI
    Ptr<TraciClient> m_traciPtr{nullptr};
#endif

    // CBR parameters
    float m_cbrWindow{100.0};     ///< CBR window in milliseconds
    float m_cbrAlpha{0.5};        ///< Exponential moving average alpha
    float m_simulationTime{-1};   ///< Simulation time in seconds
    std::string m_channelTechnology{"80211p"};  ///< Channel technology

    // CBR data storage
    std::unordered_map<std::string, std::vector<double>> m_averageCbr;

    // Output control
    bool m_cbrVerbose{false};
    bool m_cbrWriteToFile{false};

    // Event management
    std::list<EventId> m_eventList;
};

// Note: Global state variables for PHY callbacks are defined in gpsr-metric-supervisor.cc
// They cannot be declared in the header due to WifiPhyState dependency issues

} // namespace gpsr
} // namespace ns3

#endif // GPSR_METRIC_SUPERVISOR_H

