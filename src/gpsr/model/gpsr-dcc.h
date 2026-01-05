/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR DCC - Decentralized Congestion Control for GPSR HELLO messages
 * Adapted from VaN3Twin DCC module
 *
 * Original author:
 *  Diego Gasco, Politecnico di Torino
 */

#ifndef GPSR_DCC_H
#define GPSR_DCC_H

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>

#include "ns3/object.h"
#include "ns3/node.h"
#include "ns3/ptr.h"

// Forward declaration - actual includes in gpsr-dcc.cc
namespace ns3 { namespace gpsr { class GpsrMetricSupervisor; } }

namespace ns3
{
namespace gpsr
{

/**
 * \ingroup gpsr
 * \brief Decentralized Congestion Control (DCC) for GPSR HELLO messages
 *
 * Implements both Reactive and Adaptive DCC algorithms according to ETSI standards.
 */
class GpsrDcc : public Object
{
public:
    static TypeId GetTypeId();

    GpsrDcc();
    virtual ~GpsrDcc();

    /**
     * \brief Setup DCC with a metric supervisor
     * \param nodeId Node identifier string
     * \param metricSupervisor Pointer to CBR supervisor
     * \param node The node this DCC is attached to
     * \param modality "reactive" or "adaptive"
     * \param dccInterval DCC check interval in ms
     * \param cbrTarget Target CBR for adaptive mode (default 0.63)
     */
    void SetupDCC(const std::string& nodeId,
                  Ptr<GpsrMetricSupervisor> metricSupervisor,
                  Ptr<Node> node,
                  const std::string& modality,
                  uint32_t dccInterval,
                  float cbrTarget = 0.63);

    /**
     * \brief Start DCC operation
     */
    void StartDCC();

    /**
     * \brief Check if the DCC gate is open (allowed to send)
     * \param nowMs Current time in milliseconds
     * \return true if transmission is allowed
     */
    bool CheckGateOpen(int64_t nowMs);

    /**
     * \brief Update gate timing after transmission
     */
    void UpdateTgoAfterTransmission();

    /**
     * \brief Notify DCC of a transmission at the given time (ms)
     * Applies adaptive or reactive behavior based on modality.
     */
    void NotifyTx(int64_t nowMs);

    /**
     * \brief Get the current Toff (minimum inter-packet time) in ms
     */
    float GetToff() const { return m_toffMs; }

    /**
     * \brief Get current DCC modality
     */
    std::string GetModality() const { return m_modality; }

    /**
     * \brief Set the bitrate for Ton calculation
     */
    void SetBitRate(long bitrateBps) { m_bitrateBps = bitrateBps; }

    /**
     * \brief Set last transmission time
     */
    void SetLastTx(float t) { m_lastTx = t; }

    /**
     * \brief Update Ton (packet transmission time) based on packet size
     */
    void UpdateTonpp(ssize_t pktSize);

private:
    /**
     * DCC Reactive states
     */
    enum ReactiveState
    {
        Relaxed = 0,
        Active1 = 1,
        Active2 = 2,
        Active3 = 3,
        Restrictive = 4
    };

    /**
     * Parameters for each reactive state
     */
    struct ReactiveParameters
    {
        double cbrThreshold;
        double txPower;
        double dataRate;
        long txInterPacketTime;  // ms
        double sensitivity;
    };

    // Reactive DCC state machine
    void ReactiveDCC();

    // Adaptive DCC algorithm
    void AdaptiveDCC();
    void AdaptiveDCCCheckCBR();

    // State transition helper
    std::unordered_map<ReactiveState, ReactiveParameters> GetConfiguration(double ton, double currentCBR);

    // Update Tgo after state check
    void UpdateTgoAfterStateCheck(uint32_t toff);

    // Update Tgo after delta update
    void UpdateTgoAfterDeltaUpdate();

    // Reactive parameters for Ton = 0.5ms
    const std::unordered_map<ReactiveState, ReactiveParameters> m_reactiveParams500us = {
        {Relaxed,     {0.3, 30.0, -1, 50, -95.0}},
        {Active1,     {0.4, 30.0, -1, 100, -95.0}},
        {Active2,     {0.5, 30.0, -1, 200, -95.0}},
        {Active3,     {0.65, 12.0, -1, 250, -95.0}},
        {Restrictive, {1.0, 6.0, -1, 1000, -65.0}}
    };

    // Reactive parameters for Ton = 1ms
    const std::unordered_map<ReactiveState, ReactiveParameters> m_reactiveParams1ms = {
        {Relaxed,     {0.3, 30.0, -1, 100, -95.0}},
        {Active1,     {0.4, 30.0, -1, 200, -95.0}},
        {Active2,     {0.5, 18.0, -1, 400, -95.0}},
        {Active3,     {0.6, 12.0, -1, 500, -95.0}},
        {Restrictive, {1.0, 6.0, -1, 1000, -65.0}}
    };

    // Configuration
    std::string m_nodeId;
    Ptr<Node> m_node;
    std::string m_modality;
    uint32_t m_dccInterval{100};
    Ptr<GpsrMetricSupervisor> m_metricSupervisor{nullptr};
    ReactiveState m_currentState{Relaxed};

    // Adaptive DCC parameters
    double m_cbrIts{-1};
    double m_alpha{0.016};
    double m_beta{0.0012};
    double m_cbrTarget{0.68};
    double m_deltaMax{0.03};
    double m_deltaMin{0.0006};
    double m_gMax{0.0005};
    double m_gMin{-0.00025};
    uint32_t m_tCbr{100};  // CBR check interval for adaptive DCC
    double m_delta{0};
    double m_previousCbr{-1};

    // Timing parameters
    float m_tpgMs{0.0};      // Time of previous packet generation
    float m_tgoMs{0.0};      // Time gate opens
    float m_tonPp{0.5};      // Packet transmission time (ms)
    float m_toffMs{0.0};     // Minimum inter-packet time
    float m_lastTx{0.0};     // Last transmission time
    long m_bitrateBps{6000000};  // Default 6 Mbps

    // Statistics
    uint32_t m_droppedByGate{0};
};

} // namespace gpsr
} // namespace ns3

#endif // GPSR_DCC_H
