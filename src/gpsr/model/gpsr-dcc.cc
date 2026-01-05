/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR DCC - Decentralized Congestion Control for GPSR HELLO messages
 * Adapted from VaN3Twin DCC module
 */

#include "gpsr-dcc.h"
#include "gpsr-metric-supervisor.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"

#include <fstream>
#include <cmath>

namespace ns3
{
namespace gpsr
{

NS_LOG_COMPONENT_DEFINE("GpsrDcc");
NS_OBJECT_ENSURE_REGISTERED(GpsrDcc);

TypeId
GpsrDcc::GetTypeId()
{
    static TypeId tid = TypeId("ns3::gpsr::GpsrDcc")
                            .SetParent<Object>()
                            .SetGroupName("Gpsr")
                            .AddConstructor<GpsrDcc>();
    return tid;
}

GpsrDcc::GpsrDcc()
{
    NS_LOG_FUNCTION(this);
}

GpsrDcc::~GpsrDcc()
{
    NS_LOG_FUNCTION(this);
}

std::unordered_map<GpsrDcc::ReactiveState, GpsrDcc::ReactiveParameters>
GpsrDcc::GetConfiguration(double ton, double currentCBR)
{
    std::unordered_map<ReactiveState, ReactiveParameters> map;

    if (ton < 0.5)
    {
        map = m_reactiveParams500us;
    }
    else
    {
        map = m_reactiveParams1ms;
    }

    // State transition logic
    ReactiveState oldState = m_currentState;

    if (currentCBR >= map[m_currentState].cbrThreshold && m_currentState != Restrictive)
    {
        m_currentState = static_cast<ReactiveState>(m_currentState + 1);
    }
    else if (m_currentState != Relaxed)
    {
        ReactiveState prevState = static_cast<ReactiveState>(m_currentState - 1);
        if (currentCBR <= map[prevState].cbrThreshold)
        {
            m_currentState = static_cast<ReactiveState>(m_currentState - 1);
        }
    }

    if (oldState != m_currentState)
    {
        NS_LOG_INFO("GpsrDcc: State transition " << oldState << " -> " << m_currentState
                                                  << " (CBR=" << currentCBR << ")");
    }

    return map;
}

void
GpsrDcc::SetupDCC(const std::string& nodeId,
                  Ptr<GpsrMetricSupervisor> metricSupervisor,
                  Ptr<Node> node,
                  const std::string& modality,
                  uint32_t dccInterval,
                  float cbrTarget)
{
    NS_ASSERT_MSG(modality == "adaptive" || modality == "reactive",
                  "DCC modality must be 'adaptive' or 'reactive'");
    NS_ASSERT_MSG(dccInterval > 0, "DCC interval must be greater than 0");
    NS_ASSERT_MSG(metricSupervisor, "MetricSupervisor is null");
    NS_ASSERT_MSG(node, "Node is null");

    m_nodeId = nodeId;
    m_node = node;
    m_modality = modality;
    m_dccInterval = dccInterval;
    m_metricSupervisor = metricSupervisor;
    m_cbrTarget = cbrTarget;

    NS_LOG_INFO("GpsrDcc: Setup complete for node " << nodeId << " mode=" << modality);
}

void
GpsrDcc::StartDCC()
{
    if (m_modality == "adaptive")
    {
        Simulator::Schedule(MilliSeconds(m_tCbr), &GpsrDcc::AdaptiveDCCCheckCBR, this);
        Simulator::Schedule(MilliSeconds(m_dccInterval), &GpsrDcc::AdaptiveDCC, this);
    }
    else
    {
        Simulator::Schedule(MilliSeconds(m_dccInterval), &GpsrDcc::ReactiveDCC, this);
    }

    NS_LOG_INFO("GpsrDcc: Started " << m_modality << " DCC for node " << m_nodeId);
}

void
GpsrDcc::ReactiveDCC()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(m_metricSupervisor, "Metric Supervisor not set");

    double currentCBR = m_metricSupervisor->GetCBRPerNode(m_nodeId);
    if (currentCBR < 0)
    {
        // CBR not yet available
        Simulator::Schedule(MilliSeconds(m_dccInterval), &GpsrDcc::ReactiveDCC, this);
        return;
    }

    // Get configuration and update state
    auto map = GetConfiguration(m_tonPp, currentCBR);

    // Get PHY and adjust parameters (if WiFi device available)
    Ptr<NetDevice> netDevice = m_node->GetDevice(0);
    Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(netDevice);

    if (wifiDevice)
    {
        Ptr<WifiPhy> phy = wifiDevice->GetPhy();
        phy->SetTxPowerStart(map[m_currentState].txPower);
        phy->SetTxPowerEnd(map[m_currentState].txPower);
        phy->SetRxSensitivity(map[m_currentState].sensitivity);
    }

    UpdateTgoAfterStateCheck(map[m_currentState].txInterPacketTime);

    NS_LOG_DEBUG("GpsrDcc Reactive: CBR=" << currentCBR << " state=" << m_currentState
                                          << " Toff=" << m_toffMs << "ms");

    Simulator::Schedule(MilliSeconds(m_dccInterval), &GpsrDcc::ReactiveDCC, this);
}

void
GpsrDcc::AdaptiveDCCCheckCBR()
{
    NS_ASSERT_MSG(m_metricSupervisor, "Metric Supervisor not set");

    m_previousCbr = m_metricSupervisor->GetCBRPerNode(m_nodeId);
    if (m_previousCbr < 0)
    {
        m_previousCbr = 0;
    }

    Simulator::Schedule(MilliSeconds(m_tCbr), &GpsrDcc::AdaptiveDCCCheckCBR, this);
}

void
GpsrDcc::AdaptiveDCC()
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT_MSG(m_metricSupervisor, "Metric Supervisor not set");

    double currentCBR = m_metricSupervisor->GetCBRPerNode(m_nodeId);
    if (currentCBR < 0)
    {
        Simulator::Schedule(MilliSeconds(m_dccInterval), &GpsrDcc::AdaptiveDCC, this);
        return;
    }

    // Step 1: Update CBR_ITS (exponential moving average)
    if (m_cbrIts >= 0)
    {
        m_cbrIts = 0.5 * m_cbrIts + 0.25 * ((currentCBR + m_previousCbr) / 2);
    }
    else
    {
        m_cbrIts = (currentCBR + m_previousCbr) / 2;
    }

    // Step 2: Calculate delta offset
    double factor1 = m_beta * (m_cbrTarget - m_cbrIts);
    double deltaOffset;
    if ((m_cbrTarget - m_cbrIts) > 0)
    {
        deltaOffset = (factor1 < m_gMax) ? factor1 : m_gMax;
    }
    else
    {
        deltaOffset = (factor1 > m_gMin) ? factor1 : m_gMin;
    }

    // Step 3: Update delta
    double newDelta = (1 - m_alpha) * m_delta + deltaOffset;

    // Step 4 & 5: Clamp delta
    if (newDelta > m_deltaMax)
    {
        m_delta = m_deltaMax;
    }
    else if (newDelta < m_deltaMin)
    {
        m_delta = m_deltaMin;
    }
    else
    {
        m_delta = newDelta;
    }

    UpdateTgoAfterDeltaUpdate();

    NS_LOG_DEBUG("GpsrDcc Adaptive: CBR=" << currentCBR << " CBR_ITS=" << m_cbrIts
                                          << " delta=" << m_delta << " Toff=" << m_toffMs << "ms");

    Simulator::Schedule(MilliSeconds(m_dccInterval), &GpsrDcc::AdaptiveDCC, this);
}

void
GpsrDcc::UpdateTgoAfterTransmission()
{
    float aux;
    if (m_delta > 0 && m_tonPp / m_delta > 25)
    {
        aux = m_tonPp / m_delta;
    }
    else
    {
        aux = 25;
    }

    if (aux > 1000)
    {
        aux = 1000;
    }

    m_tpgMs = static_cast<float>(Simulator::Now().GetMilliSeconds());
    m_tgoMs = m_tpgMs + aux;
    m_toffMs = aux;
    m_lastTx = m_tpgMs;

    NS_LOG_DEBUG("GpsrDcc: After TX, Toff=" << m_toffMs << "ms, next gate opens at " << m_tgoMs);
}

void
GpsrDcc::NotifyTx(int64_t nowMs)
{
    if (m_modality == "adaptive")
    {
        UpdateTgoAfterTransmission();
    }
    else
    {
        SetLastTx(static_cast<float>(nowMs));
    }
}

void
GpsrDcc::UpdateTgoAfterDeltaUpdate()
{
    auto now = static_cast<float>(Simulator::Now().GetMilliSeconds());

    if (CheckGateOpen(static_cast<int64_t>(now)))
    {
        return;  // Gate already open, no need to update
    }

    float aux = m_tonPp / m_delta;
    aux = aux * ((m_tgoMs - now) / (m_tgoMs - m_tpgMs));
    aux = aux + (now - m_tpgMs);

    if (aux < 25)
    {
        aux = 25;
    }
    if (aux > 1000)
    {
        aux = 1000;
    }

    m_tgoMs = m_tpgMs + aux;
    m_toffMs = aux;
}

void
GpsrDcc::UpdateTgoAfterStateCheck(uint32_t toff)
{
    auto now = static_cast<float>(Simulator::Now().GetMilliSeconds());
    m_tgoMs = now + toff;
    m_toffMs = static_cast<float>(toff);
}

bool
GpsrDcc::CheckGateOpen(int64_t nowMs)
{
    bool gateOpen = (nowMs - static_cast<int64_t>(m_lastTx)) >= static_cast<int64_t>(m_toffMs);
    return gateOpen;
}

void
GpsrDcc::UpdateTonpp(ssize_t pktSize)
{
    double bits = pktSize * 8;
    double txDurationS = static_cast<double>(bits) / m_bitrateBps;
    double totalDurationS = txDurationS + (68e-6);  // 68 µs extra overhead
    m_tonPp = static_cast<float>(totalDurationS * 1000.0);
}

} // namespace gpsr
} // namespace ns3
