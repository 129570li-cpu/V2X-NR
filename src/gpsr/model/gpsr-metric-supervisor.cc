/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Metric Supervisor - CBR Monitoring for DCC
 * Adapted from VaN3Twin MetricSupervisor
 */

#include "gpsr-metric-supervisor.h"
#include "ns3/log.h"
#include "ns3/config.h"
#include "ns3/simulator.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy-state.h"

#include <fstream>
#include <iomanip>

namespace ns3
{
namespace gpsr
{

NS_LOG_COMPONENT_DEFINE("GpsrMetricSupervisor");
NS_OBJECT_ENSURE_REGISTERED(GpsrMetricSupervisor);

// Global state for PHY callbacks
std::unordered_map<std::string, Time> g_currentBusyCBR;
std::unordered_map<std::string, std::pair<Time, WifiPhyState>> g_nodeLastState80211p;
Time g_lastCBRCheck = Time(-1.0);

// Helper: check if channel state is busy
static bool
IsChannelBusy(WifiPhyState state)
{
    return state != WifiPhyState::SLEEP && state != WifiPhyState::IDLE;
}

// Callback for 802.11p PHY state changes
void
StoreCBR80211p(std::string context, Time start, Time duration, WifiPhyState state)
{
    // Extract node ID from context: /NodeList/<id>/...
    std::size_t first = context.find("/NodeList/") + 10;
    std::size_t last = context.find("/", first);
    std::string node = context.substr(first, last - first);

    if (IsChannelBusy(state))
    {
        // Check if the last measurement started before the last CBR check
        if (start < g_lastCBRCheck)
        {
            duration -= g_lastCBRCheck - start;
            if (duration.IsNegative())
            {
                duration = Seconds(0);
            }
        }

        if (g_currentBusyCBR.find(node) == g_currentBusyCBR.end())
        {
            g_currentBusyCBR[node] = duration;
        }
        else
        {
            g_currentBusyCBR[node] += duration;
        }

        g_nodeLastState80211p[node].first = Simulator::Now();
        g_nodeLastState80211p[node].second = WifiPhyState::CCA_BUSY;
    }
    else
    {
        if (g_currentBusyCBR.find(node) == g_currentBusyCBR.end())
        {
            g_currentBusyCBR[node] = Time(0);
        }
        g_nodeLastState80211p[node].first = Simulator::Now();
        g_nodeLastState80211p[node].second = WifiPhyState::IDLE;
    }
}

TypeId
GpsrMetricSupervisor::GetTypeId()
{
    static TypeId tid = TypeId("ns3::gpsr::GpsrMetricSupervisor")
                            .SetParent<Object>()
                            .SetGroupName("Gpsr")
                            .AddConstructor<GpsrMetricSupervisor>();
    return tid;
}

GpsrMetricSupervisor::GpsrMetricSupervisor()
{
    NS_LOG_FUNCTION(this);
}

GpsrMetricSupervisor::~GpsrMetricSupervisor()
{
    NS_LOG_FUNCTION(this);

    // Cancel pending events
    for (auto& event : m_eventList)
    {
        if (!event.IsExpired())
        {
            Simulator::Cancel(event);
        }
    }
    m_eventList.clear();
}

void
GpsrMetricSupervisor::SetChannelTechnology(const std::string& tech)
{
    if (tech != "80211p" && tech != "Nr")
    {
        NS_FATAL_ERROR("Invalid channel technology. Must be '80211p' or 'Nr'.");
    }
    m_channelTechnology = tech;
}

void
GpsrMetricSupervisor::StartCheckCBR(int numNodes)
{
    NS_ASSERT_MSG(m_cbrWindow > 0, "CBR window must be greater than 0");
    NS_ASSERT_MSG(m_cbrAlpha >= 0 && m_cbrAlpha <= 1, "CBR alpha must be between 0 and 1");
    NS_ASSERT_MSG(!m_channelTechnology.empty(), "Channel technology must be set");
    NS_ASSERT_MSG(m_nodeContainer.GetN() != 0, "Node container must be filled before CBR checking");

    uint32_t nodesToCheck = (numNodes == -1) ? m_nodeContainer.GetN() : static_cast<uint32_t>(numNodes);

    for (uint32_t i = 0; i < nodesToCheck; i++)
    {
        Ptr<Node> node = m_nodeContainer.Get(i);
        std::ostringstream oss;

        if (m_channelTechnology == "80211p")
        {
            oss << "/NodeList/" << node->GetId() << "/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State";
            Config::Connect(oss.str(), MakeCallback(&StoreCBR80211p));
        }
        // NR support can be added here if needed
    }

    Simulator::Schedule(MilliSeconds(m_cbrWindow), &GpsrMetricSupervisor::CheckCBR, this);

    if (m_simulationTime > 0)
    {
        Simulator::Schedule(Seconds(m_simulationTime), &GpsrMetricSupervisor::LogLastCBRs, this);
    }

    NS_LOG_INFO("GpsrMetricSupervisor: Started CBR monitoring for " << nodesToCheck << " nodes");
}

void
GpsrMetricSupervisor::CheckCBR()
{
    // Process each node
    for (uint32_t i = 0; i < m_nodeContainer.GetN(); i++)
    {
        Ptr<Node> node = m_nodeContainer.Get(i);
        std::string nodeId = std::to_string(node->GetId());

        if (g_currentBusyCBR.find(nodeId) == g_currentBusyCBR.end())
        {
            continue;
        }

        Time busyCbr = g_currentBusyCBR[nodeId];
        double currentCbr = busyCbr.GetDouble() / (m_cbrWindow * 1e6);

        // Exponential moving average
        if (m_averageCbr.find(nodeId) != m_averageCbr.end() && !m_averageCbr[nodeId].empty())
        {
            double newCbr = m_cbrAlpha * m_averageCbr[nodeId].back() + (1 - m_cbrAlpha) * currentCbr;
            m_averageCbr[nodeId].push_back(newCbr);
        }
        else
        {
            m_averageCbr[nodeId].push_back(currentCbr);
        }

        if (m_cbrVerbose)
        {
            NS_LOG_INFO("Node " << nodeId << " CBR: " << std::fixed << std::setprecision(2)
                                << (currentCbr * 100) << "%");
        }
    }

    // Clear state for next window
    g_currentBusyCBR.clear();
    if (m_channelTechnology == "80211p")
    {
        g_nodeLastState80211p.clear();
    }
    g_lastCBRCheck = Simulator::Now();

    // Schedule next check
    Simulator::Schedule(MilliSeconds(m_cbrWindow), &GpsrMetricSupervisor::CheckCBR, this);
}

double
GpsrMetricSupervisor::GetCBRPerNode(const std::string& nodeId)
{
    if (m_averageCbr.find(nodeId) != m_averageCbr.end() && !m_averageCbr[nodeId].empty())
    {
        return m_averageCbr[nodeId].back();
    }
    return -1.0;
}

float
GpsrMetricSupervisor::GetAverageCBROverall()
{
    float sum = 0;
    int count = 0;

    for (const auto& pair : m_averageCbr)
    {
        if (!pair.second.empty())
        {
            sum += static_cast<float>(pair.second.back());
            count++;
        }
    }

    return (count > 0) ? (sum / count) : 0.0f;
}

std::unordered_map<std::string, std::vector<double>>
GpsrMetricSupervisor::GetCBRValues()
{
    return m_averageCbr;
}

void
GpsrMetricSupervisor::LogLastCBRs()
{
    if (m_cbrVerbose)
    {
        std::cout << "CBR final values for each node:" << std::endl;
        for (const auto& pair : m_averageCbr)
        {
            if (!pair.second.empty())
            {
                std::cout << "Node " << pair.first << ": " << std::fixed << std::setprecision(2)
                          << (pair.second.back() * 100) << "%" << std::endl;
            }
        }
    }

    if (m_cbrWriteToFile)
    {
        std::ofstream file("gpsr_cbr_values.txt");
        file << "CBR final values for each node:" << std::endl;
        for (const auto& pair : m_averageCbr)
        {
            if (!pair.second.empty())
            {
                file << "Node " << pair.first << ": " << std::fixed << std::setprecision(2)
                     << (pair.second.back() * 100) << "%" << std::endl;
            }
        }
        file.close();
    }
}

} // namespace gpsr
} // namespace ns3
