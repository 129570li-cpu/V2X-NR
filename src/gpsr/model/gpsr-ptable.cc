/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Position Table - Complete Implementation based on old version
 */

#include "gpsr-ptable.h"

#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("GpsrPositionTable");

namespace gpsr
{

PositionTable::PositionTable()
    : m_entryLifeTime(Seconds(2))
{
    m_txErrorCallback = MakeCallback(&PositionTable::ProcessTxError, this);
}

Time
PositionTable::GetEntryUpdateTime(Ipv4Address id)
{
    if (id == Ipv4Address::GetZero())
    {
        return Time(Seconds(0));
    }
    auto i = m_table.find(id);
    if (i != m_table.end())
    {
        return i->second.lastUpdate;
    }
    return Time(Seconds(0));
}

void
PositionTable::AddEntry(Ipv4Address id, Vector position)
{
    NS_LOG_FUNCTION(this << id << position);

    auto i = m_table.find(id);
    if (i != m_table.end())
    {
        // Preserve existing SINR data, only update position and time
        i->second.position = position;
        i->second.lastUpdate = Simulator::Now();
        NS_LOG_DEBUG("Updated neighbor " << id << " preserving SINR=" << i->second.sinr);
    }
    else
    {
        // New neighbor, initialize with invalid SINR
        NeighborEntry entry;
        entry.position = position;
        entry.lastUpdate = Simulator::Now();
        entry.sinr = -1.0;
        entry.lastSinrUpdate = Seconds(0);
        m_table.insert(std::make_pair(id, entry));
        NS_LOG_DEBUG("Added new neighbor " << id << " with SINR=-1.0");
    }
}

void
PositionTable::AddEntryExtended(Ipv4Address id, 
                                 Vector position, 
                                 Vector velocity,
                                 const std::vector<NeighborSummary>& twoHopNeighbors)
{
    NS_LOG_FUNCTION(this << id << position << velocity);

    auto i = m_table.find(id);
    if (i != m_table.end())
    {
        // Preserve existing SINR data, update position, velocity, and two-hop
        i->second.position = position;
        i->second.velocity = velocity;
        i->second.lastUpdate = Simulator::Now();
        i->second.twoHopNeighbors = twoHopNeighbors;
        NS_LOG_DEBUG("Updated neighbor " << id << " vel(" << velocity.x << "," << velocity.y 
                     << ") 2hop:" << twoHopNeighbors.size() << " preserving SINR=" << i->second.sinr);
    }
    else
    {
        // New neighbor with full info
        NeighborEntry entry;
        entry.position = position;
        entry.velocity = velocity;
        entry.lastUpdate = Simulator::Now();
        entry.sinr = -1.0;
        entry.smoothedSinr = -1.0;
        entry.lastSinrUpdate = Seconds(0);
        entry.twoHopNeighbors = twoHopNeighbors;
        m_table.insert(std::make_pair(id, entry));
        NS_LOG_DEBUG("Added new neighbor " << id << " vel(" << velocity.x << "," << velocity.y 
                     << ") 2hop:" << twoHopNeighbors.size());
    }
}

void
PositionTable::DeleteEntry(Ipv4Address id)
{
    NS_LOG_FUNCTION(this << id);
    m_table.erase(id);
}

void
PositionTable::UpdateSinr(Ipv4Address id, double sinr)
{
    NS_LOG_FUNCTION(this << id << sinr);
    auto i = m_table.find(id);
    if (i != m_table.end())
    {
        i->second.sinr = sinr;
        i->second.lastSinrUpdate = Simulator::Now();
        if (sinr > 0.0)
        {
            NS_LOG_DEBUG("Updated SINR for " << id << " to " << sinr << " (" << 10*std::log10(sinr) << " dB)");
        }
        else
        {
            NS_LOG_DEBUG("Updated SINR for " << id << " to " << sinr << " (invalid, skipped dB conversion)");
        }
    }
    else
    {
        NS_LOG_DEBUG("UpdateSinr: neighbor " << id << " not found, ignoring");
    }
}

Vector
PositionTable::GetPosition(Ipv4Address id)
{
    NS_LOG_FUNCTION(this << id);

    // Use God mode - look up position from NodeList
    for (uint32_t i = 0; i < NodeList::GetNNodes(); i++)
    {
        Ptr<Node> node = NodeList::GetNode(i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        if (ipv4)
        {
            for (uint32_t j = 1; j < ipv4->GetNInterfaces(); j++)
            {
                if (ipv4->GetNAddresses(j) > 0 && ipv4->GetAddress(j, 0).GetLocal() == id)
                {
                    Ptr<MobilityModel> mm = node->GetObject<MobilityModel>();
                    if (mm)
                    {
                        return mm->GetPosition();
                    }
                }
            }
        }
    }
    return GetInvalidPosition();
}

bool
PositionTable::IsNeighbour(Ipv4Address id)
{
    NS_LOG_FUNCTION(this << id);
    Purge();
    auto i = m_table.find(id);
    return (i != m_table.end());
}

void
PositionTable::Purge()
{
    NS_LOG_FUNCTION(this);

    if (m_table.empty())
    {
        return;
    }

    std::list<Ipv4Address> toErase;

    for (auto i = m_table.begin(); i != m_table.end(); ++i)
    {
        if (m_entryLifeTime + GetEntryUpdateTime(i->first) <= Simulator::Now())
        {
            toErase.push_back(i->first);
        }
    }

    for (auto& addr : toErase)
    {
        NS_LOG_DEBUG("Purging neighbor " << addr);
        m_table.erase(addr);
    }
}

void
PositionTable::Clear()
{
    NS_LOG_FUNCTION(this);
    m_table.clear();
}

double
PositionTable::CalculateDistance(Vector a, Vector b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

Ipv4Address
PositionTable::BestNeighbor(Vector dstPosition, Vector nodePos)
{
    NS_LOG_FUNCTION(this << dstPosition << nodePos);

    // Single Purge at the beginning - DO NOT call IsNeighbour inside loop
    Purge();

    double initialDistance = CalculateDistance(nodePos, dstPosition);

    if (m_table.empty())
    {
        NS_LOG_DEBUG("BestNeighbor table is empty; Position: " << dstPosition);
        return Ipv4Address::GetZero();
    }

    // SINR threshold and aging parameters
    const double SINR_THRESHOLD = 2.0;  // Linear (~3dB)
    const Time AGING_TIMEOUT = Seconds(1.0);

    Ipv4Address bestQualityID = Ipv4Address::GetZero();
    double bestQualityDistance = std::numeric_limits<double>::max();
    Ipv4Address bestFallbackID = Ipv4Address::GetZero();
    double bestFallbackDistance = std::numeric_limits<double>::max();

    // Two-stage selection: Stage 1 (Quality Filter) and Stage 2 (Fallback)
    for (auto& entry : m_table)
    {
        double distance = CalculateDistance(entry.second.position, dstPosition);

        // Stage 2: Track best fallback (all neighbors)
        if (distance < bestFallbackDistance)
        {
            bestFallbackID = entry.first;
            bestFallbackDistance = distance;
        }

        // Stage 1: Quality filter - valid SINR, above threshold, not aged
        if (entry.second.sinr >= 0.0 &&  // Valid measurement
            entry.second.sinr >= SINR_THRESHOLD &&
            (Simulator::Now() - entry.second.lastSinrUpdate) <= AGING_TIMEOUT)
        {
            if (distance < bestQualityDistance)
            {
                bestQualityID = entry.first;
                bestQualityDistance = distance;
            }
        }
    }

    // Select best candidate: prefer quality-filtered, fallback to any neighbor
    Ipv4Address bestFoundID;
    double bestFoundDistance;
    if (bestQualityID != Ipv4Address::GetZero())
    {
        bestFoundID = bestQualityID;
        bestFoundDistance = bestQualityDistance;
        NS_LOG_DEBUG("Using quality-filtered neighbor: " << bestFoundID);
    }
    else
    {
        bestFoundID = bestFallbackID;
        bestFoundDistance = bestFallbackDistance;
        NS_LOG_DEBUG("Fallback to distance-only neighbor: " << bestFoundID);
    }

    // Progress check: Only return neighbor if it's closer to destination than we are
    if (bestFoundID != Ipv4Address::GetZero() && initialDistance > bestFoundDistance)
    {
        NS_LOG_DEBUG("Best neighbor: " << bestFoundID << " distance: " << bestFoundDistance);
        return bestFoundID;
    }
    else
    {
        NS_LOG_DEBUG("No neighbor closer than current node, entering recovery");
        return Ipv4Address::GetZero(); // Enter Recovery-mode
    }
}

double
PositionTable::GetAngle(Vector centrePos, Vector refPos, Vector node)
{
    const double PI = 4 * std::atan(1.0);

    std::complex<double> A = std::complex<double>(centrePos.x, centrePos.y);
    std::complex<double> B = std::complex<double>(node.x, node.y);
    std::complex<double> C = std::complex<double>(refPos.x, refPos.y);

    std::complex<double> AB = B - A;
    double normAB = std::norm(AB);
    if (normAB == 0)
    {
        return 0;
    }
    AB = (std::real(AB) / normAB) + (std::complex<double>(0.0, 1.0) * (std::imag(AB) / normAB));

    std::complex<double> AC = C - A;
    double normAC = std::norm(AC);
    if (normAC == 0)
    {
        return 0;
    }
    AC = (std::real(AC) / normAC) + (std::complex<double>(0.0, 1.0) * (std::imag(AC) / normAC));

    std::complex<double> tmp = std::log(AC / AB);
    std::complex<double> tmpCplx = std::complex<double>(0.0, -1.0);
    std::complex<double> Angle = tmp * tmpCplx;
    Angle *= (180.0 / PI);

    double angle = std::real(Angle);
    if (angle < 0)
    {
        angle = 360.0 + angle;
    }

    return angle;
}

Ipv4Address
PositionTable::BestAngle(Vector previousHop, Vector nodePos)
{
    NS_LOG_FUNCTION(this << previousHop << nodePos);

    Purge();

    if (m_table.empty())
    {
        NS_LOG_DEBUG("BestAngle table is empty; Position: " << nodePos);
        return Ipv4Address::GetZero();
    }

    double tmpAngle;
    Ipv4Address bestFoundID = Ipv4Address::GetZero();
    double bestFoundAngle = 360.0;

    for (auto i = m_table.begin(); i != m_table.end(); ++i)
    {
        tmpAngle = GetAngle(nodePos, previousHop, i->second.position);
        if (bestFoundAngle > tmpAngle && tmpAngle != 0)
        {
            bestFoundID = i->first;
            bestFoundAngle = tmpAngle;
        }
    }

    // If only neighbor is who sent the packet, use first neighbor
    if (bestFoundID == Ipv4Address::GetZero() && !m_table.empty())
    {
        bestFoundID = m_table.begin()->first;
    }

    NS_LOG_DEBUG("Best angle neighbor: " << bestFoundID << " angle: " << bestFoundAngle);
    return bestFoundID;
}

void
PositionTable::ProcessTxError(WifiMacHeader const&)
{
    // Could be used for link layer feedback
}

bool
PositionTable::IsInSearch(Ipv4Address id)
{
    return false;
}

bool
PositionTable::HasPosition(Ipv4Address id)
{
    return true;
}

std::string
PositionTable::GetNeighborList()
{
    Purge();  // Remove expired entries first
    
    std::ostringstream oss;
    oss << "[" << m_table.size() << " neighbors: ";
    
    bool first = true;
    for (auto& entry : m_table)
    {
        if (!first) oss << ", ";
        first = false;
        oss << entry.first << "(" << entry.second.position.x << "," << entry.second.position.y 
            << ",sinr=" << entry.second.sinr << ")";
    }
    oss << "]";
    
    return oss.str();
}

std::vector<NeighborSummary>
PositionTable::GetTopKNeighborSummaries(uint8_t k, Vector selfPos)
{
    NS_LOG_FUNCTION(this << k);
    
    // First purge stale entries
    Purge();
    
    // Collect all valid neighbors with their scores
    std::vector<std::pair<double, NeighborSummary>> scoredNeighbors;
    
    for (const auto& entry : m_table)
    {
        NeighborSummary ns;
        ns.ip = entry.first;
        ns.x = static_cast<float>(entry.second.position.x);
        ns.y = static_cast<float>(entry.second.position.y);
        
        // TODO: Add velocity when NeighborEntry is extended in Phase B
        ns.vx = 0.0f;
        ns.vy = 0.0f;
        
        // Map SINR to link quality (0-255)
        // Use log scale: SINR 0dB -> 0, SINR 30dB -> 255
        double sinrDb = (entry.second.sinr > 0) ? 10.0 * std::log10(entry.second.sinr) : -10.0;
        sinrDb = std::max(-10.0, std::min(30.0, sinrDb)); // Clamp to [-10, 30] dB
        ns.linkQuality = static_cast<uint8_t>((sinrDb + 10.0) / 40.0 * 255.0);
        
        // Score: prioritize by link quality (higher is better)
        double score = ns.linkQuality;
        
        scoredNeighbors.push_back({score, ns});
    }
    
    // Sort by score (descending)
    std::sort(scoredNeighbors.begin(), scoredNeighbors.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    
    // Return top K
    std::vector<NeighborSummary> result;
    result.reserve(std::min(static_cast<size_t>(k), scoredNeighbors.size()));
    for (size_t i = 0; i < std::min(static_cast<size_t>(k), scoredNeighbors.size()); ++i)
    {
        result.push_back(scoredNeighbors[i].second);
    }
    
    NS_LOG_DEBUG("GetTopKNeighborSummaries: returning " << result.size() << " of " << m_table.size() << " neighbors");
    return result;
}

Callback<void, WifiMacHeader const&>
PositionTable::GetTxErrorCallback() const
{
    return m_txErrorCallback;
}

} // namespace gpsr
} // namespace ns3
