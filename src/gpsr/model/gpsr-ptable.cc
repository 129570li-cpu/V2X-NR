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
        // Update raw SINR
        i->second.sinr = sinr;
        i->second.lastSinrUpdate = Simulator::Now();
        
        // Apply EWMA for smoothed SINR (alpha = 0.3)
        const double EWMA_ALPHA = 0.3;
        if (sinr > 0.0)
        {
            if (i->second.smoothedSinr < 0.0)
            {
                // First valid measurement - initialize directly
                i->second.smoothedSinr = sinr;
            }
            else
            {
                // EWMA: smoothed = alpha * new + (1-alpha) * old
                i->second.smoothedSinr = EWMA_ALPHA * sinr + 
                                         (1.0 - EWMA_ALPHA) * i->second.smoothedSinr;
            }
            NS_LOG_DEBUG("Updated SINR for " << id << " raw=" << sinr 
                         << " smoothed=" << i->second.smoothedSinr
                         << " (" << 10*std::log10(i->second.smoothedSinr) << " dB)");
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

Ipv4Address
PositionTable::BestNeighborTwoHop(Vector dstPosition, Vector nodePos, Vector nodeVel)
{
    NS_LOG_FUNCTION(this << dstPosition << nodePos << nodeVel);

    // Single Purge at the beginning
    Purge();

    double initialDistance = CalculateDistance(nodePos, dstPosition);

    if (m_table.empty())
    {
        NS_LOG_DEBUG("BestNeighborTwoHop: table is empty");
        return Ipv4Address::GetZero();
    }

    Ipv4Address bestNeighbor = Ipv4Address::GetZero();
    double bestScore = -1.0;
    double bestDistance = std::numeric_limits<double>::max();

    // Score all neighbors that make forward progress
    for (const auto& entry : m_table)
    {
        double distance = CalculateDistance(entry.second.position, dstPosition);
        
        // Only consider neighbors that make progress toward destination
        if (distance >= initialDistance)
        {
            continue;
        }

        // Calculate composite score using CalculateTwoHopScore
        double score = CalculateTwoHopScore(entry.first, dstPosition, nodePos, nodeVel);
        
        if (score < 0)
        {
            // Invalid score, skip
            continue;
        }

        // Select neighbor with highest score
        // If scores are equal, prefer the one closer to destination
        if (score > bestScore || (score == bestScore && distance < bestDistance))
        {
            bestScore = score;
            bestNeighbor = entry.first;
            bestDistance = distance;
        }
    }

    if (bestNeighbor != Ipv4Address::GetZero())
    {
        NS_LOG_DEBUG("BestNeighborTwoHop: selected " << bestNeighbor 
                     << " score=" << bestScore << " dist=" << bestDistance);
        return bestNeighbor;
    }

    // Fallback: No neighbor passed the scoring, try standard greedy as last resort
    NS_LOG_DEBUG("BestNeighborTwoHop: no scored neighbor, falling back to greedy");
    return BestNeighbor(dstPosition, nodePos);
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
        
        // Use stored velocity from NeighborEntry
        ns.vx = static_cast<float>(entry.second.velocity.x);
        ns.vy = static_cast<float>(entry.second.velocity.y);
        
        // Map SINR to link quality (0-255)
        // Prefer smoothedSinr for stability, fall back to raw sinr
        double sinrValue = (entry.second.smoothedSinr > 0) ? entry.second.smoothedSinr : entry.second.sinr;
        // Use log scale: SINR 0dB -> 0, SINR 30dB -> 255
        double sinrDb = (sinrValue > 0) ? 10.0 * std::log10(sinrValue) : -10.0;
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

double
PositionTable::CalculateLinkDuration(Vector pos1, Vector vel1, 
                                      Vector pos2, Vector vel2, 
                                      double commRange)
{
    // Relative position and velocity
    double rx = pos2.x - pos1.x;
    double ry = pos2.y - pos1.y;
    double vx = vel2.x - vel1.x;
    double vy = vel2.y - vel1.y;
    
    // Current distance
    double currentDist = std::sqrt(rx*rx + ry*ry);
    
    // If already out of range, return 0
    if (currentDist >= commRange)
    {
        return 0.0;
    }
    
    // Relative speed squared
    double v2 = vx*vx + vy*vy;
    
    // If relative velocity is near zero, link is stable
    if (v2 < 1e-6)
    {
        return 1e6; // Very large value = stable link
    }
    
    // Solve quadratic: |r + v*t|^2 = R^2
    // (rx + vx*t)^2 + (ry + vy*t)^2 = R^2
    // v2*t^2 + 2*(rx*vx + ry*vy)*t + (rx^2 + ry^2 - R^2) = 0
    double a = v2;
    double b = 2.0 * (rx*vx + ry*vy);
    double c = rx*rx + ry*ry - commRange*commRange;
    
    double discriminant = b*b - 4*a*c;
    
    if (discriminant < 0)
    {
        // No real solution - link stays within range
        return 1e6;
    }
    
    double sqrtD = std::sqrt(discriminant);
    double t1 = (-b + sqrtD) / (2*a);
    double t2 = (-b - sqrtD) / (2*a);
    
    // We want the smallest positive t (when link breaks)
    double tBreak = 1e6;
    if (t1 > 0 && t1 < tBreak) tBreak = t1;
    if (t2 > 0 && t2 < tBreak) tBreak = t2;
    
    return tBreak;
}

double
PositionTable::CalculateTwoHopScore(Ipv4Address neighborId,
                                     Vector dstPos,
                                     Vector selfPos,
                                     Vector selfVel)
{
    NS_LOG_FUNCTION(this << neighborId << dstPos);
    
    // Find the neighbor
    auto it = m_table.find(neighborId);
    if (it == m_table.end())
    {
        return -1.0; // Neighbor not found
    }
    
    const NeighborEntry& neighbor = it->second;
    
    // Weight factors (tunable)
    const double W_PROGRESS = 0.4;
    const double W_QUALITY = 0.3;
    const double W_DURATION = 0.3;
    const double COMM_RANGE = 300.0; // meters (configurable)
    const double TWO_HOP_DECAY = 0.9; // Penalty for 2-hop paths
    
    double selfToDst = CalculateDistance(selfPos, dstPos);
    
    // ========== One-Hop Path Evaluation ==========
    double neighborToDst = CalculateDistance(neighbor.position, dstPos);
    double progress1Hop = selfToDst - neighborToDst;
    double progress1HopNorm = std::max(0.0, progress1Hop / COMM_RANGE);
    
    // Link Quality for 1-hop
    double sinr1Hop = (neighbor.smoothedSinr > 0) ? neighbor.smoothedSinr : neighbor.sinr;
    double quality1HopNorm = 0.0;
    if (sinr1Hop > 0)
    {
        double sinrDb = 10.0 * std::log10(sinr1Hop);
        quality1HopNorm = std::max(0.0, std::min(1.0, (sinrDb + 10.0) / 40.0));
    }
    
    // Link Duration for 1-hop
    double duration1Hop = CalculateLinkDuration(selfPos, selfVel, 
                                                neighbor.position, neighbor.velocity,
                                                COMM_RANGE);
    double duration1HopNorm = std::min(1.0, duration1Hop / 10.0);
    
    double score1Hop = W_PROGRESS * progress1HopNorm + 
                       W_QUALITY * quality1HopNorm + 
                       W_DURATION * duration1HopNorm;
    
    // ========== Two-Hop Path Evaluation ==========
    double bestScore2Hop = -1.0;
    Ipv4Address bestTwoHopId;
    
    for (const auto& twoHop : neighbor.twoHopNeighbors)
    {
        // Note: We skip checking if 2-hop is self since the neighbor wouldn't include us in its list
        
        Vector twoHopPos(twoHop.x, twoHop.y, 0.0);
        double twoHopToDst = CalculateDistance(twoHopPos, dstPos);
        
        // Progress via 2-hop: how much closer does the 2-hop neighbor get to dst
        double progress2Hop = selfToDst - twoHopToDst;
        double progress2HopNorm = std::max(0.0, progress2Hop / COMM_RANGE);
        
        // Quality: bottleneck of the two links
        // First link: self -> neighbor (quality1HopNorm)
        // Second link: neighbor -> 2hop (use twoHop.linkQuality, 0-255)
        double quality2ndLink = twoHop.linkQuality / 255.0;
        double quality2HopNorm = std::min(quality1HopNorm, quality2ndLink);
        
        // Duration: bottleneck of two links using twoHop.vx/vy for second hop prediction
        Vector twoHopVel(twoHop.vx, twoHop.vy, 0.0);
        double duration2ndHop = CalculateLinkDuration(neighbor.position, neighbor.velocity,
                                                       twoHopPos, twoHopVel,
                                                       COMM_RANGE);
        double duration2ndHopNorm = std::min(1.0, duration2ndHop / 10.0);
        // Bottleneck: use minimum of both link durations
        double duration2HopNorm = std::min(duration1HopNorm, duration2ndHopNorm);
        
        double score2Hop = TWO_HOP_DECAY * (W_PROGRESS * progress2HopNorm + 
                                            W_QUALITY * quality2HopNorm + 
                                            W_DURATION * duration2HopNorm);
        
        if (score2Hop > bestScore2Hop)
        {
            bestScore2Hop = score2Hop;
            bestTwoHopId = twoHop.ip;
        }
    }
    
    // ========== Final Score: max of 1-hop and best 2-hop ==========
    double finalScore = score1Hop;
    if (bestScore2Hop > score1Hop)
    {
        finalScore = bestScore2Hop;
        NS_LOG_DEBUG("TwoHopScore for " << neighborId << ": 2-hop via " << bestTwoHopId
                     << " score=" << bestScore2Hop << " > 1hop=" << score1Hop);
    }
    else
    {
        NS_LOG_DEBUG("TwoHopScore for " << neighborId << ": 1-hop score=" << score1Hop
                     << " (best 2hop=" << bestScore2Hop << ")");
    }
    
    return finalScore;
}

} // namespace gpsr
} // namespace ns3
