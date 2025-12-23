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
        return i->second.second;
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
        m_table.erase(id);
    }
    m_table.insert(std::make_pair(id, std::make_pair(position, Simulator::Now())));
}

void
PositionTable::DeleteEntry(Ipv4Address id)
{
    NS_LOG_FUNCTION(this << id);
    m_table.erase(id);
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

    Purge();

    double initialDistance = CalculateDistance(nodePos, dstPosition);

    if (m_table.empty())
    {
        NS_LOG_DEBUG("BestNeighbor table is empty; Position: " << dstPosition);
        return Ipv4Address::GetZero();
    }

    Ipv4Address bestFoundID = m_table.begin()->first;
    double bestFoundDistance = CalculateDistance(m_table.begin()->second.first, dstPosition);

    for (auto i = m_table.begin(); i != m_table.end(); ++i)
    {
        double distance = CalculateDistance(i->second.first, dstPosition);
        if (bestFoundDistance > distance)
        {
            bestFoundID = i->first;
            bestFoundDistance = distance;
        }
    }

    // Only return neighbor if it's closer to destination than we are
    if (initialDistance > bestFoundDistance)
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
        tmpAngle = GetAngle(nodePos, previousHop, i->second.first);
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

Callback<void, WifiMacHeader const&>
PositionTable::GetTxErrorCallback() const
{
    return m_txErrorCallback;
}

} // namespace gpsr
} // namespace ns3
