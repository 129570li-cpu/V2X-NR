/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "gpsr-elite-tag.h"

#include "ns3/log.h"

#include <algorithm>

namespace ns3
{
namespace gpsr
{

NS_LOG_COMPONENT_DEFINE("GpsrEliteTag");

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

EliteRoutePathTag::EliteRoutePathTag()
    : m_currentIndex(0)
{
}

EliteRoutePathTag::EliteRoutePathTag(const std::vector<Vector>& path)
    : m_currentIndex(0)
{
    // Truncate to MAX_JUNCTIONS if necessary
    const uint8_t count = static_cast<uint8_t>(
        std::min(static_cast<size_t>(MAX_JUNCTIONS), path.size()));
    m_path.assign(path.begin(), path.begin() + count);
}

// ---------------------------------------------------------------------------
// ns-3 Tag interface
// ---------------------------------------------------------------------------

TypeId
EliteRoutePathTag::GetTypeId()
{
    static TypeId tid = TypeId("ns3::gpsr::EliteRoutePathTag")
                            .SetParent<Tag>()
                            .AddConstructor<EliteRoutePathTag>();
    return tid;
}

TypeId
EliteRoutePathTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
EliteRoutePathTag::GetSerializedSize() const
{
    // 1 byte: path size
    // 1 byte: current index
    // N * 2 * 8 bytes: x, y for each junction (double = 8 bytes)
    return 1 + 1 + static_cast<uint32_t>(m_path.size()) * 2 * sizeof(double);
}

void
EliteRoutePathTag::Serialize(TagBuffer i) const
{
    const uint8_t count = static_cast<uint8_t>(m_path.size());
    i.WriteU8(count);
    i.WriteU8(m_currentIndex);
    for (uint8_t j = 0; j < count; ++j)
    {
        i.WriteDouble(m_path[j].x);
        i.WriteDouble(m_path[j].y);
    }
}

void
EliteRoutePathTag::Deserialize(TagBuffer i)
{
    const uint8_t count = i.ReadU8();
    m_currentIndex = i.ReadU8();
    m_path.resize(count);
    for (uint8_t j = 0; j < count; ++j)
    {
        m_path[j].x = i.ReadDouble();
        m_path[j].y = i.ReadDouble();
        m_path[j].z = 0.0;
    }
}

void
EliteRoutePathTag::Print(std::ostream& os) const
{
    os << "EliteRoutePathTag: size=" << (int)m_path.size()
       << " idx=" << (int)m_currentIndex;
    if (!m_path.empty() && m_currentIndex < m_path.size())
    {
        os << " target=(" << m_path[m_currentIndex].x << ","
           << m_path[m_currentIndex].y << ")";
    }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

const std::vector<Vector>&
EliteRoutePathTag::GetPath() const
{
    return m_path;
}

uint8_t
EliteRoutePathTag::GetPathSize() const
{
    return static_cast<uint8_t>(m_path.size());
}

uint8_t
EliteRoutePathTag::GetCurrentIndex() const
{
    return m_currentIndex;
}

void
EliteRoutePathTag::SetCurrentIndex(uint8_t index)
{
    m_currentIndex = index;
}

Vector
EliteRoutePathTag::GetCurrentTarget() const
{
    if (m_currentIndex < m_path.size())
    {
        return m_path[m_currentIndex];
    }
    return Vector(0.0, 0.0, 0.0);
}

bool
EliteRoutePathTag::AdvanceToNext()
{
    if (m_currentIndex + 1 < static_cast<uint8_t>(m_path.size()))
    {
        ++m_currentIndex;
        return true;
    }
    return false;
}

bool
EliteRoutePathTag::IsFinished() const
{
    return m_path.empty() || m_currentIndex >= m_path.size();
}

} // namespace gpsr
} // namespace ns3
