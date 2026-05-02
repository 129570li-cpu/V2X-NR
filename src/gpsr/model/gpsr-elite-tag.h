/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * EliteRoutePathTag — carries a junction coordinate sequence inside a packet
 * so that GPSR can perform restricted greedy forwarding along the ELITE path.
 */

#ifndef GPSR_ELITE_TAG_H
#define GPSR_ELITE_TAG_H

#include "ns3/tag.h"
#include "ns3/vector.h"

#include <cstdint>
#include <vector>

namespace ns3
{
namespace gpsr
{

/**
 * \ingroup gpsr
 * \brief Tag that carries an ELITE junction-level path inside a packet.
 *
 * The tag stores a sequence of junction coordinates and an index pointing
 * to the "current target junction".  Intermediate nodes read the tag,
 * forward greedily toward the current target junction instead of the
 * final destination, and advance the index once they are close enough.
 *
 * Serialisation stores up to MAX_JUNCTIONS waypoints (x, y as doubles)
 * plus the current index byte.
 */
class EliteRoutePathTag : public Tag
{
  public:
    /// Maximum number of junctions that can be stored in a tag.
    static constexpr uint8_t MAX_JUNCTIONS = 16;

    EliteRoutePathTag();

    /**
     * \brief Construct from a junction coordinate path.
     * \param path ordered junction positions (src → dst)
     */
    explicit EliteRoutePathTag(const std::vector<Vector>& path);

    // ------ ns-3 Tag interface ------
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;

    // ------ Accessors ------

    /**
     * \brief Get the full junction path.
     */
    const std::vector<Vector>& GetPath() const;

    /**
     * \brief Get the number of junctions in the path.
     */
    uint8_t GetPathSize() const;

    /**
     * \brief Get the current waypoint index.
     */
    uint8_t GetCurrentIndex() const;

    /**
     * \brief Set the current waypoint index.
     */
    void SetCurrentIndex(uint8_t index);

    /**
     * \brief Get the current target junction coordinate.
     *
     * Returns (0,0,0) if the index is out of range.
     */
    Vector GetCurrentTarget() const;

    /**
     * \brief Advance to the next junction in the path.
     * \return true if successfully advanced, false if already at the end
     */
    bool AdvanceToNext();

    /**
     * \brief Check whether the path has been fully traversed.
     */
    bool IsFinished() const;

  private:
    std::vector<Vector> m_path; ///< Ordered junction coordinates
    uint8_t m_currentIndex;     ///< Index of the current target junction
};

} // namespace gpsr
} // namespace ns3

#endif /* GPSR_ELITE_TAG_H */
