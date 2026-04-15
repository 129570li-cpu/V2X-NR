/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef SUMO_ROAD_GRAPH_LOADER_H
#define SUMO_ROAD_GRAPH_LOADER_H

#include <string>

namespace ns3
{

class TwinEnvironment;

/**
 * \ingroup elite
 * \brief Load a SUMO net.xml road graph into a TwinEnvironment.
 */
class SumoRoadGraphLoader
{
  public:
    /**
     * \brief Parse a SUMO net.xml file and populate the twin road graph.
     * \param fileName path to the SUMO net.xml file
     * \param twin twin environment receiving the graph
     * \return true on success
     */
    static bool LoadNetXml(const std::string& fileName, TwinEnvironment& twin);
};

} // namespace ns3

#endif /* SUMO_ROAD_GRAPH_LOADER_H */
