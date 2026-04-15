/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "sumo-road-graph-loader.h"

#include "twin-environment.h"

#include "ns3/log.h"

#ifdef HAVE_LIBXML2
#include <libxml/xmlreader.h>
#endif

#include <cstdlib>
#include <string>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("SumoRoadGraphLoader");

namespace
{

#ifdef HAVE_LIBXML2
std::string
GetXmlAttribute(xmlTextReaderPtr reader, const char* name)
{
    xmlChar* value = xmlTextReaderGetAttribute(reader, BAD_CAST name);
    if (value == nullptr)
    {
        return "";
    }

    std::string result = reinterpret_cast<const char*>(value);
    xmlFree(value);
    return result;
}

double
ParseDouble(const std::string& value, double fallback = 0.0)
{
    if (value.empty())
    {
        return fallback;
    }

    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end == value.c_str())
    {
        return fallback;
    }

    return parsed;
}
#endif

} // namespace

bool
SumoRoadGraphLoader::LoadNetXml(const std::string& fileName, TwinEnvironment& twin)
{
    NS_LOG_FUNCTION(fileName);

#ifndef HAVE_LIBXML2
    NS_LOG_ERROR("SUMO road graph loading requires libxml2 support");
    return false;
#else
    xmlTextReaderPtr reader = xmlReaderForFile(fileName.c_str(), nullptr, 0);
    if (reader == nullptr)
    {
        NS_LOG_ERROR("Failed to open SUMO net.xml file: " << fileName);
        return false;
    }

    twin.ClearRoadGraph();

    int rc = xmlTextReaderRead(reader);
    while (rc == 1)
    {
        if (xmlTextReaderNodeType(reader) != XML_READER_TYPE_ELEMENT)
        {
            rc = xmlTextReaderRead(reader);
            continue;
        }

        const auto* nodeName = reinterpret_cast<const char*>(xmlTextReaderConstName(reader));
        if (nodeName == nullptr)
        {
            rc = xmlTextReaderRead(reader);
            continue;
        }

        const std::string elementName(nodeName);
        if (elementName == "junction")
        {
            TwinJunction junction;
            junction.junctionId = GetXmlAttribute(reader, "id");
            junction.position.x = ParseDouble(GetXmlAttribute(reader, "x"));
            junction.position.y = ParseDouble(GetXmlAttribute(reader, "y"));
            junction.type = GetXmlAttribute(reader, "type");

            const bool isInternalJunction =
                junction.type == "internal" ||
                (!junction.junctionId.empty() && junction.junctionId.front() == ':');

            if (!junction.junctionId.empty() && !isInternalJunction)
            {
                twin.AddJunction(junction);
            }
        }
        else if (elementName == "edge")
        {
            const std::string function = GetXmlAttribute(reader, "function");
            if (function == "internal")
            {
                rc = xmlTextReaderNext(reader);
                continue;
            }

            TwinRoadSegment road;
            road.roadId = GetXmlAttribute(reader, "id");
            road.fromJunctionId = GetXmlAttribute(reader, "from");
            road.toJunctionId = GetXmlAttribute(reader, "to");

            if (road.roadId.empty() || road.fromJunctionId.empty() || road.toJunctionId.empty())
            {
                rc = xmlTextReaderNext(reader);
                continue;
            }

            const int edgeDepth = xmlTextReaderDepth(reader);
            int childRc = xmlTextReaderRead(reader);
            while (childRc == 1)
            {
                const int childDepth = xmlTextReaderDepth(reader);
                if (childDepth <= edgeDepth &&
                    xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT)
                {
                    break;
                }

                if (childDepth == edgeDepth + 1 &&
                    xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
                {
                    const auto* childName =
                        reinterpret_cast<const char*>(xmlTextReaderConstName(reader));
                    if (childName != nullptr && std::string(childName) == "lane")
                    {
                        road.laneCount += 1;

                        const double laneLength =
                            ParseDouble(GetXmlAttribute(reader, "length"), road.lengthMeters);
                        if (laneLength > road.lengthMeters)
                        {
                            road.lengthMeters = laneLength;
                        }

                        const double laneSpeed = ParseDouble(GetXmlAttribute(reader, "speed"));
                        if (laneSpeed > road.maxSpeedMetersPerSecond)
                        {
                            road.maxSpeedMetersPerSecond = laneSpeed;
                        }
                    }
                }

                childRc = xmlTextReaderRead(reader);
            }

            twin.AddRoadSegment(road);
            rc = childRc;
            continue;
        }

        rc = xmlTextReaderRead(reader);
    }

    xmlFreeTextReader(reader);

    if (rc < 0)
    {
        NS_LOG_ERROR("Error while parsing SUMO net.xml file: " << fileName);
        return false;
    }

    return true;
#endif
}

} // namespace ns3
