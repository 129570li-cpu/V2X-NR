/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 University of Washington
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "location-service.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LocationService");

NS_OBJECT_ENSURE_REGISTERED(LocationService);

TypeId
LocationService::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LocationService")
                            .SetParent<Object>()
                            .SetGroupName("LocationService");
    return tid;
}

LocationService::LocationService()
{
    NS_LOG_FUNCTION(this);
}

LocationService::~LocationService()
{
    NS_LOG_FUNCTION(this);
}

} // namespace ns3
