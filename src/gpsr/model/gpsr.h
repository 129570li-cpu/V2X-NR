/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * GPSR Routing Protocol Header - Complete Implementation
 */

#ifndef GPSR_H
#define GPSR_H

#include "gpsr-packet.h"
#include "gpsr-ptable.h"
#include "gpsr-rqueue.h"

#include "ns3/god.h"
#include "ns3/ip-l4-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/location-service.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/tag.h"
#include "ns3/timer.h"

#include <map>

namespace ns3
{
namespace gpsr
{

/**
 * \ingroup gpsr
 * \brief Tag used for deferred route output
 */
struct DeferredRouteOutputTag : public Tag
{
    uint32_t m_isCallFromL3;

    DeferredRouteOutputTag()
        : Tag(),
          m_isCallFromL3(0)
    {
    }

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;
};

/**
 * \ingroup gpsr
 * \brief Tag to mark packets that have GPSR PositionHeader added
 *
 * This tag is used to safely identify packets that have been processed
 * by GPSR and have PositionHeader attached, distinguishing them from
 * other packets (like HELLO messages or non-GPSR traffic).
 */
struct GpsrDataPacketTag : public Tag
{
    GpsrDataPacketTag()
        : Tag()
    {
    }

    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;
};

/**
 * \ingroup gpsr
 * \brief GPSR routing protocol - Complete Implementation
 */
class RoutingProtocol : public Ipv4RoutingProtocol
{
  public:
    static TypeId GetTypeId();
    static const uint32_t GPSR_PORT;

    RoutingProtocol();
    virtual ~RoutingProtocol();
    virtual void DoDispose() override;

    // From Ipv4RoutingProtocol
    Ptr<Ipv4Route> RouteOutput(Ptr<Packet> p,
                               const Ipv4Header& header,
                               Ptr<NetDevice> oif,
                               Socket::SocketErrno& sockerr) override;

    bool RouteInput(Ptr<const Packet> p,
                    const Ipv4Header& header,
                    Ptr<const NetDevice> idev,
                    const UnicastForwardCallback& ucb,
                    const MulticastForwardCallback& mcb,
                    const LocalDeliverCallback& lcb,
                    const ErrorCallback& ecb) override;

    virtual void NotifyInterfaceUp(uint32_t interface) override;
    virtual void NotifyInterfaceDown(uint32_t interface) override;
    virtual void NotifyAddAddress(uint32_t interface, Ipv4InterfaceAddress address) override;
    virtual void NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address) override;
    virtual void SetIpv4(Ptr<Ipv4> ipv4) override;
    virtual void PrintRoutingTable(Ptr<OutputStreamWrapper> stream,
                                   Time::Unit unit = Time::S) const override;

    /**
     * \brief Get the location service
     */
    Ptr<LocationService> GetLS();

    /**
     * \brief Set the location service
     */
    void SetLS(Ptr<LocationService> locationService);

    /// Add GPSR headers to packet and send via m_downTarget
    /// This must be public for the callback mechanism to work
    void AddHeaders(Ptr<Packet> p,
                    Ipv4Address source,
                    Ipv4Address destination,
                    uint8_t protocol,
                    Ptr<Ipv4Route> route);

    /**
     * \brief Get pointer to the position table for external SINR updates
     * \return Pointer to the internal PositionTable
     */
    PositionTable* GetPositionTable() { return &m_neighbors; }

  private:
    /// Start protocol operation
    void Start();

    /// Send HELLO message
    void SendHello();

    /// HELLO timer expiration handler
    void HelloTimerExpire();

    /// Receive GPSR control packet
    void RecvGpsr(Ptr<Socket> socket);

    /// Update route to neighbor
    void UpdateRouteToNeighbor(Ipv4Address sender, Ipv4Address receiver, Vector pos);

    /// Check if address belongs to this node
    bool IsMyOwnAddress(Ipv4Address src);

    /// Queue packet for deferred forwarding
    void DeferredRouteOutput(Ptr<const Packet> p,
                             const Ipv4Header& header,
                             UnicastForwardCallback ucb,
                             ErrorCallback ecb);

    /// Forward packet using GPSR algorithm
    bool Forwarding(Ptr<const Packet> p,
                    const Ipv4Header& header,
                    const UnicastForwardCallback& ucb,
                    const ErrorCallback& ecb);

    /// Send packet from queue
    bool SendPacketFromQueue(Ipv4Address dst);

    /// Check queue for pending packets
    void CheckQueue();

    /// Recovery mode forwarding (perimeter mode with right-hand rule)
    void RecoveryMode(Ipv4Address dst,
                      Ptr<Packet> p,
                      const UnicastForwardCallback& ucb,
                      Ipv4Header header);

    /// Create loopback route
    Ptr<Ipv4Route> LoopbackRoute(const Ipv4Header& header, Ptr<NetDevice> oif);

    /// Find socket with interface address
    Ptr<Socket> FindSocketWithInterfaceAddress(Ipv4InterfaceAddress iface) const;

    Ptr<Ipv4> m_ipv4;
    std::map<Ptr<Socket>, Ipv4InterfaceAddress> m_socketAddresses;
    Ptr<NetDevice> m_lo;

    Time m_helloInterval;
    uint32_t m_maxQueueLen;
    Time m_maxQueueTime;
    RequestQueue m_queue;

    Timer m_helloIntervalTimer;
    Timer m_checkQueueTimer;

    PositionTable m_neighbors;
    bool m_perimeterMode;

    std::list<Ipv4Address> m_queuedAddresses;
    Ptr<LocationService> m_locationService;

    uint8_t m_locationServiceName;

    /// IP layer down target callback for sending packets
    IpL4Protocol::DownTargetCallback m_downTarget;

  public:
    /// Set the down target callback
    void SetDownTarget(IpL4Protocol::DownTargetCallback callback);
    /// Get the down target callback
    IpL4Protocol::DownTargetCallback GetDownTarget() const;
};

} // namespace gpsr
} // namespace ns3

#endif /* GPSR_H */
