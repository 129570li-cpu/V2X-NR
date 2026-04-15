/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "elite-controller.h"

#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("EliteController");
NS_OBJECT_ENSURE_REGISTERED(EliteController);

// ---------------------------------------------------------------------------
// Default fusion weights [msgType][objective index: PDR, DELAY, HOP, COST]
// These match the intuitive priorities described in the ELITE paper.
// ---------------------------------------------------------------------------
static const std::array<std::array<double, ELITE_OBJECTIVE_COUNT>, ELITE_MSG_TYPE_COUNT>
    kDefaultFusionWeights{{
        // SAFETY:    high PDR + low delay, hop/cost less important
        {0.60, 0.30, 0.05, 0.05},
        // DATA:      balanced
        {0.25, 0.25, 0.25, 0.25},
        // STREAMING: cost efficiency dominates
        {0.10, 0.15, 0.25, 0.50},
        // CONTROL:   delay + cost, reliability secondary
        {0.15, 0.40, 0.05, 0.40},
    }};

// ---------------------------------------------------------------------------
// TypeId
// ---------------------------------------------------------------------------

TypeId
EliteController::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::EliteController")
            .SetParent<Object>()
            .SetGroupName("Elite")
            .AddConstructor<EliteController>()
            .AddAttribute("CycleInterval",
                          "Interval between twin-sync and training cycles.",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&EliteController::m_cycleInterval),
                          MakeTimeChecker())
            .AddAttribute("EpisodesPerCycle",
                          "Number of training episodes per objective per cycle.",
                          UintegerValue(10),
                          MakeUintegerAccessor(&EliteController::m_episodesPerCycle),
                          MakeUintegerChecker<uint32_t>(1));
    return tid;
}

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

EliteController::EliteController()
    : m_twin(nullptr),
      m_cycleInterval(Seconds(1.0)),
      m_episodesPerCycle(10),
      m_running(false)
{
    // 初始化融合权重为默认值
    m_fusionWeights = kDefaultFusionWeights;

    // 为每个训练目标配置对应的 RewardModel 和 QTable
    const std::array<EliteTrainingObjective, ELITE_OBJECTIVE_COUNT> objectives{{
        EliteTrainingObjective::PDR,
        EliteTrainingObjective::DELAY,
        EliteTrainingObjective::HOP_COUNT,
        EliteTrainingObjective::ROUTING_COST,
    }};

    for (uint8_t i = 0; i < ELITE_OBJECTIVE_COUNT; ++i)
    {
        m_rewardModels[i].SetObjective(objectives[i]);
        m_trainers[i].SetQTable(&m_qTables[i]);
        m_trainers[i].SetRewardModel(&m_rewardModels[i]);
        // TwinEnvironment 在 Start() 中设置，此处暂不配置
    }
}

EliteController::~EliteController()
{
    Stop();
}

// ---------------------------------------------------------------------------
// Configuration setters / getters
// ---------------------------------------------------------------------------

void
EliteController::SetTwinEnvironment(Ptr<TwinEnvironment> twin)
{
    m_twin = twin;
    for (auto& trainer : m_trainers)
    {
        trainer.SetEnvironment(PeekPointer(twin));
    }
}

Ptr<TwinEnvironment>
EliteController::GetTwinEnvironment() const
{
    return m_twin;
}

void
EliteController::SetCycleInterval(Time interval)
{
    m_cycleInterval = interval;
}

Time
EliteController::GetCycleInterval() const
{
    return m_cycleInterval;
}

void
EliteController::SetEpisodesPerCycle(uint32_t episodes)
{
    m_episodesPerCycle = std::max(1u, episodes);
}

uint32_t
EliteController::GetEpisodesPerCycle() const
{
    return m_episodesPerCycle;
}

void
EliteController::SetFusionWeights(EliteMessageType type,
                                   const std::array<double, ELITE_OBJECTIVE_COUNT>& weights)
{
    const auto idx = static_cast<uint8_t>(type);
    if (idx >= ELITE_MSG_TYPE_COUNT)
    {
        NS_LOG_WARN("SetFusionWeights: invalid message type index " << static_cast<int>(idx));
        return;
    }

    // 归一化，使权重之和为 1.0
    double total = 0.0;
    for (double w : weights)
    {
        total += w;
    }
    if (total <= 0.0)
    {
        NS_LOG_WARN("SetFusionWeights: all weights are zero; ignoring.");
        return;
    }

    for (uint8_t j = 0; j < ELITE_OBJECTIVE_COUNT; ++j)
    {
        m_fusionWeights[idx][j] = weights[j] / total;
    }
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void
EliteController::Start()
{
    if (m_twin == nullptr)
    {
        NS_LOG_ERROR("EliteController::Start() called without a TwinEnvironment.");
        return;
    }
    m_running = true;
    NS_LOG_INFO("EliteController started; cycle interval = " << m_cycleInterval.GetSeconds()
                                                              << "s");
    // 立刻触发第一轮
    ScheduleNextCycle();
}

void
EliteController::Stop()
{
    m_running = false;
}

// ---------------------------------------------------------------------------
// Training cycle
// ---------------------------------------------------------------------------

void
EliteController::ScheduleNextCycle()
{
    if (!m_running)
    {
        return;
    }
    Simulator::Schedule(m_cycleInterval, &EliteController::OnCycleTick, this);
}

void
EliteController::OnCycleTick()
{
    SyncPhysicalToTwin();
    RunTrainingCycle();
    ScheduleNextCycle();
}

void
EliteController::SyncPhysicalToTwin()
{
    if (m_twin == nullptr)
    {
        return;
    }

    // 遍历仿真中的所有节点，将移动模型信息写入孪生环境
    for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i)
    {
        Ptr<Node> node = NodeList::GetNode(i);
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
        if (mob == nullptr)
        {
            continue; // 没有移动模型的节点（如基站）跳过
        }

        TwinVehicleState state;
        state.vehicleId = static_cast<uint64_t>(node->GetId());
        state.position  = mob->GetPosition();
        state.velocity  = mob->GetVelocity();
        state.lastUpdate = Simulator::Now();

        m_twin->UpdateVehicleState(state);
    }

    NS_LOG_DEBUG("SyncPhysicalToTwin: synced " << NodeList::GetNNodes() << " nodes at t="
                                                << Simulator::Now().GetSeconds() << "s");
}

void
EliteController::RunTrainingCycle()
{
    if (m_twin == nullptr)
    {
        return;
    }

    // 收集所有可用的路口 ID 用于随机采样 Source-Destination 对
    const auto& junctions = m_twin->GetJunctions();
    if (junctions.size() < 2)
    {
        NS_LOG_WARN("RunTrainingCycle: fewer than 2 junctions in twin; skipping training.");
        return;
    }

    std::vector<std::string> junctionIds;
    junctionIds.reserve(junctions.size());
    for (const auto& kv : junctions)
    {
        junctionIds.push_back(kv.first);
    }

    // 每轮训练，对每个目标跑 m_episodesPerCycle 个随机 src-dst 对
    auto rv = CreateObject<UniformRandomVariable>();
    const Time now = Simulator::Now();

    for (uint8_t obj = 0; obj < ELITE_OBJECTIVE_COUNT; ++obj)
    {
        for (uint32_t ep = 0; ep < m_episodesPerCycle; ++ep)
        {
            // 随机选 src != dst
            const uint32_t si = rv->GetInteger(0, junctionIds.size() - 1);
            uint32_t di = rv->GetInteger(0, junctionIds.size() - 2);
            if (di >= si)
            {
                ++di; // 保证 di != si
            }

            m_trainers[obj].TrainEpisode(junctionIds[si], junctionIds[di], now);
        }
    }

    NS_LOG_DEBUG("RunTrainingCycle: objective[" << ELITE_OBJECTIVE_COUNT
                                                 << "] x " << m_episodesPerCycle
                                                 << " episodes completed.");
}

// ---------------------------------------------------------------------------
// Routing interface
// ---------------------------------------------------------------------------

std::string
EliteController::VehicleToJunction(uint64_t vehicleId) const
{
    if (m_twin == nullptr)
    {
        return "";
    }

    const TwinVehicleState* vstate = m_twin->GetVehicleState(vehicleId);
    if (vstate == nullptr)
    {
        return "";
    }

    // 在孪生路口表中找到离该车辆位置最近的路口
    const auto& junctions = m_twin->GetJunctions();
    std::string closestId;
    double closestDist = std::numeric_limits<double>::max();

    for (const auto& kv : junctions)
    {
        const double dx = kv.second.position.x - vstate->position.x;
        const double dy = kv.second.position.y - vstate->position.y;
        const double dist = dx * dx + dy * dy; // 不开方，只比较大小

        if (dist < closestDist)
        {
            closestDist = dist;
            closestId = kv.first;
        }
    }

    return closestId;
}

std::vector<std::string>
EliteController::RequestPath(uint64_t srcVehicleId,
                              uint64_t dstVehicleId,
                              EliteMessageType type)
{
    const std::string srcJunction = VehicleToJunction(srcVehicleId);
    const std::string dstJunction = VehicleToJunction(dstVehicleId);

    if (srcJunction.empty() || dstJunction.empty())
    {
        NS_LOG_WARN("RequestPath: cannot resolve vehicle IDs to junctions ("
                    << srcVehicleId << " -> " << dstVehicleId << ")");
        return {};
    }

    return RequestPathByJunction(srcJunction, dstJunction, type);
}

std::vector<std::string>
EliteController::RequestPathByJunction(const std::string& srcJunction,
                                        const std::string& dstJunction,
                                        EliteMessageType type)
{
    if (srcJunction == dstJunction)
    {
        return {srcJunction};
    }

    // 融合 4 张 Q 表，得到加权合成 Q 表
    const EliteQTable fused = BuildFusedQTable(type);

    // 基于合成 Q 表贪婪地走出一条路口序列
    const std::vector<std::string> path = WalkFusedTable(fused, srcJunction, dstJunction);

    NS_LOG_INFO("RequestPathByJunction: " << srcJunction << " -> " << dstJunction
                                           << " | hops=" << path.size());
    return path;
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

EliteQTable
EliteController::BuildFusedQTable(EliteMessageType type) const
{
    const auto idx = static_cast<uint8_t>(type);
    const auto& weights = m_fusionWeights[idx];

    EliteQTable fused;

    // 遍历第一个 Q 表里所有已知的 (dst, cur, next) 三元组，
    // 把 4 个目标对应的 Q 值加权求和
    // 由于 EliteQTable 内部是 unordered_map，没有公开迭代接口，
    // 我们通过"问每个表同一个 key"的方式来融合
    // 这里需要知道所有路口对：从 TwinEnvironment 的邻接表遍历
    if (m_twin == nullptr)
    {
        return fused;
    }

    const auto& adjacency = m_twin->GetAdjacency();
    const auto& junctions = m_twin->GetJunctions();

    for (const auto& dstKv : junctions)
    {
        const std::string& dst = dstKv.first;
        for (const auto& curKv : adjacency)
        {
            const std::string& cur = curKv.first;
            for (const std::string& next : curKv.second)
            {
                double fusedQ = 0.0;
                for (uint8_t obj = 0; obj < ELITE_OBJECTIVE_COUNT; ++obj)
                {
                    fusedQ += weights[obj] * m_qTables[obj].GetQ(dst, cur, next);
                }
                fused.SetQ(dst, cur, next, fusedQ);
            }
        }
    }

    return fused;
}

std::vector<std::string>
EliteController::WalkFusedTable(const EliteQTable& fused,
                                 const std::string& src,
                                 const std::string& dst) const
{
    std::vector<std::string> path;
    path.push_back(src);

    // 防止死循环：最多走路口数量步
    const uint32_t maxSteps = m_twin ? static_cast<uint32_t>(m_twin->GetJunctions().size()) : 64;
    std::string current = src;

    for (uint32_t step = 0; step < maxSteps && current != dst; ++step)
    {
        const auto* candidates = m_twin->GetAdjacentJunctions(current);
        if (candidates == nullptr || candidates->empty())
        {
            NS_LOG_WARN("WalkFusedTable: no adjacent junctions from " << current);
            break;
        }

        // 如果目的地就是当前节点的邻居，直接跳过去
        bool directFound = false;
        for (const auto& c : *candidates)
        {
            if (c == dst)
            {
                path.push_back(dst);
                directFound = true;
                break;
            }
        }
        if (directFound)
        {
            break;
        }

        // 否则从合成 Q 表里选最优下一跳
        const std::string next = fused.GetBestAction(dst, current, *candidates);
        if (next.empty())
        {
            NS_LOG_WARN("WalkFusedTable: GetBestAction returned empty at " << current);
            break;
        }

        // 检测环路
        if (std::find(path.begin(), path.end(), next) != path.end())
        {
            NS_LOG_WARN("WalkFusedTable: loop detected at " << next << "; aborting");
            break;
        }

        path.push_back(next);
        current = next;
    }

    // 如果没走到目的地，返回空路径（表示找不到）
    if (path.empty() || path.back() != dst)
    {
        NS_LOG_WARN("WalkFusedTable: could not reach " << dst << " from " << src);
        return {};
    }

    return path;
}

} // namespace ns3
