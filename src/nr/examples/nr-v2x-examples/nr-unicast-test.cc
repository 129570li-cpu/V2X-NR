/* nr-unicast-test.cc - 测试 NR Sidelink 单播功能
 * 
 * 目标: 验证 LteSlTft::CommType::Uincast 是否可以正常工作
 * 基于 cttc-nr-v2x-demo-simple.cc 简化
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/nr-module.h"
#include "ns3/lte-module.h"
#include "ns3/antenna-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrUnicastTest");

// 发送计数
static uint32_t g_txPackets = 0;
static uint32_t g_rxPackets = 0;

void TxCallback(Ptr<const Packet> pkt)
{
  g_txPackets++;
  NS_LOG_UNCOND("[TX] t=" << Simulator::Now().GetSeconds() << " size=" << pkt->GetSize());
}

void RxCallback(Ptr<const Packet> pkt, const Address& addr)
{
  g_rxPackets++;
  NS_LOG_UNCOND("[RX] t=" << Simulator::Now().GetSeconds() << " size=" << pkt->GetSize() 
                << " from=" << InetSocketAddress::ConvertFrom(addr).GetIpv4());
}

int main(int argc, char* argv[])
{
  double simTime = 10.0;
  bool useUnicast = true;
  
  CommandLine cmd(__FILE__);
  cmd.AddValue("sim-time", "Simulation time [s]", simTime);
  cmd.AddValue("unicast", "Use unicast (true) or groupcast (false)", useUnicast);
  cmd.Parse(argc, argv);
  
  // 日志
  LogComponentEnable("NrUnicastTest", LOG_LEVEL_INFO);
  
  NS_LOG_UNCOND("[NrUnicastTest] Mode=" << (useUnicast ? "UNICAST" : "GROUPCAST"));
  
  // NR 配置
  Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));
  
  // 创建 2 个节点: Node0 (发送) -> Node1 (接收)
  NodeContainer ueNodes;
  ueNodes.Create(2);
  
  // 移动性: 两个节点相距 50m
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
  posAlloc->Add(Vector(0, 0, 1.5));    // Node0
  posAlloc->Add(Vector(50, 0, 1.5));   // Node1
  mobility.SetPositionAllocator(posAlloc);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(ueNodes);
  
  // EPC + NR Helper
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
  nrHelper->SetEpcHelper(epcHelper);
  
  // 频谱配置 (Band n47, 5.89 GHz, 40 MHz)
  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;
  
  CcBwpCreator::SimpleOperationBandConf bandConfSl(5.89e9, 400, numCcPerBand, BandwidthPartInfo::V2V_Highway);
  OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc(bandConfSl);
  
  Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
  nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
  nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
  
  nrHelper->InitializeOperationBand(&bandSl);
  allBwps = CcBwpCreator::GetAllBwps({bandSl});
  
  // 天线
  nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
  nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
  nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));
  
  // PHY
  nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));
  
  // MAC
  nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(false));
  nrHelper->SetUeMacAttribute("T1", UintegerValue(2));
  nrHelper->SetUeMacAttribute("T2", UintegerValue(81));
  nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));
  nrHelper->SetUeMacAttribute("ReservationPeriod", TimeValue(MilliSeconds(20)));
  nrHelper->SetUeMacAttribute("NumSidelinkProcess", UintegerValue(4));
  nrHelper->SetUeMacAttribute("EnableBlindReTx", BooleanValue(true));
  nrHelper->SetUeMacAttribute("SlThresPsschRsrp", IntegerValue(-128));
  
  // BWP Manager
  uint8_t bwpIdForV2x = 0;
  nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
  nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK", UintegerValue(bwpIdForV2x));
  
  std::set<uint8_t> bwpIdContainer;
  bwpIdContainer.insert(bwpIdForV2x);
  
  // 安装 NR 设备
  NetDeviceContainer ueNetDevices = nrHelper->InstallUeDevice(ueNodes, allBwps);
  
  for (auto it = ueNetDevices.Begin(); it != ueNetDevices.End(); ++it) {
    DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
  }
  
  // NR Sidelink 配置
  Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
  nrSlHelper->SetEpcHelper(epcHelper);
  
  std::string errorModel = "ns3::NrLteMiErrorModel";
  nrSlHelper->SetSlErrorModel(errorModel);
  nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
  
  nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerSimple::GetTypeId());
  nrSlHelper->SetUeSlSchedulerAttribute("FixNrSlMcs", BooleanValue(true));
  nrSlHelper->SetUeSlSchedulerAttribute("InitialNrSlMcs", UintegerValue(14));
  
  nrSlHelper->PrepareUeForSidelink(ueNetDevices, bwpIdContainer);
  
  // 资源池配置 (简化)
  LteRrcSap::SlResourcePoolNr slResourcePoolNr;
  Ptr<NrSlCommPreconfigResourcePoolFactory> ptrFactory = Create<NrSlCommPreconfigResourcePoolFactory>();
  
  std::vector<std::bitset<1>> slBitMapVector;
  slBitMapVector.push_back(1);
  slBitMapVector.push_back(1);
  slBitMapVector.push_back(1);
  slBitMapVector.push_back(1);
  slBitMapVector.push_back(1);
  
  ptrFactory->SetSlTimeResources(slBitMapVector);
  ptrFactory->SetSlSensingWindow(100);
  ptrFactory->SetSlSelectionWindow(5);
  ptrFactory->SetSlFreqResourcePscch(10);
  ptrFactory->SetSlSubchannelSize(10);
  ptrFactory->SetSlMaxNumPerReserve(3);
  slResourcePoolNr = ptrFactory->CreatePool();
  
  LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
  slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
  LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
  slResourcePoolIdNr.id = 0;
  slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
  slresoPoolConfigNr.slResourcePool = slResourcePoolNr;
  
  LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
  slBwpPoolConfigCommonNr.slTxPoolSelectedNormal[0] = slresoPoolConfigNr;
  
  LteRrcSap::Bwp bwp;
  bwp.numerology = 2;
  bwp.symbolsPerSlots = 14;
  bwp.rbPerRbg = 1;
  bwp.bandwidth = 400;
  
  LteRrcSap::SlBwpGeneric slBwpGeneric;
  slBwpGeneric.bwp = bwp;
  slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
  slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);
  
  LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
  slBwpConfigCommonNr.haveSlBwpGeneric = true;
  slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
  slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
  slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;
  
  LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
  for (const auto& it : bwpIdContainer) {
    slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
  }
  
  LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
  tddUlDlConfigCommon.tddPattern = "UL|UL|UL|UL|UL|";
  
  LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
  slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;
  
  LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
  slUeSelectedPreConfig.slProbResourceKeep = 0.0;
  LteRrcSap::SlPsschTxParameters psschParams;
  psschParams.slMaxTxTransNumPssch = 5;
  LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
  pscchTxConfigList.slPsschTxParameters[0] = psschParams;
  slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;
  
  LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
  slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
  slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
  slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;
  
  nrSlHelper->InstallNrSlPreConfiguration(ueNetDevices, slPreConfigNr);
  
  // 随机流
  int64_t stream = 1;
  stream += nrHelper->AssignStreams(ueNetDevices, stream);
  stream += nrSlHelper->AssignStreams(ueNetDevices, stream);
  
  // IP 配置
  InternetStackHelper internet;
  internet.Install(ueNodes);
  
  Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueNetDevices);
  
  // 获取分配的 IP 地址
  Ipv4Address node0Ip = ueIpIface.GetAddress(0);
  Ipv4Address node1Ip = ueIpIface.GetAddress(1);
  
  NS_LOG_UNCOND("[NrUnicastTest] Node0 IP=" << node0Ip << " Node1 IP=" << node1Ip);
  
  // 路由配置
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
    Ptr<Node> ueNode = ueNodes.Get(u);
    Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
  }
  
  // ========== Sidelink Bearer 配置 (关键部分) ==========
  Time slBearersActivationTime = Seconds(2.0);
  
  Ptr<LteSlTft> tftTx0, tftRx0, tftTx1, tftRx1;
  
  if (useUnicast) {
    // 单播模式: Node0 -> Node1
    // Node0 发送到 Node1 的 IP
    uint32_t dstL2Id_0to1 = 1;  // Node1 的 L2 ID
    uint32_t dstL2Id_1to0 = 2;  // Node0 的 L2 ID
    
    NS_LOG_UNCOND("[NrUnicastTest] Configuring UNICAST bearers");
    NS_LOG_UNCOND("  Node0->Node1: dstIP=" << node1Ip << " dstL2Id=" << dstL2Id_0to1);
    NS_LOG_UNCOND("  Node1->Node0: dstIP=" << node0Ip << " dstL2Id=" << dstL2Id_1to0);
    
    // Node0: 发送到 Node1
    tftTx0 = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, 
                               LteSlTft::CommType::Uincast,  // 单播!
                               node1Ip, dstL2Id_0to1);
    // Node0: 从 Node1 接收
    tftRx0 = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, 
                               LteSlTft::CommType::Uincast, 
                               node1Ip, dstL2Id_1to0);
    
    // Node1: 发送到 Node0
    tftTx1 = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, 
                               LteSlTft::CommType::Uincast, 
                               node0Ip, dstL2Id_1to0);
    // Node1: 从 Node0 接收
    tftRx1 = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, 
                               LteSlTft::CommType::Uincast, 
                               node0Ip, dstL2Id_0to1);
    
    // 激活 Node0 的 Bearer
    NetDeviceContainer node0Dev;
    node0Dev.Add(ueNetDevices.Get(0));
    nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, node0Dev, tftTx0);
    nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, node0Dev, tftRx0);
    
    // 激活 Node1 的 Bearer
    NetDeviceContainer node1Dev;
    node1Dev.Add(ueNetDevices.Get(1));
    nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, node1Dev, tftTx1);
    nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, node1Dev, tftRx1);
    
  } else {
    // GroupCast 模式
    uint32_t dstL2Id = 255;
    Ipv4Address groupAddress4("225.0.0.0");
    
    NS_LOG_UNCOND("[NrUnicastTest] Configuring GROUPCAST bearers");
    NS_LOG_UNCOND("  Group IP=" << groupAddress4 << " L2Id=" << dstL2Id);
    
    Ptr<LteSlTft> tft = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, 
                                          LteSlTft::CommType::GroupCast, 
                                          groupAddress4, dstL2Id);
    nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, ueNetDevices, tft);
    
    tft = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, 
                            LteSlTft::CommType::GroupCast, 
                            groupAddress4, dstL2Id);
    nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, ueNetDevices, tft);
  }
  
  // ========== 应用配置 ==========
  uint16_t port = 9000;
  
  // Node1 作为 Sink (接收)
  PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", 
                               InetSocketAddress(Ipv4Address::GetAny(), port));
  ApplicationContainer sinkApp = sinkHelper.Install(ueNodes.Get(1));
  sinkApp.Start(Seconds(3.0));
  sinkApp.Stop(Seconds(simTime));
  
  // 获取 PacketSink 添加回调
  Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApp.Get(0));
  sink->TraceConnectWithoutContext("Rx", MakeCallback(&RxCallback));
  
  // Node0 作为 OnOff (发送)
  OnOffHelper onoffHelper("ns3::UdpSocketFactory", 
                           InetSocketAddress(node1Ip, port));
  onoffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
  onoffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
  onoffHelper.SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
  onoffHelper.SetAttribute("PacketSize", UintegerValue(200));
  
  ApplicationContainer clientApp = onoffHelper.Install(ueNodes.Get(0));
  clientApp.Start(Seconds(3.5));
  clientApp.Stop(Seconds(simTime - 1));
  
  // 添加 TX 回调
  Ptr<OnOffApplication> onoff = DynamicCast<OnOffApplication>(clientApp.Get(0));
  onoff->TraceConnectWithoutContext("Tx", MakeCallback(&TxCallback));
  
  // 运行仿真
  NS_LOG_UNCOND("[NrUnicastTest] Starting simulation for " << simTime << "s...");
  
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  
  // 输出结果
  NS_LOG_UNCOND("\n========== NR Unicast Test Results ==========");
  NS_LOG_UNCOND("Mode: " << (useUnicast ? "UNICAST (Uincast)" : "GROUPCAST"));
  NS_LOG_UNCOND("TX Packets: " << g_txPackets);
  NS_LOG_UNCOND("RX Packets: " << g_rxPackets);
  NS_LOG_UNCOND("Delivery Rate: " << (g_txPackets > 0 ? (100.0 * g_rxPackets / g_txPackets) : 0) << "%");
  NS_LOG_UNCOND("==============================================\n");
  
  Simulator::Destroy();
  
  return 0;
}
