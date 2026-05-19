

#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <utility>
#include <numeric>
#include <cmath>

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/wifi-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/propagation-delay-model.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("QoS_Simulation_V11");

// =============================================================================
// [C] PRIOMAP CORRIGÉE — alignée sur IP Precedence (TOS >> 5)
//
//  IP Precedence |  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
//  Bande          |  2  2  2  2  1  0  0  0  2  2  2  2  2  2  2  2
//
//  EF  = DSCP 46 = TOS 0xB8 => Prec 5 => bande 0  (haute prio VoIP)
//  AF41= DSCP 34 = TOS 0x88 => Prec 4 => bande 1  (moyenne prio Vidéo)
//  CS1 = DSCP 8  = TOS 0x20 => Prec 1 => bande 2  (faible prio BG)
//  BE  = DSCP 0  = TOS 0x00 => Prec 0 => bande 2  (faible prio FTP)
// =============================================================================
static const std::string PRIOMAP_V11 = "2 2 2 2 1 0 0 0 2 2 2 2 2 2 2 2";

// =============================================================================
// [D] Installation QDisc avec RED sur bande haute priorité
// =============================================================================
void InstallPrioQdiscV11(NetDeviceContainer& devs,
                         uint32_t devIdx,
                         const std::string& maxSize)
{
    Ptr<NetDevice> dev = devs.Get(devIdx);
    Ptr<TrafficControlLayer> tc =
        dev->GetNode()->GetObject<TrafficControlLayer>();
    if (tc && tc->GetRootQueueDiscOnDevice(dev)) {
        TrafficControlHelper tchDel;
        tchDel.Uninstall(dev);
    }

    TrafficControlHelper tch;
    uint16_t handle = tch.SetRootQueueDisc(
        "ns3::PrioQueueDisc",
        "Priomap", StringValue(PRIOMAP_V11));
    tch.AddQueueDiscClasses(handle, 3, "ns3::QueueDiscClass");

    // [D] Bande 0 (VoIP) : queue courte = drops rapides sur surcharge
    tch.AddChildQueueDisc(handle, 0, "ns3::FifoQueueDisc",
        "MaxSize", StringValue("4p"));
    // Bande 1 (Vidéo)
    tch.AddChildQueueDisc(handle, 1, "ns3::FifoQueueDisc",
        "MaxSize", StringValue(maxSize));
    // Bande 2 (FTP/BG)
    tch.AddChildQueueDisc(handle, 2, "ns3::FifoQueueDisc",
        "MaxSize", StringValue(maxSize));
    tch.Install(dev);
}

void InstallFifoQdisc(NetDeviceContainer& devs,
                      uint32_t devIdx,
                      const std::string& maxSize)
{
    Ptr<NetDevice> dev = devs.Get(devIdx);
    Ptr<TrafficControlLayer> tc =
        dev->GetNode()->GetObject<TrafficControlLayer>();
    if (tc && tc->GetRootQueueDiscOnDevice(dev)) {
        TrafficControlHelper tchDel;
        tchDel.Uninstall(dev);
    }
    TrafficControlHelper tch;
    tch.SetRootQueueDisc("ns3::FifoQueueDisc",
        "MaxSize", StringValue(maxSize));
    tch.Install(dev);
}

// =============================================================================
// Export topologie .dot
// =============================================================================
void ExportTopologyToDot(NodeContainer& nodes,
                         const std::vector<std::pair<uint32_t,uint32_t>>& links,
                         const std::string& filename,
                         const std::vector<std::string>& nodeLabels = {})
{
    std::ofstream file(filename);
    if (!file.is_open()) { return; }
    file << "graph Topology {\n  rankdir=LR;\n";
    file << "  node [shape=circle, style=filled, fillcolor=lightblue];\n";
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        std::string label = (i < nodeLabels.size() && !nodeLabels[i].empty())
                            ? nodeLabels[i] : std::to_string(i);
        file << "  n" << i << " [label=\"" << label << "\"];\n";
    }
    for (auto& lk : links)
        file << "  n" << lk.first << " -- n" << lk.second << ";\n";
    file << "}\n";
    file.close();
}

// =============================================================================
// [F] Calcul MOS (E-Model simplifié — ITU-T G.107)
//     Entrée : délai moyen (ms), taux de perte (%)
//     Sortie : MOS entre 1.0 et 4.5
// =============================================================================
double ComputeMOS(double delayMs, double lossPct)
{
    // Id = dégradation due au délai
    double Id = 0.024 * delayMs + 0.11 * (delayMs - 177.3) *
                (delayMs > 177.3 ? 1.0 : 0.0);
    // Ie = dégradation due aux pertes (codec G.711)
    double Ie = 30.0 * std::log(1.0 + 15.0 * lossPct / 100.0);
    double R  = 93.2 - Id - Ie;
    R = std::max(0.0, std::min(100.0, R));
    if (R < 0)   return 1.0;
    if (R > 100) return 4.5;
    return 1.0 + 0.035 * R + R * (R - 60.0) * (100.0 - R) * 7e-6;
}

// =============================================================================
// INDICE DE JAIN (Raj Jain, DEC TR-301, 1984)
// =============================================================================
double ComputeJainFairness(Ptr<FlowMonitor> monitor,
                           Ptr<Ipv4FlowClassifier> classifier,
                           double simDuration,
                           bool qosOnly = true)
{
    monitor->CheckForLostPackets();
    auto stats = monitor->GetFlowStats();
    auto isQosPort = [](uint16_t dp) -> bool {
        return (dp==5001||dp==6001||dp==7001 ||
                dp==5002||dp==6002||dp==7002 ||
                dp==5003||dp==7003);
    };
    std::vector<double> xi;
    for (auto& kv : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
        uint16_t dp = t.destinationPort;
        if (qosOnly && !isQosPort(dp)) continue;
        double rxBps = kv.second.rxBytes * 8.0 / simDuration;
        if (rxBps < 1.0) continue;
        xi.push_back(rxBps);
    }
    if (xi.empty()) return 0.0;
    double sumX = 0.0, sumX2 = 0.0;
    for (double x : xi) { sumX += x; sumX2 += x * x; }
    int N = (int)xi.size();
    if (sumX2 < 1e-6) return 0.0;
    return (sumX * sumX) / ((double)N * sumX2);
}

// =============================================================================
// [F] Export CSV des résultats
// =============================================================================
void ExportCSV(const std::string& filename,
               Ptr<FlowMonitor> monitor,
               Ptr<Ipv4FlowClassifier> classifier,
               double simDuration,
               bool enableQoS,
               const std::string& scenario,
               int runId)
{
    monitor->CheckForLostPackets();
    auto stats = monitor->GetFlowStats();
    std::ofstream csv(filename, std::ios::app);
    if (!csv.is_open()) return;
    // Header si fichier vide
    csv.seekp(0, std::ios::end);
    if (csv.tellp() == 0)
        csv << "scenario,run,qos,flowId,port,type,delay_ms,jitter_ms,loss_pct,tput_mbps,mos\n";

    for (auto& kv : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
        auto& s = kv.second;
        double d  = (s.rxPackets > 0) ? s.delaySum.GetSeconds()/s.rxPackets*1000.0 : 0.0;
        double j  = (s.rxPackets > 1) ? s.jitterSum.GetSeconds()/(s.rxPackets-1)*1000.0 : 0.0;
        double l  = (s.txPackets > 0) ? 100.0*(s.txPackets-s.rxPackets)/s.txPackets : 0.0;
        double th = s.rxBytes * 8.0 / simDuration / 1e6;
        uint16_t dp = t.destinationPort;
        std::string ft = "BKGND";
        if      (dp==5001||dp==6001||dp==7001) ft="VoIP";
        else if (dp==5002||dp==6002||dp==7002) ft="Video";
        else if (dp==5003||dp==7003)           ft="FTP";
        double mos = (ft=="VoIP") ? ComputeMOS(d, l) : 0.0;
        csv << scenario << "," << runId << "," << (enableQoS?"1":"0") << ","
            << kv.first << "," << dp << "," << ft << ","
            << std::fixed << std::setprecision(3)
            << d << "," << j << "," << l << "," << th << "," << mos << "\n";
    }
    csv.close();
}

// =============================================================================
// Affichage des résultats
// =============================================================================
void PrintFlowStats(Ptr<FlowMonitor> monitor,
                    Ptr<Ipv4FlowClassifier> classifier,
                    const std::string& scenario,
                    double simDuration)
{
    monitor->CheckForLostPackets();
    auto stats = monitor->GetFlowStats();

    std::cout << "\n+--------------------------------------------------------------+\n";
    std::cout << "  Résultats : " << scenario << "\n";
    std::cout << "+--------------------------------------------------------------+\n";
    std::cout << std::left
              << std::setw(8)  << "FluxID"
              << std::setw(16) << "Port(Type)"
              << std::setw(16) << "Délai moy(ms)"
              << std::setw(14) << "Jitter(ms)"
              << std::setw(11) << "Perte(%)"
              << std::setw(14) << "Débit(Mb/s)"
              << std::setw(8)  << "MOS"
              << "\n" << std::string(87, '-') << "\n";

    uint32_t totalTx = 0, totalRx = 0;
    for (auto& kv : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
        auto& s = kv.second;
        double d  = (s.rxPackets > 0) ? s.delaySum.GetSeconds()/s.rxPackets*1000.0 : 0.0;
        double j  = (s.rxPackets > 1) ? s.jitterSum.GetSeconds()/(s.rxPackets-1)*1000.0 : 0.0;
        double l  = (s.txPackets > 0) ? 100.0*(s.txPackets-s.rxPackets)/s.txPackets : 0.0;
        double th = s.rxBytes * 8.0 / simDuration / 1e6;
        uint16_t dp = t.destinationPort;
        std::string ft = "BKGND";
        if      (dp==5001||dp==6001||dp==7001) ft="VoIP";
        else if (dp==5002||dp==6002||dp==7002) ft="Video";
        else if (dp==5003||dp==7003)           ft="FTP";
        double mos = (ft=="VoIP") ? ComputeMOS(d, l) : 0.0;
        std::string mosStr = (ft=="VoIP") ?
            (std::ostringstream() << std::fixed << std::setprecision(2) << mos).str() : "-";

        std::cout << std::left
                  << std::setw(8)  << kv.first
                  << std::setw(16) << (std::to_string(dp)+"("+ft+")")
                  << std::setw(16) << std::fixed << std::setprecision(2) << d
                  << std::setw(14) << j
                  << std::setw(11) << l
                  << std::setw(14) << th
                  << std::setw(8)  << mosStr << "\n";
        totalTx += s.txPackets; totalRx += s.rxPackets;
    }

    double jainQoS = ComputeJainFairness(monitor, classifier, simDuration, true);
    std::cout << std::string(87, '-') << "\n";
    std::cout << "  TX=" << totalTx << "  RX=" << totalRx
              << "  Perte globale=" << std::fixed << std::setprecision(2)
              << (totalTx>0 ? 100.0*(totalTx-totalRx)/totalTx : 0.0) << "%\n";
    std::cout << "  Indice de Jain QoS (VoIP+Video+FTP)="
              << std::setprecision(3) << jainQoS
              << (jainQoS>0.85 ? "  [OK  Bonne équité QoS]"
                : jainQoS>0.70 ? "  [~~  Équité QoS moyenne]"
                               : "  [!!  Très inéquitable]") << "\n";

    std::cout << "\n  --- Vérification seuils QoS (3GPP TS 23.501) ---\n";
    for (auto& kv : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
        auto& s = kv.second;
        double d = (s.rxPackets>0) ? s.delaySum.GetSeconds()/s.rxPackets*1000.0 : 0.0;
        double j = (s.rxPackets>1) ? s.jitterSum.GetSeconds()/(s.rxPackets-1)*1000.0 : 0.0;
        double l = (s.txPackets>0) ? 100.0*(s.txPackets-s.rxPackets)/s.txPackets : 0.0;
        uint16_t dp = t.destinationPort;
        double mos = ComputeMOS(d, l);
        if (dp==5001||dp==6001||dp==7001)
            std::cout << "  VoIP  délai=" << std::fixed << std::setprecision(1) << d << "ms "
                      << (d<150?"[OK]":"[!!]") << " jitter=" << j << "ms "
                      << (j<30 ?"[OK]":"[!!]") << " perte=" << l << "% "
                      << (l<1.0?"[OK]":"[!!]") << " MOS=" << std::setprecision(2) << mos
                      << (mos>3.5?"[Bon]":mos>2.5?"[Moyen]":"[Mauvais]") << "\n";
        if (dp==5002||dp==6002||dp==7002)
            std::cout << "  Video délai=" << std::setprecision(1) << d << "ms "
                      << (d<200?"[OK]":"[!!]") << " jitter=" << j << "ms "
                      << (j<50 ?"[OK]":"[!!]") << " perte=" << l << "% "
                      << (l<1.0?"[OK]":"[!!]") << "\n";
        if (dp==5003||dp==7003)
            std::cout << "  FTP   délai=" << d << "ms "
                      << (d<500?"[OK]":"[!!]") << " perte=" << l << "% "
                      << (l<5.0?"[OK]":"[!!]") << "\n";
    }
}

// =============================================================================
// SCÉNARIO 1 : Wi-Fi Statique
//
// AMÉLIORATIONS V11 :
//   [A] Canal ThreeLogDistance (exponent1=2.0, exponent2=3.5, dist0=1, dist1=15m)
//       => atténuation réaliste en intérieur
//   [B] 3 flux BG OnOff aléatoires (mean ON=500ms, OFF=100ms)
//       => congestion par rafales, non constante
//   [C] TOS=0xB8 / 0x88 / 0x08 cohérents avec PRIOMAP_V11
//   [G] Paramètre runId pour la graine RNG
// =============================================================================
void RunWifiStatic(bool enableQoS, int runId = 1, const std::string& suffix = "_static")
{
    RngSeedManager::SetSeed(100 + runId);
    RngSeedManager::SetRun(runId);

    std::string label = enableQoS
        ? "Wi-Fi Statique AVEC QoS (WMM 802.11e / DiffServ)"
        : "Wi-Fi Statique SANS QoS";
    double simDuration = 30.0;

    NodeContainer apNode;     apNode.Create(1);
    NodeContainer staNodes;   staNodes.Create(5);
    NodeContainer serverNode; serverNode.Create(1);

    // Mobilité
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    pos->Add(Vector(  0.0,  0.0, 0.0));  // AP
    pos->Add(Vector(  5.0,  0.0, 0.0));  // STA0 VoIP
    pos->Add(Vector(  0.0,  8.0, 0.0));  // STA1 Vidéo
    pos->Add(Vector( -6.0,  0.0, 0.0));  // STA2 FTP
    pos->Add(Vector(  3.0, -7.0, 0.0));  // STA3 BG
    pos->Add(Vector( -4.0,  5.0, 0.0));  // STA4 BG
    mobility.SetPositionAllocator(pos);
    mobility.Install(apNode);
    mobility.Install(staNodes);
    Ptr<ConstantPositionMobilityModel> srvMob = CreateObject<ConstantPositionMobilityModel>();
    srvMob->SetPosition(Vector(100.0, 0.0, 0.0));
    serverNode.Get(0)->AggregateObject(srvMob);

    // [A] Canal propagation réaliste ThreeLogDistance
    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::ThreeLogDistancePropagationLossModel",
                               "Distance0", DoubleValue(1.0),
                               "Distance1", DoubleValue(15.0),
                               "Distance2", DoubleValue(40.0),
                               "Exponent0", DoubleValue(2.0),
                               "Exponent1", DoubleValue(3.5),
                               "Exponent2", DoubleValue(5.0));
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(20.0));
    phy.Set("TxPowerEnd",   DoubleValue(20.0));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

    WifiMacHelper mac;
    Ssid ssid = Ssid("QoS-Lab-V11");
    mac.SetType("ns3::ApWifiMac",
                "Ssid",         SsidValue(ssid),
                "QosSupported", BooleanValue(enableQoS));
    NetDeviceContainer apDevice = wifi.Install(phy, mac, apNode);

    mac.SetType("ns3::StaWifiMac",
                "Ssid",          SsidValue(ssid),
                "QosSupported",  BooleanValue(enableQoS),
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer staDevices = wifi.Install(phy, mac, staNodes);

    // Lien backhaul AP→Serveur (bottleneck)
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("2Mbps"));
    p2p.SetChannelAttribute("Delay",   StringValue("20ms"));
    NetDeviceContainer p2pDevices = p2p.Install(apNode.Get(0), serverNode.Get(0));

    InternetStackHelper stack;
    stack.Install(apNode);
    stack.Install(staNodes);
    stack.Install(serverNode);

    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0", "255.255.255.0");
    address.Assign(staDevices);
    address.Assign(apDevice);
    address.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer p2pIfaces = address.Assign(p2pDevices);
    Ipv4Address serverAddr = p2pIfaces.GetAddress(1);
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // QDisc sur bottleneck
    if (enableQoS)
        InstallPrioQdiscV11(p2pDevices, 0, "8p");
    else
        InstallFifoQdisc(p2pDevices, 0, "8p");

    // ── Applications ────────────────────────────────────────────────
    // [C] TOS cohérents avec PRIOMAP_V11

    // VoIP 64kbps EF (TOS=0xB8, DSCP 46, Precedence 5 → bande 0)
    OnOffHelper voipApp("ns3::UdpSocketFactory",
                        InetSocketAddress(serverAddr, 5001));
    voipApp.SetConstantRate(DataRate("64kbps"), 160);
    if (enableQoS) voipApp.SetAttribute("Tos", UintegerValue(0xB8));
    voipApp.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
    voipApp.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
    voipApp.Install(staNodes.Get(0));
    PacketSinkHelper s1("ns3::UdpSocketFactory",
                        InetSocketAddress(Ipv4Address::GetAny(), 5001));
    s1.Install(serverNode.Get(0)).Start(Seconds(0.5));

    // Vidéo 1Mbps AF41 (TOS=0x88, DSCP 34, Precedence 4 → bande 1)
    OnOffHelper videoApp("ns3::UdpSocketFactory",
                         InetSocketAddress(serverAddr, 5002));
    videoApp.SetConstantRate(DataRate("1Mbps"), 1000);
    if (enableQoS) videoApp.SetAttribute("Tos", UintegerValue(0x88));
    videoApp.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
    videoApp.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
    videoApp.Install(staNodes.Get(1));
    PacketSinkHelper s2("ns3::UdpSocketFactory",
                        InetSocketAddress(Ipv4Address::GetAny(), 5002));
    s2.Install(serverNode.Get(0)).Start(Seconds(0.5));

    // FTP TCP BE (TOS=0x00 → bande 2)
    BulkSendHelper ftpApp("ns3::TcpSocketFactory",
                          InetSocketAddress(serverAddr, 5003));
    ftpApp.SetAttribute("MaxBytes",  UintegerValue(0));
    ftpApp.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
    ftpApp.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
    ftpApp.Install(staNodes.Get(2));
    PacketSinkHelper s3("ns3::TcpSocketFactory",
                        InetSocketAddress(Ipv4Address::GetAny(), 5003));
    s3.Install(serverNode.Get(0)).Start(Seconds(0.5));

    // [B] BG OnOff ALÉATOIRE — rafales pour créer congestion réaliste
    // 3 flux BG : total moyen ~2.1Mbps >> bottleneck 2Mbps
    for (uint32_t i = 3; i < 5; ++i) {
        OnOffHelper bgApp("ns3::UdpSocketFactory",
                          InetSocketAddress(serverAddr, 5010 + (i-3)));
        bgApp.SetAttribute("DataRate",   DataRateValue(DataRate("1500kbps")));
        bgApp.SetAttribute("PacketSize", UintegerValue(1400));
        // [B] Distribution exponentielle ON/OFF
        bgApp.SetAttribute("OnTime",  StringValue("ns3::ExponentialRandomVariable[Mean=0.5]"));
        bgApp.SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=0.1]"));
        if (enableQoS) bgApp.SetAttribute("Tos", UintegerValue(0x08)); // CS1 → bande 2
        bgApp.SetAttribute("StartTime", TimeValue(Seconds(0.3)));
        bgApp.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
        bgApp.Install(staNodes.Get(i));
        PacketSinkHelper bgSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), 5010 + (i-3)));
        bgSink.Install(serverNode.Get(0)).Start(Seconds(0.0));
    }

    // 3ème flux BG depuis STA0 en parallèle (démarre plus tard)
    OnOffHelper bgApp3("ns3::UdpSocketFactory",
                       InetSocketAddress(serverAddr, 5012));
    bgApp3.SetAttribute("DataRate",   DataRateValue(DataRate("800kbps")));
    bgApp3.SetAttribute("PacketSize", UintegerValue(1400));
    bgApp3.SetAttribute("OnTime",  StringValue("ns3::ExponentialRandomVariable[Mean=0.3]"));
    bgApp3.SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=0.2]"));
    if (enableQoS) bgApp3.SetAttribute("Tos", UintegerValue(0x08));
    bgApp3.SetAttribute("StartTime", TimeValue(Seconds(2.0)));
    bgApp3.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
    bgApp3.Install(apNode.Get(0)); // depuis l'AP vers le serveur
    PacketSinkHelper bgSink3("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), 5012));
    bgSink3.Install(serverNode.Get(0)).Start(Seconds(0.0));

    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> monitor = flowHelper.InstallAll();
    Simulator::Stop(Seconds(simDuration));
    Simulator::Run();

    // Export
    NodeContainer allNodes;
    allNodes.Add(apNode); allNodes.Add(staNodes); allNodes.Add(serverNode);
    std::vector<std::pair<uint32_t,uint32_t>> links;
    links.push_back({0, 6});
    for (uint32_t i = 1; i <= 5; ++i) links.push_back({0, i});
    std::vector<std::string> labels = {"AP","VoIP","Video","FTP","BG1","BG2","Serveur"};
    if (runId == 1)
        ExportTopologyToDot(allNodes, links,
            "topology_wifi" + suffix + (enableQoS?"_qos":"_noqos") + ".dot", labels);

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    PrintFlowStats(monitor, classifier, label + " [run=" + std::to_string(runId) + "]", simDuration);
    ExportCSV("results_V11.csv", monitor, classifier, simDuration, enableQoS, "SC1_WiFiStatic", runId);
    Simulator::Destroy();
}

// =============================================================================
// SCÉNARIO 2 : Wi-Fi Mobile
//
// AMÉLIORATIONS V11 :
//   [A] Canal ThreeLogDistance identique au SC1
//   [B] Trafic BG OnOff aléatoire
//   [C] TOS cohérents
//   [E] RandomWaypointMobilityModel pour la STA (mobilité réaliste piétonne)
//   [G] Paramètre runId
// =============================================================================
void RunWifiMobile(bool enableQoS, int runId = 1, const std::string& suffix = "_mobile")
{
    RngSeedManager::SetSeed(200 + runId);
    RngSeedManager::SetRun(runId);

    std::string label = enableQoS
        ? "Wi-Fi Mobile AVEC QoS (WMM + Handover)"
        : "Wi-Fi Mobile SANS QoS (Handover)";
    double simDuration = 45.0;

    NodeContainer ap1Node; ap1Node.Create(1);
    NodeContainer ap2Node; ap2Node.Create(1);
    NodeContainer staNode; staNode.Create(1);
    NodeContainer swNode;  swNode.Create(1);
    NodeContainer gwNode;  gwNode.Create(1);

    // AP et infra fixes
    MobilityHelper fixMob;
    Ptr<ListPositionAllocator> fixPos = CreateObject<ListPositionAllocator>();
    fixPos->Add(Vector(  0.0,  0.0, 0.0));  // AP1
    fixPos->Add(Vector( 50.0,  0.0, 0.0));  // AP2
    fixPos->Add(Vector( 25.0, 30.0, 0.0));  // Switch
    fixPos->Add(Vector( 25.0, 50.0, 0.0));  // GW
    fixMob.SetPositionAllocator(fixPos);
    fixMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    fixMob.Install(ap1Node);
    fixMob.Install(ap2Node);
    fixMob.Install(swNode);
    fixMob.Install(gwNode);

    // [E] STA mobile — RandomWaypoint dans une zone [0,50] x [0,10]
    MobilityHelper staMob;
    staMob.SetMobilityModel("ns3::RandomWaypointMobilityModel",
        "Speed",  StringValue("ns3::UniformRandomVariable[Min=0.5|Max=1.5]"),
        "Pause",  StringValue("ns3::UniformRandomVariable[Min=0.0|Max=2.0]"),
        "PositionAllocator",
        PointerValue([] {
            Ptr<RandomRectanglePositionAllocator> alloc =
                CreateObject<RandomRectanglePositionAllocator>();
            alloc->SetX(CreateObjectWithAttributes<UniformRandomVariable>(
                "Min", DoubleValue(0.0), "Max", DoubleValue(50.0)));
            alloc->SetY(CreateObjectWithAttributes<UniformRandomVariable>(
                "Min", DoubleValue(0.0), "Max", DoubleValue(10.0)));
            return alloc;
        }()));
    Ptr<ListPositionAllocator> staPos = CreateObject<ListPositionAllocator>();
    staPos->Add(Vector(2.0, 0.0, 0.0));
    staMob.SetPositionAllocator(staPos);
    staMob.Install(staNode);

    // [A] Canal partagé ThreeLogDistance
    YansWifiChannelHelper chanHelper;
    chanHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    chanHelper.AddPropagationLoss("ns3::ThreeLogDistancePropagationLossModel",
                                  "Distance0", DoubleValue(1.0),
                                  "Distance1", DoubleValue(15.0),
                                  "Distance2", DoubleValue(40.0),
                                  "Exponent0", DoubleValue(2.0),
                                  "Exponent1", DoubleValue(3.5),
                                  "Exponent2", DoubleValue(5.0));
    Ptr<YansWifiChannel> sharedChan = chanHelper.Create();
    YansWifiPhyHelper phy;
    phy.SetChannel(sharedChan);
    phy.Set("TxPowerStart", DoubleValue(20.0));
    phy.Set("TxPowerEnd",   DoubleValue(20.0));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");

    WifiMacHelper mac;
    Ssid ssid = Ssid("ESS-Mobile-V11");
    mac.SetType("ns3::ApWifiMac",
                "Ssid",         SsidValue(ssid),
                "QosSupported", BooleanValue(enableQoS));
    NetDeviceContainer ap1Dev = wifi.Install(phy, mac, ap1Node);
    NetDeviceContainer ap2Dev = wifi.Install(phy, mac, ap2Node);

    mac.SetType("ns3::StaWifiMac",
                "Ssid",          SsidValue(ssid),
                "QosSupported",  BooleanValue(enableQoS),
                "ActiveProbing", BooleanValue(true));
    NetDeviceContainer staDev = wifi.Install(phy, mac, staNode);

    PointToPointHelper p2pFast;
    p2pFast.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    p2pFast.SetChannelAttribute("Delay",   StringValue("1ms"));
    NetDeviceContainer ap1sw = p2pFast.Install(ap1Node.Get(0), swNode.Get(0));
    NetDeviceContainer ap2sw = p2pFast.Install(ap2Node.Get(0), swNode.Get(0));

    // Bottleneck légèrement plus serré pour bien différencier QoS
    PointToPointHelper p2pBottle;
    p2pBottle.SetDeviceAttribute("DataRate", StringValue("1100kbps"));
    p2pBottle.SetChannelAttribute("Delay",   StringValue("10ms"));
    NetDeviceContainer swgw = p2pBottle.Install(swNode.Get(0), gwNode.Get(0));

    InternetStackHelper stack;
    stack.Install(ap1Node); stack.Install(ap2Node);
    stack.Install(staNode); stack.Install(swNode); stack.Install(gwNode);

    Ipv4AddressHelper addr;
    addr.SetBase("192.168.1.0", "255.255.255.0");
    addr.Assign(ap1Dev); addr.Assign(ap2Dev); addr.Assign(staDev);
    addr.SetBase("10.1.1.0", "255.255.255.0"); addr.Assign(ap1sw);
    addr.SetBase("10.1.2.0", "255.255.255.0"); addr.Assign(ap2sw);
    addr.SetBase("10.2.0.0", "255.255.255.0");
    Ipv4InterfaceContainer swgwIfaces = addr.Assign(swgw);
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    Ipv4Address gwAddr = swgwIfaces.GetAddress(1);

    if (enableQoS)
        InstallPrioQdiscV11(swgw, 0, "10p");
    else
        InstallFifoQdisc(swgw, 0, "10p");

    // VoIP 64kbps EF
    OnOffHelper voipApp("ns3::UdpSocketFactory",
                        InetSocketAddress(gwAddr, 6001));
    voipApp.SetConstantRate(DataRate("64kbps"), 160);
    if (enableQoS) voipApp.SetAttribute("Tos", UintegerValue(0xB8));
    voipApp.SetAttribute("StartTime", TimeValue(Seconds(2.0)));
    voipApp.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
    voipApp.Install(staNode.Get(0));
    PacketSinkHelper sink1("ns3::UdpSocketFactory",
                           InetSocketAddress(Ipv4Address::GetAny(), 6001));
    sink1.Install(gwNode.Get(0)).Start(Seconds(1.0));

    // Vidéo 1Mbps AF41
    OnOffHelper videoApp("ns3::UdpSocketFactory",
                         InetSocketAddress(gwAddr, 6002));
    videoApp.SetConstantRate(DataRate("1Mbps"), 1000);
    if (enableQoS) videoApp.SetAttribute("Tos", UintegerValue(0x88));
    videoApp.SetAttribute("StartTime", TimeValue(Seconds(2.0)));
    videoApp.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
    videoApp.Install(staNode.Get(0));
    PacketSinkHelper sink2("ns3::UdpSocketFactory",
                           InetSocketAddress(Ipv4Address::GetAny(), 6002));
    sink2.Install(gwNode.Get(0)).Start(Seconds(1.0));

    // [B] BG OnOff aléatoire depuis la STA
    OnOffHelper bgApp("ns3::UdpSocketFactory",
                      InetSocketAddress(gwAddr, 6010));
    bgApp.SetAttribute("DataRate",   DataRateValue(DataRate("1200kbps")));
    bgApp.SetAttribute("PacketSize", UintegerValue(1400));
    bgApp.SetAttribute("OnTime",  StringValue("ns3::ExponentialRandomVariable[Mean=0.6]"));
    bgApp.SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=0.15]"));
    if (enableQoS) bgApp.SetAttribute("Tos", UintegerValue(0x08));
    bgApp.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
    bgApp.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
    bgApp.Install(staNode.Get(0));
    PacketSinkHelper sink3("ns3::UdpSocketFactory",
                           InetSocketAddress(Ipv4Address::GetAny(), 6010));
    sink3.Install(gwNode.Get(0)).Start(Seconds(0.0));

    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> monitor = flowHelper.InstallAll();
    Simulator::Stop(Seconds(simDuration));
    Simulator::Run();

    NodeContainer allNodes;
    allNodes.Add(ap1Node); allNodes.Add(ap2Node);
    allNodes.Add(staNode); allNodes.Add(swNode); allNodes.Add(gwNode);
    std::vector<std::pair<uint32_t,uint32_t>> links;
    links.push_back({0,3}); links.push_back({1,3}); links.push_back({3,4});
    links.push_back({2,0}); links.push_back({2,1});
    std::vector<std::string> labels = {"AP1","AP2","STA mobile","Switch","GW"};
    if (runId == 1)
        ExportTopologyToDot(allNodes, links,
            "topology_wifi" + suffix + (enableQoS?"_qos":"_noqos") + ".dot", labels);

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    PrintFlowStats(monitor, classifier, label + " [run=" + std::to_string(runId) + "]", simDuration);
    ExportCSV("results_V11.csv", monitor, classifier, simDuration, enableQoS, "SC2_WiFiMobile", runId);
    Simulator::Destroy();
}

// =============================================================================
// SCÉNARIO 3 : LTE
//
// AMÉLIORATIONS V11 :
//   [A] LogDistancePropagationLossModel avec exponent=3.5 (urbain dense)
//   [B] BG OnOff aléatoire, backhaul réduit à 1.5Mbps (saturation plus nette)
//   [C] TOS sur VoIP et Vidéo en mode QoS
//   [G] Paramètre runId
// =============================================================================
void RunLte(bool enableQoS, int runId = 1, const std::string& suffix = "_lte")
{
    RngSeedManager::SetSeed(300 + runId);
    RngSeedManager::SetRun(runId);

    std::string label = enableQoS
        ? "LTE AVEC QoS (PF + QCI 1/2/8 + GBR + TC)"
        : "LTE SANS QoS (RR + FIFO backhaul)";
    double simDuration = 30.0;

    Config::SetDefault("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(25));
    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(25));

    Ptr<LteHelper>             lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epc       = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epc);

    lteHelper->SetSchedulerType(enableQoS
        ? "ns3::PfFfMacScheduler"
        : "ns3::RrFfMacScheduler");

    // [A] Propagation urbain dense
    lteHelper->SetAttribute("PathlossModel",
                            StringValue("ns3::LogDistancePropagationLossModel"));
    Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent",
                       DoubleValue(3.5));
    Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                       DoubleValue(46.7));

    Ptr<Node> pgw = epc->GetPgwNode();
    NodeContainer remoteHostContainer; remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // [B] Backhaul 1.5Mbps => saturation forte avec BG 6Mbps
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute ("DataRate", StringValue("1500kbps"));
    p2p.SetChannelAttribute("Delay",    StringValue("15ms"));
    NetDeviceContainer internetDevices = p2p.Install(pgw, remoteHost);

    if (enableQoS)
        InstallPrioQdiscV11(internetDevices, 0, "6p");
    else
        InstallFifoQdisc(internetDevices, 0, "6p");

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper routingHelper;
    Ptr<Ipv4StaticRouting> remoteHostRouting =
        routingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostRouting->AddNetworkRouteTo(
        Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NodeContainer enbNodes; enbNodes.Create(1);
    NodeContainer ueNodes;  ueNodes.Create(7);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> enbPos = CreateObject<ListPositionAllocator>();
    enbPos->Add(Vector(0.0, 0.0, 30.0));
    mob.SetPositionAllocator(enbPos);
    mob.Install(enbNodes);

    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    uePos->Add(Vector( 10.0,   0.0, 1.5));  // UE0 VoIP   (proche)
    uePos->Add(Vector(  0.0,  15.0, 1.5));  // UE1 Vidéo  (proche)
    uePos->Add(Vector(-15.0,   0.0, 1.5));  // UE2 FTP    (moyen)
    uePos->Add(Vector( 90.0,  90.0, 1.5));  // UE3 BG     (loin)
    uePos->Add(Vector(-90.0,  90.0, 1.5));  // UE4 BG     (loin)
    uePos->Add(Vector( 90.0, -90.0, 1.5));  // UE5 BG     (loin)
    uePos->Add(Vector(-90.0, -90.0, 1.5));  // UE6 BG     (loin)
    mob.SetPositionAllocator(uePos);
    mob.Install(ueNodes);

    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueDevs  = lteHelper->InstallUeDevice(ueNodes);

    internet.Install(ueNodes);
    epc->AssignUeIpv4Address(ueDevs);
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {
        Ptr<Ipv4StaticRouting> ueRouting =
            routingHelper.GetStaticRouting(ueNodes.Get(i)->GetObject<Ipv4>());
        ueRouting->SetDefaultRoute(epc->GetUeDefaultGatewayAddress(), 1);
    }
    lteHelper->Attach(ueDevs, enbDevs.Get(0));

    if (enableQoS) {
        // VoIP QCI 1 GBR 64kbps
        Ptr<EpcTft> tftV = Create<EpcTft>();
        EpcTft::PacketFilter pfV;
        pfV.remotePortStart = pfV.remotePortEnd = 7001;
        tftV->Add(pfV);
        GbrQosInformation qV;
        qV.gbrDl = qV.gbrUl =  64000;
        qV.mbrDl = qV.mbrUl = 128000;
        lteHelper->ActivateDedicatedEpsBearer(
            ueDevs.Get(0), EpsBearer(EpsBearer::GBR_CONV_VOICE, qV), tftV);

        // Vidéo QCI 2 GBR 1Mbps
        Ptr<EpcTft> tftVi = Create<EpcTft>();
        EpcTft::PacketFilter pfVi;
        pfVi.remotePortStart = pfVi.remotePortEnd = 7002;
        tftVi->Add(pfVi);
        GbrQosInformation qVi;
        qVi.gbrDl = qVi.gbrUl = 1000000;
        qVi.mbrDl = qVi.mbrUl = 2000000;
        lteHelper->ActivateDedicatedEpsBearer(
            ueDevs.Get(1), EpsBearer(EpsBearer::GBR_CONV_VIDEO, qVi), tftVi);

        // FTP QCI 8 Non-GBR
        Ptr<EpcTft> tftF = Create<EpcTft>();
        EpcTft::PacketFilter pfF;
        pfF.remotePortStart = pfF.remotePortEnd = 7003;
        tftF->Add(pfF);
        lteHelper->ActivateDedicatedEpsBearer(
            ueDevs.Get(2), EpsBearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT), tftF);
    }

    // VoIP 64kbps EF [C]
    OnOffHelper voipApp("ns3::UdpSocketFactory",
                        InetSocketAddress(remoteHostAddr, 7001));
    voipApp.SetConstantRate(DataRate("64kbps"), 160);
    if (enableQoS) voipApp.SetAttribute("Tos", UintegerValue(0xB8));
    voipApp.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
    voipApp.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
    voipApp.Install(ueNodes.Get(0));
    PacketSinkHelper vs("ns3::UdpSocketFactory",
                        InetSocketAddress(Ipv4Address::GetAny(), 7001));
    vs.Install(remoteHost).Start(Seconds(0.5));

    // Vidéo 1Mbps AF41 [C]
    OnOffHelper videoApp("ns3::UdpSocketFactory",
                         InetSocketAddress(remoteHostAddr, 7002));
    videoApp.SetConstantRate(DataRate("1Mbps"), 1000);
    if (enableQoS) videoApp.SetAttribute("Tos", UintegerValue(0x88));
    videoApp.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
    videoApp.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
    videoApp.Install(ueNodes.Get(1));
    PacketSinkHelper vi("ns3::UdpSocketFactory",
                        InetSocketAddress(Ipv4Address::GetAny(), 7002));
    vi.Install(remoteHost).Start(Seconds(0.5));

    // FTP TCP BE
    BulkSendHelper ftpApp("ns3::TcpSocketFactory",
                          InetSocketAddress(remoteHostAddr, 7003));
    ftpApp.SetAttribute("MaxBytes",  UintegerValue(0));
    ftpApp.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
    ftpApp.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
    ftpApp.Install(ueNodes.Get(2));
    PacketSinkHelper fs("ns3::TcpSocketFactory",
                        InetSocketAddress(Ipv4Address::GetAny(), 7003));
    fs.Install(remoteHost).Start(Seconds(0.5));

    // [B] BG OnOff aléatoire x4 UE lointains — total ~6Mbps >> 1.5Mbps backhaul
    for (uint32_t i = 3; i < 7; ++i) {
        OnOffHelper bgApp("ns3::UdpSocketFactory",
                          InetSocketAddress(remoteHostAddr, 7010+(i-3)));
        bgApp.SetAttribute("DataRate",   DataRateValue(DataRate("1500kbps")));
        bgApp.SetAttribute("PacketSize", UintegerValue(1400));
        bgApp.SetAttribute("OnTime",  StringValue("ns3::ExponentialRandomVariable[Mean=0.5]"));
        bgApp.SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=0.1]"));
        bgApp.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
        bgApp.SetAttribute("StopTime",  TimeValue(Seconds(simDuration)));
        bgApp.Install(ueNodes.Get(i));
        PacketSinkHelper bs("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), 7010+(i-3)));
        bs.Install(remoteHost).Start(Seconds(0.0));
    }

    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> monitor = flowHelper.InstallAll();
    Simulator::Stop(Seconds(simDuration));
    Simulator::Run();

    NodeContainer allNodes;
    allNodes.Add(enbNodes); allNodes.Add(ueNodes);
    allNodes.Add(pgw);      allNodes.Add(remoteHost);
    std::vector<std::pair<uint32_t,uint32_t>> links;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i) links.push_back({0, 1+i});
    links.push_back({8, 0}); links.push_back({8, 9});
    std::vector<std::string> labels = {
        "eNB","UE0_VoIP","UE1_Video","UE2_FTP",
        "UE3_BG","UE4_BG","UE5_BG","UE6_BG","PGW","RemoteHost"
    };
    if (runId == 1)
        ExportTopologyToDot(allNodes, links,
            "topology_lte" + suffix + (enableQoS?"_qos":"_noqos") + ".dot", labels);

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    PrintFlowStats(monitor, classifier, label + " [run=" + std::to_string(runId) + "]", simDuration);
    ExportCSV("results_V11.csv", monitor, classifier, simDuration, enableQoS, "SC3_LTE", runId);
    Simulator::Destroy();
}

// =============================================================================
// MAIN
// =============================================================================
int main(int argc, char* argv[])
{
    bool runWifiStatic = true;
    bool runWifiMobile = true;
    bool runLte        = true;
    int  numRuns       = 3;  // [G] Nombre de répétitions statistiques

    CommandLine cmd(__FILE__);
    cmd.AddValue("wifi-static", "Scénario Wi-Fi statique", runWifiStatic);
    cmd.AddValue("wifi-mobile", "Scénario Wi-Fi mobile",   runWifiMobile);
    cmd.AddValue("lte",         "Scénario LTE",            runLte);
    cmd.AddValue("runs",        "Nombre de répétitions",   numRuns);
    cmd.Parse(argc, argv);

    LogComponentEnable("QoS_Simulation_V11", LOG_LEVEL_INFO);

    // Supprimer l'ancien CSV pour repartir proprement
    std::remove("results_V11.csv");

    std::cout
        << "\n+============================================================+\n"
        << "   Simulation QoS V11 — Réseaux Sans Fil & Mobile           \n"
        << "   Projet ISIC 2A — Prof. HAIDINE — Groupe B4               \n"
        << "+============================================================+\n\n"
        << "Améliorations V11 :\n"
        << "  [A] Propagation ThreeLogDistance (Wi-Fi) / LogDistance 3.5 (LTE)\n"
        << "  [B] Trafic BG OnOff exponentiel (rafales réalistes)\n"
        << "  [C] DSCP EF/AF41/CS1 cohérents avec Priomap corrigée\n"
        << "  [D] Queue haute-prio courte (4p) pour drops rapides sur BG\n"
        << "  [E] Mobilité RandomWaypoint (SC2)\n"
        << "  [F] MOS (E-Model G.107) + Export CSV\n"
        << "  [G] " << numRuns << " runs par scénario (seeds différentes)\n\n"
        << "Seuils QoS (3GPP TS 23.501 / ITU-T Y.1541) :\n"
        << "  VoIP  : délai < 150ms  jitter < 30ms  perte < 1%  MOS > 3.5\n"
        << "  Vidéo : délai < 200ms  jitter < 50ms  perte < 1%\n"
        << "  FTP   : délai < 500ms  perte  < 5%\n\n";

    if (runWifiStatic) {
        std::cout << "\n>>> SCÉNARIO 1 : Wi-Fi Statique (802.11n / WMM 802.11e)\n"
                  << "    ThreeLogDistance, BG OnOff x3, bottleneck 2Mbps\n";
        for (int r = 1; r <= numRuns; r++) {
            RunWifiStatic(false, r, "_static");
            RunWifiStatic(true,  r, "_static");
        }
    }

    if (runWifiMobile) {
        std::cout << "\n>>> SCÉNARIO 2 : Wi-Fi Mobile (RandomWaypoint)\n"
                  << "    ThreeLogDistance, BG OnOff, bottleneck 1.1Mbps\n";
        for (int r = 1; r <= numRuns; r++) {
            RunWifiMobile(false, r, "_mobile");
            RunWifiMobile(true,  r, "_mobile");
        }
    }

    if (runLte) {
        std::cout << "\n>>> SCÉNARIO 3 : LTE (4G EPS Bearers QCI)\n"
                  << "    LogDistance 3.5, BG OnOff x4, backhaul 1.5Mbps\n";
        for (int r = 1; r <= numRuns; r++) {
            RunLte(false, r, "_lte");
            RunLte(true,  r, "_lte");
        }
    }

    std::cout
        << "\n[OK] Simulation V11 terminée.\n"
        << "  Fichiers générés :\n"
        << "    topology_*.dot  — topologies (convertir: dot -Tpng x.dot -o x.png)\n"
        << "    results_V11.csv — résultats bruts pour analyse statistique\n\n"
        << "  Post-traitement CSV (Python) :\n"
        << "    import pandas as pd\n"
        << "    df = pd.read_csv('results_V11.csv')\n"
        << "    print(df.groupby(['scenario','qos','type'])[['delay_ms','loss_pct','mos']].agg(['mean','std']))\n\n"
        << "Références :\n"
        << "  [1] R. Jain et al., DEC TR-301, 1984\n"
        << "  [2] IEEE 802.11e-2005 (WMM/EDCA)\n"
        << "  [3] 3GPP TS 23.203 / TS 23.501\n"
        << "  [4] RFC 2474/3246/2597 (DSCP/EF/AF)\n"
        << "  [5] ITU-T G.107 (E-Model MOS)\n";
    return 0;
}
