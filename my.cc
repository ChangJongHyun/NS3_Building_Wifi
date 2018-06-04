#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/propagation-module.h"
#include "ns3/buildings-module.h"
#include "ns3/buildings-helper.h"
#include "ns3/mobility-building-info.h"
#include "ns3/netanim-module.h"
#include "ns3/csma-module.h"
#include <iostream>
#include <algorithm>
#include "ns3/point-to-point-module.h"
#include "ns3/basic-energy-source.h"
#include "ns3/simple-device-energy-model.h"
#include <sys/stat.h>
#include <unistd.h>
#include "ns3/yans-wifi-phy.h"
#include <ns3/mobility-helper.h>
#include "ns3/ipv4.h"
#include "ns3/flow-monitor-module.h"
#include <math.h>
#include <stdio.h>
#include <float.h>
#include <set>
#include <vector>

NS_LOG_COMPONENT_DEFINE ("wifi-tcp-nt");

using namespace ns3;

void ThroughputMonitor (FlowMonitorHelper* flowHelper, Ptr<FlowMonitor> flowMonitor){
    flowMonitor->CheckForLostPackets ();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowHelper->GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats ();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        std::cout << "\nFlow: "<< i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
        std::cout << "Tx Packets: " <<i->second.txPackets << "\n";
        std::cout << "Rx Packets: " <<i->second.rxPackets << "\n";
        std::cout << "Throughput: " <<i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds()-i->second.timeFirstTxPacket.GetSeconds())/1024/1024<<" Mbps\n";
        std::cout << "Packet Loss Ratio: " << (i->second.txPackets -i->second.rxPackets)*100/(double)i->second.txPackets << std::endl;
    }
    Simulator::Schedule(Seconds(3), &ThroughputMonitor, flowHelper, flowMonitor);
}


void
ShowNodeInformation(NodeContainer m_ap, NodeContainer m_sta, Ptr<Building> m_building) {
    std::cout << "---------------AP info---------------" << std::endl;
    for (auto it = m_ap.Begin(); it != m_ap.End(); it++) {
        Ptr<MobilityModel> mm = (*it)->GetObject<MobilityModel>();
        Ptr<Ipv4> ipv4 = (*it)->GetObject<Ipv4>();
        Vector p = mm->GetPosition();

        std::cout << " AP  id => " << (*it)->GetId() << std::endl;
        std::cout << " AP  ipv4 => " << ipv4->GetAddress(1, 0).GetLocal() << std::endl;
        //std::cout<<" AP  Mac => "<<mac48->GetBssid()<<std::endl;
        std::cout << " AP  Room Pos => (" << m_building->GetRoomX(p) <<
                  ", " << m_building->GetRoomY(p) << ", " << m_building->GetFloor(p) << ")" << std::endl;
        if ((*it)->GetNDevices() != 0) {
            for (uint32_t i = 0; i < (*it)->GetNDevices(); i++) {
                std::cout << " AP  Device => (" << i << ")" << (*it)->GetDevice(i)->GetInstanceTypeId() << std::endl;
            }
        } else {
            std::cout << " AP  Devices => " << (*it)->GetNDevices() << std::endl;
        }
        if ((*it)->GetNApplications() != 0) {
            for (uint32_t i = 0; i < (*it)->GetNApplications(); i++)
                std::cout << " AP  App    => (" << i << ")" << (*it)->GetApplication(i)->GetInstanceTypeId() << std::endl;
        } else {
            std::cout << " AP  App    => " << (*it)->GetNApplications() << std::endl;
        }
    }

    std::cout << "\n---------------STA info---------------" << std::endl;
    for (auto it = m_sta.Begin(); it != m_sta.End(); it++) {
        Ptr<MobilityModel> mm = (*it)->GetObject<MobilityModel>();
        Ptr<Ipv4> ipv4 = (*it)->GetObject<Ipv4>();
        Vector p = mm->GetPosition();

        std::cout << " STA id => " << (*it)->GetId() << std::endl;
        std::cout << " STA ipv4 => " << ipv4->GetAddress(1, 0).GetLocal() << std::endl;
        std::cout << " STA Room Pos => (" << m_building->GetRoomX(p) <<
                  ", " << m_building->GetRoomY(p) << ", " << m_building->GetFloor(p) << ")" << std::endl;
        if ((*it)->GetNDevices() != 0) {
            for (uint32_t i = 0; i < (*it)->GetNDevices(); i++)
                std::cout << " STA Device => (" << i << ")" << (*it)->GetDevice(i)->GetInstanceTypeId() << std::endl;
        } else {
            std::cout << " STA Devices => " << (*it)->GetNDevices() << std::endl;
        }
        if ((*it)->GetNApplications() != 0) {
            for (uint32_t i = 0; i < (*it)->GetNApplications(); i++)
                std::cout << " STA App    => (" << i << ")" << (*it)->GetApplication(i)->GetInstanceTypeId() << std::endl;
        } else {
            std::cout << " STA App    => " << (*it)->GetNApplications() << std::endl;
        }
    }

    std::cout << "-------------------------------------\n" << std::endl;

    std::cout << "----------wifi channel info----------" << std::endl;
    std::cout << "---------------AP 01--------------------" << std::endl;
    Ptr<NetDevice> net = m_ap.Get(0)->GetDevice(0);
    Ptr<WifiNetDevice> wifi = StaticCast<WifiNetDevice>(net);
    Ptr<WifiPhy> wifiPhy = wifi->GetPhy();
    Ptr<WifiMac> wifiMac = wifi->GetMac();
    std::cout<<"Wifi info"<<std::endl;
    std::cout<<" # Channel: "<< wifi->GetChannel()->GetId()<<std::endl;
    std::cout<<" # Freq: "<<wifiPhy->GetFrequency()<<std::endl;

    std::cout << "----------wifi channel info----------" << std::endl;
    std::cout << "---------------AP 02--------------------" << std::endl;
    Ptr<NetDevice> net2 = m_ap.Get(0)->GetDevice(0);
    Ptr<WifiNetDevice> wifi2 = StaticCast<WifiNetDevice>(net);
    Ptr<WifiPhy> wifiPhy2 = wifi2->GetPhy();
    Ptr<WifiMac> wifiMac2 = wifi2->GetMac();
    std::cout<<"Wifi info"<<std::endl;
    std::cout<<" # Channel: "<< wifi2->GetChannel()->GetId()<<std::endl;
    std::cout<<" # Freq: "<<wifiPhy2->GetFrequency()<<std::endl;

    std::cout << "----------wifi channel info----------" << std::endl;
    std::cout << "---------------AP 01--------------------" << std::endl;
    Ptr<NetDevice> net3 = m_ap.Get(0)->GetDevice(0);
    Ptr<WifiNetDevice> wifi3 = StaticCast<WifiNetDevice>(net);
    Ptr<WifiPhy> wifiPhy3 = wifi3->GetPhy();
    Ptr<WifiMac> wifiMac3 = wifi3->GetMac();
    std::cout<<"Wifi info"<<std::endl;
    std::cout<<" # Channel: "<< wifi3->GetChannel()->GetId()<<std::endl;
    std::cout<<" # Freq: "<<wifiPhy3->GetFrequency()<<std::endl;

}


static void
SetPosition (Ptr<Node> node, Vector position)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
    mobility->SetPosition (position);
}


static Vector
GetPosition (Ptr<Node> node)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
    return mobility->GetPosition ();
}


static void
AdvancePosition (Ptr<Node> node)
{
    Vector pos = GetPosition (node);
    pos.x += 5.0;
    if (pos.x >= 210.0)
    {
        return;
    }
    SetPosition (node, pos);

    Simulator::Schedule (Seconds (1.0), &AdvancePosition, node);
}


struct Room
{
    Room(uint32_t xx, uint32_t yy, uint32_t zz);
    uint32_t x;
    uint32_t y;
    uint32_t z;
};

Room::Room(uint32_t xx, uint32_t yy, uint32_t zz)
        :x(xx), y(yy), z(zz)
{
}

bool operator < (const Room& a, const Room& b)
{
    return ( (a.x < b.x) || ( (a.x == b.x) && (a.y < b.y) ) || ( (a.x == b.x) && (a.y == b.y) && (a.z < b.z) ));
}

template <typename Type>
Type max(Type a, Type b){
    return a>b ? a: b;
}

template <typename Type>
Type min(Type a, Type b){
    return a<b ? a: b;
}


double g_signalDbmAvg0, g_noiseDbmAvg0, sig0, nal0; uint32_t g_samples0; double max0 = -1000; double min0 = 0;
double g_signalDbmAvg1, g_noiseDbmAvg1, sig1, nal1; uint32_t g_samples1; double max1 = -1000; double min1 = 0;
double g_signalDbmAvg2, g_noiseDbmAvg2, sig2, nal2; uint32_t g_samples2; double max2 = -1000; double min2 = 0;


void MonitorSniffRx0 (Ptr<const Packet> packet,
                      uint16_t channelFreqMhz,
                      WifiTxVector txVector,
                      MpduInfo aMpdu,
                      SignalNoiseDbm signalNoise0)
{
    g_samples0++;
    g_signalDbmAvg0 += ((signalNoise0.signal - g_signalDbmAvg0) / g_samples0);
    g_noiseDbmAvg0 += ((signalNoise0.noise - g_noiseDbmAvg0) / g_samples0);

    sig0 = max (max0, signalNoise0.signal);
    max0 = sig0;
    std::cout<<"max0 :"<<max0<<std::endl;

    nal0 = min (min0, signalNoise0.signal);
    min0 = nal0;
    //std::cout<<"min0 :"<<min0<<std::endl;
}

void MonitorSniffRx1 (Ptr<const Packet> packet,
                      uint16_t channelFreqMhz,
                      WifiTxVector txVector,
                      MpduInfo aMpdu,
                      SignalNoiseDbm signalNoise1)
{
    g_samples1++;
    g_signalDbmAvg1 += ((signalNoise1.signal - g_signalDbmAvg1) / g_samples1);
    g_noiseDbmAvg1 += ((signalNoise1.noise - g_noiseDbmAvg1) / g_samples1);

    sig1 = max (max1, signalNoise1.signal);
    max1 = sig1;
    //std::cout<<"max1 :"<<max1<<std::endl;

    nal1 = min (min1, signalNoise1.signal);
    min1 = nal1;
    //std::cout<<"min0 :"<<min0<<std::endl;
}


void MonitorSniffRx2 (Ptr<const Packet> packet,
                      uint16_t channelFreqMhz,
                      WifiTxVector txVector,
                      MpduInfo aMpdu,
                      SignalNoiseDbm signalNoise2)
{
    g_samples2++;
    g_signalDbmAvg2 += ((signalNoise2.signal - g_signalDbmAvg2) / g_samples2);
    g_noiseDbmAvg2 += ((signalNoise2.noise - g_noiseDbmAvg2) / g_samples2);

    sig2 = max (max2, signalNoise2.signal);
    max2 = sig2;
    //std::cout<<"max2 :"<<max2<<std::endl;

    nal2 = min (min2, signalNoise2.signal);
    min2 = nal2;
    //std::cout<<"min0 :"<<min0<<std::endl;
}


void signalMonitor (){

    std::cout<<"\nAP0 Avg_RSSI(dBm): "<<g_signalDbmAvg0<<std::endl;
    std::cout<<"AP0 Max_RSSI(dBm): "<<max0<<std::endl;
    std::cout<<"AP0 Min_RSSI(dBm): "<<min0<<std::endl;
    std::cout<<"AP0 Noise(dBm): "<<g_noiseDbmAvg0<<std::endl;
    std::cout<<"AP0 SNR(dB): "<<(g_signalDbmAvg0 - g_noiseDbmAvg0)<<std::endl;

    std::cout<<"\nAP1 Avg_RSSI(dBm): "<<g_signalDbmAvg1<<std::endl;
    std::cout<<"AP1 Max_RSSI(dBm): "<<max1<<std::endl;
    std::cout<<"AP1 Min_RSSI(dBm): "<<min1<<std::endl;
    std::cout<<"AP1 Noise(dBm): "<<g_noiseDbmAvg1<<std::endl;
    std::cout<<"AP1 SNR(dB): "<<(g_signalDbmAvg1 - g_noiseDbmAvg1)<<std::endl;

    std::cout<<"\nAP2 Avg_RSSI(dBm): "<<g_signalDbmAvg2<<std::endl;
    std::cout<<"AP2 Max_RSSI(dBm): "<<max2<<std::endl;
    std::cout<<"AP2 Min_RSSI(dBm): "<<min2<<std::endl;
    std::cout<<"AP2 Noise(dBm): "<<g_noiseDbmAvg2<<std::endl;
    std::cout<<"AP2 SNR(dB): "<<(g_signalDbmAvg2 - g_noiseDbmAvg2)<<std::endl;

    Simulator::Schedule(MilliSeconds(100), &signalMonitor);
}

double nt0;
double nt1;
double nt2;

//uint32_t nAP = 3;
//NodeContainer ap;
//ap.Create (nAP);


//void calculateCST (YansWifiPhyHelper wifiPhy, WifiMacHelper wifiMac, WifiHelper wifiHelper, NetDeviceContainer apDevices, NodeContainer ap, InternetStackHelper stack, Ipv4AddressHelper address,  Ipv4InterfaceContainer apInterface){

//void calculateCST (YansWifiPhyHelper* wifiPhy, NetDeviceContainer apDevices, WifiHelper* wifiHelper, WifiMacHelper wifiMac, NodeContainer ap){

void calculateCST (NetDeviceContainer apDevices, NodeContainer ap, WifiMacHelper wifiMac,  YansWifiChannelHelper channel){
    YansWifiPhyHelper wifiPhy;

    WifiHelper wifiHelper;
    wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
    Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(40.046));
    wifiHelper.SetRemoteStationManager("ns3::IdealWifiManager");

    wifiPhy = YansWifiPhyHelper::Default();
    wifiPhy.SetChannel(channel.Create());

//  if (ap.Get(0)){
    nt0 = min (max (max0, min0) - 25, min0);
    wifiPhy.Set ("CcaMode1Threshold", DoubleValue (nt0));
    wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (nt0 + 3));
//  } else if (ap.Get(1)){

    nt1 = min (max (max1, min1) - 25, min1);
    wifiPhy.Set ("CcaMode1Threshold", DoubleValue (nt1));
    wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (nt1 + 3));
//  } else if (ap.Get(2)){

    nt2 = min (max (max2, min2) - 25, min2);
    wifiPhy.Set ("CcaMode1Threshold", DoubleValue (nt2));
    wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (nt2 + 3));
    std::cout<<"HEllo"<<std::endl;
    apDevices = wifiHelper.Install (wifiPhy, wifiMac, ap);
//  }
    std::cout<<"\nA2 CST: "<<nt2<<std::endl;
    //Simulator::Schedule(Seconds(0.1), &calculateCST, wifiPhy, apDevices, wifiHelper, wifiMac, ap);
    Simulator::Schedule(Seconds(0.1), &calculateCST, apDevices, ap, wifiMac, channel);


/*  wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
  address.SetBase ("10.0.0.0", "255.255.252.0");

  if (ap.Get(0)){
    nt0 = min (max (max0, min0) - 25, min0);
    wifiPhy.Set ("CcaMode1Threshold", DoubleValue (nt0));
    wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (nt0 + 3));

    apDevices = wifiHelper.Install (wifiPhy, wifiMac, ap.Get(0));

    stack.Install (ap.Get(0));

  } else if (ap.Get(1)){
      nt1 = min (max (max1, min1) - 25, min1);
      wifiPhy.Set ("CcaMode1Threshold", DoubleValue (nt1));
      wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (nt1 + 3));

      apDevices = wifiHelper.Install (wifiPhy, wifiMac, ap.Get(1));
      stack.Install (ap.Get(1));


  } else if (ap.Get(2)){
      nt2 = min (max (max2, min2) - 25, min2);
      //nt2 = max (max2, min2);
      wifiPhy.Set ("CcaMode1Threshold", DoubleValue (nt2));
      wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (nt2 + 3));

      apDevices = wifiHelper.Install (wifiPhy, wifiMac, ap.Get(2));
      stack.Install (ap.Get(2));
  }
  apInterface = address.Assign(apDevices);*/


    //std::cout<<"\nA2 CST: "<<nt2<<std::endl;
    //Simulator::Schedule(Seconds(0.1), &calculateCST, wifiPhy, wifiMac, wifiHelper, apDevices, ap, stack, address, apInterface);
}



int main (int argc, char *argv[])
{
    uint32_t payloadSize = 972;                        /* Transport layer payload size in bytes. */
    std::string dataRate = "72.2Mbps";                 /* Application layer datarate. */
    std::string phyRate = "HtMcs7";                    /* Physical layer bitrate. */
    double simulationTime = 2;                        /* Simulation time in seconds. */
    bool pcapTracing = false;                          /* PCAP Tracing is enabled or not. */
    uint32_t nWifi = 3;
    uint32_t nAP = 3;

    /* Command line argument parser setup. */
    CommandLine cmd;
    cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
    cmd.AddValue ("nAP", "Number of wifi AP devices", nWifi);
    cmd.AddValue ("payloadSize", "Payload size in bytes", payloadSize);
    cmd.AddValue ("dataRate", "Application data ate", dataRate);
    cmd.AddValue ("phyRate", "Physical layer bitrate", phyRate);
    cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
    cmd.AddValue ("pcap", "Enable/disable PCAP Tracing", pcapTracing);
    cmd.Parse (argc, argv);


    /* No fragmentation and no RTS/CTS */
    Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
    Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));

    WifiMacHelper wifiMac;
    WifiHelper wifiHelper;
    wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);

    /* Set up Legacy Channel */
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::HybridBuildingsPropagationLossModel", "Frequency", DoubleValue (2.4e9),
                                    "CitySize", EnumValue (1), "InternalWallLoss", DoubleValue (12));

    Ptr<Building> b = CreateObject<Building> ();
    b->SetBoundaries (Box (1.0, 30.0, 1.0, 10.0, 1.0, 3.0));
    b->SetBuildingType (Building::Residential);
    b->SetExtWallsType (Building::ConcreteWithWindows);
    b->SetNFloors(1);
    b->SetNRoomsX(3);
    b->SetNRoomsY(1);

    uint16_t x = b->GetNRoomsX();
    uint16_t y = b->GetNRoomsY();
    uint16_t z = b->GetNFloors();

    std::cout<<"\nSize of Building"<<std::endl;
    std::cout<<"-------------------------------------"<<std::endl;
    std::cout<<"# of rooms x-axis: "<<x<<std::endl;
    std::cout<<"# of rooms y-axis: "<<y<<std::endl;
    std::cout<<"# of rooms z-axis: "<<z<<std::endl;
    std::cout<<"# of total rooms: "<<x*y*z<<std::endl;
    std::cout<<"-------------------------------------\n"<<std::endl;


    /* Setup Physical Layer */
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
    wifiPhy.SetChannel (wifiChannel.Create ());
    //wifiPhy.Set ("Frequency", UintegerValue (2400000000));
    //wifiPhy.Set ("ChannelNumber", UintegerValue (1 + (nAP % 3) * 5));
    wifiPhy.Set ("ChannelWidth", UintegerValue (20));
    wifiPhy.Set ("TxPowerStart", DoubleValue (16.0));
    wifiPhy.Set ("TxPowerEnd", DoubleValue (16.0));
    wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
    wifiPhy.Set ("TxGain", DoubleValue (1));
    wifiPhy.Set ("RxGain", DoubleValue (1));
    wifiPhy.Set ("RxNoiseFigure", DoubleValue (7));
    wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-60));
    wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-60 + 3));
    wifiPhy.Set ("ShortGuardEnabled", BooleanValue (true));
    wifiPhy.Set ("ShortPlcpPreambleSupported", BooleanValue (true));
    wifiPhy.SetErrorRateModel ("ns3::YansErrorRateModel");
    wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                        "DataMode", StringValue (phyRate),
                                        "ControlMode", StringValue ("HtMcs0"));


    /* Create ALL, AP, STA */
    NodeContainer allNodes, ap, sta;
    //NodeContainer allNodes, sta;

    sta.Create (nWifi);
    allNodes.Add (sta);

    ap.Create (nAP);
    allNodes.Add (ap);


    /* Device setting */
    NetDeviceContainer apDevices, staDevices;
    Ssid ssid;
    ssid = Ssid ("network");

/*  for (uint32_t i = 0; i < nAP; ++i)
    {
      wifiPhy.Set ("ChannelNumber", UintegerValue (1 + (i % 3) * 5));
      std::cout<<"nAP :"<<i<<", Channel number: "<<(1 + (i % 3) * 5)<<std::endl;
    }*/

    /* Configure AP */
    wifiMac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid), "BeaconGeneration", BooleanValue (true), "BeaconInterval", TimeValue (MilliSeconds (100)), "HtSupported", BooleanValue (true));
    apDevices = wifiHelper.Install (wifiPhy, wifiMac, ap);

    /* Configure STA */
    wifiMac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid));
    staDevices = wifiHelper.Install (wifiPhy, wifiMac, sta);



    /* Mobility model */
    MobilityHelper mobility;

    Ptr<PositionAllocator> positionAlloc = CreateObject<RandomRoomPositionAllocator> ();
    mobility.SetPositionAllocator (positionAlloc);

    //mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel");


    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install (sta);
    mobility.Install (ap);

    BuildingsHelper::Install (sta);
    BuildingsHelper::Install (ap);
    BuildingsHelper::MakeMobilityModelConsistent();


    std::map<Room, uint32_t> roomCounter;

    for (NodeContainer::Iterator it = allNodes.Begin (); it != allNodes.End (); ++it)
    {
        Ptr<MobilityModel> mm = (*it)->GetObject<MobilityModel> ();
        Ptr<MobilityBuildingInfo> bmm = mm->GetObject<MobilityBuildingInfo> ();
        Ptr<Channel> cmm = (*it)->GetObject<Channel> ();

        Room r (bmm->GetRoomNumberX (), bmm->GetRoomNumberY (), bmm->GetFloorNumber ());
        ++(roomCounter[r]);

        Vector p = mm->GetPosition ();
        std::cout<<"Room Position(x,y,z): ("<<b->GetRoomX(p)<<", "<<b->GetRoomY(p)<<", "<<b->GetFloor(p)<<")"<<std::endl;
        std::cout<<"Node Position(x,y,z): ("<<p.x<<","<<p.y<<","<<p.z<<")"<<"IsInDoor: "<<bmm->IsIndoor ()<<"\n"<<std::endl;

        bmm->SetIndoor(b, b->GetFloor(p), b->GetRoomX(p), b->GetRoomY(p));

    }

    for (std::map<Room, uint32_t>::iterator it = roomCounter.begin (); it != roomCounter.end (); ++it)
    {
        std::cout<<"# of nodes per Room: "<<it->second<<std::endl;
    }



    /* Internet stack */
    InternetStackHelper stack;
    stack.Install (allNodes);

    Ipv4AddressHelper address;
    address.SetBase ("10.0.0.0", "255.255.252.0");

    Ipv4InterfaceContainer staInterface;
    staInterface = address.Assign(staDevices);

    Ipv4InterfaceContainer apInterface;
    apInterface = address.Assign(apDevices);


    /* Populate routing table */
    Ipv4GlobalRoutingHelper::PopulateRoutingTables ();


    ApplicationContainer clientApp, sinkApp;
    uint32_t portNumber = 9;
    for (uint8_t index = 0; index < nWifi; ++index){
        for (uint8_t i = 0; i < nAP; i++) {
            auto ipv4 = ap.Get(i)->GetObject<Ipv4> ();
            const auto address = ipv4->GetAddress (1, 0).GetLocal ();
            InetSocketAddress sinkSocket (address, portNumber++);
            OnOffHelper client ("ns3::UdpSocketFactory", sinkSocket);
            client.SetAttribute ("PacketSize", UintegerValue (payloadSize)); //bytes
            client.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
            client.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
            client.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
            clientApp.Add (client.Install (sta.Get (index)));

            PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", sinkSocket);
            //sinkApp.Add (sinkHelper.Install (ap));
            sinkApp.Add (sinkHelper.Install (ap.Get (i)));
        }
    }


    ShowNodeInformation(ap, sta, b);


//  serverApp.Start (Seconds (0.0));
    sinkApp.Start (Seconds (0.0));
    clientApp.Start (Seconds (1.0));

    Simulator::Schedule (Seconds (1.1), &signalMonitor);
//  Simulator::Schedule (Seconds (1.1), &CalculateThroughput);


    //Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx));
    Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx0));
    Config::ConnectWithoutContext ("/NodeList/1/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx1));
    Config::ConnectWithoutContext ("/NodeList/2/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx2));



    /* Enable Traces */
    if (pcapTracing)
    {
        wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
        wifiPhy.EnablePcap ("ap", apDevices);
        wifiPhy.EnablePcap ("sta", staDevices);
    }


    AnimationInterface anim ("wifi-tcp-nt.xml"); // Mandatory
    for (uint32_t i = 0; i < sta.GetN (); ++i)
    {
        anim.UpdateNodeDescription (sta.Get (i), "STA"); // Optional
        anim.UpdateNodeColor (sta.Get (i), 255, 0, 0); // Optional
        anim.SetMaxPktsPerTraceFile(50000);
    }
    for (uint32_t i = 0; i < ap.GetN (); ++i)
    {
        anim.UpdateNodeDescription (ap.Get (i), "AP"); // Optional
        anim.UpdateNodeColor (ap.Get (i), 0, 255, 0); // Optional
        anim.SetMaxPktsPerTraceFile(50000);
    }

    anim.EnablePacketMetadata (); // Optional
    anim.EnableWifiMacCounters (Seconds (0), Seconds (11)); //Optional
    anim.EnableWifiPhyCounters (Seconds (0), Seconds (11)); //Optional


    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor;
    flowMonitor = flowHelper.InstallAll();
    flowMonitor->SerializeToXmlFile("nt_flow.xml", true, true);

    ThroughputMonitor(&flowHelper, flowMonitor);
    //calculateCST(&wifiPhy, wifiMac, wifiHelper, apDevices, ap, stack, address, apInterface);
    //calculateCST(&wifiPhy, apDevices, wifiHelper, wifiMac, ap);
    calculateCST(apDevices, ap, wifiMac, wifiChannel);


    /* Start Simulation */
    Simulator::Stop (Seconds (simulationTime + 1));

    Simulator::Run ();


    uint64_t save = 0;
    double throughput = 0;
    for (unsigned index = 0; index < sinkApp.GetN (); ++index)
    {
        uint64_t totalPacketsThrough = DynamicCast<PacketSink> (sinkApp.Get (index))->GetTotalRx ();
        //throughput += ((totalPacketsThrough * 8) / (simulationTime * 1000000.0)); //Mbit/s
        throughput = ((totalPacketsThrough * 8) / (simulationTime * 1000000.0)); //Mbit/s
        //std::cout << "\nAggregated throughput: " << throughput << " Mbit/s" << std::endl;
        std::cout <<"sinkApp_Num"<< index <<" Aggregated throughput: " << throughput << " Mbit/s" << std::endl;
        save = save + throughput;
    }


    double averageThroughput = save / nAP;
    std::cout << "\nAverage throughput of APs: " << averageThroughput << " Mbit/s" << std::endl;

    Simulator::Destroy ();
    return 0;
}

