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
#include "ns3/point-to-point-module.h"
#include "ns3/basic-energy-source.h"
#include "ns3/simple-device-energy-model.h"
#include <sys/stat.h>
#include <unistd.h>
#include "ns3/yans-wifi-phy.h"
#include <ns3/mobility-helper.h>
#include "ns3/ipv4.h"
#include "ns3/flow-monitor-module.h"

NS_LOG_COMPONENT_DEFINE ("wifi-tcp-nt");

using namespace ns3;

Ptr<PacketSink> sink;
uint64_t lastTotalRx = 0;

bool
ReceivePacket(Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol, const Address &address) {

    std::cout<<"packet: "<<packet->GetSize()<<std::endl;
    return true;
}

void
CalculateThroughput() {
    Time now = Simulator::Now();
    //double car = sink->GetTotalRx ();
    double cor = (sink->GetTotalRx () - lastTotalRx);
    double cur =
            (sink->GetTotalRx() - lastTotalRx) * (double) 8 / 1e5;
    //std::cout << now.GetSeconds () << "s: \t" << "RX Total Packets= " << car << std::endl;
    std::cout << now.GetSeconds () << "s: \t" << "RX Packets= " << cor <<",  Throughput= " << cur << " Mbit/s" << std::endl;
    lastTotalRx = sink->GetTotalRx();
    Simulator::Schedule(MilliSeconds(100), & CalculateThroughput);
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
                std::cout << " AP  App    => (" << i << ")" << (*it)->GetApplication(i)->GetInstanceTypeId()
                          << std::endl;
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
                std::cout << " STA App    => (" << i << ")" << (*it)->GetApplication(i)->GetInstanceTypeId()
                          << std::endl;
        } else {
            std::cout << " STA App    => " << (*it)->GetNApplications() << std::endl;
        }
    }
    std::cout << "-------------------------------------\n" << std::endl;

    std::cout << "----------wifi channel info----------" << std::endl;
    std::cout << "---------------AP--------------------" << std::endl;
    Ptr<NetDevice> net = m_sta.Get(0)->GetDevice(0);
    Ptr<WifiNetDevice> wifi = StaticCast<WifiNetDevice>(net);

    Ptr<WifiPhy> wifiPhy = wifi->GetPhy();
    Ptr<WifiMac> wifiMac = wifi->GetMac();
    std::cout << "Wifi info" << std::endl;
    std::cout << " # Channel: " << wifi->GetChannel()->GetId() << std::endl;
    std::cout << " # MTU: " << wifi->GetMtu() << std::endl;
    std::cout << "Phy info" << std::endl;
    std::cout << " # Standard: " << wifiPhy->GetStandard() << std::endl;
    std::cout << " # Channel width: " << wifiPhy->GetChannelWidth() << std::endl;
    std::cout << " # Freq: " << wifiPhy->GetFrequency() << std::endl;
    std::cout << " # Guard Interval: " << wifiPhy->GetGuardInterval() << std::endl;
    std::cout << " # # of Rx Antenna: " << wifiPhy->GetNumberOfReceiveAntennas() << std::endl;
    std::cout << " # # of Tx Antenna: " << wifiPhy->GetNumberOfTransmitAntennas() << std::endl;
    std::cout << " # Rx Gain: " << wifiPhy->GetRxGain() << std::endl;
    std::cout << " # Tx Gain: " << wifiPhy->GetTxGain() << std::endl;
    std::cout << " # Rx noise: " << wifiPhy->GetRxNoiseFigure() << std::endl;
    std::cout << "-------------------------------------" << std::endl;

    std::cout << "---------------STA 01----------------" << std::endl;
    Ptr<NetDevice> net2 = m_ap.Get(0)->GetDevice(0);
    Ptr<WifiNetDevice> wifi2 = StaticCast<WifiNetDevice>(net2);
    Ptr<WifiPhy> wifiPhy2 = wifi2->GetPhy();
    Ptr<WifiMac> wifiMac2 = wifi2->GetMac();
    std::cout << "Wifi info" << std::endl;
    std::cout << " # Channel: " << wifi2->GetChannel()->GetId() << std::endl;
    std::cout << " # MTU: " << wifi2->GetMtu() << std::endl;
    std::cout << "Phy info" << std::endl;
    std::cout << " # Standard: " << wifiPhy2->GetStandard() << std::endl;
    std::cout << " # Channel width: " << wifiPhy2->GetChannelWidth() << std::endl;
    std::cout << " # Freq: " << wifiPhy2->GetFrequency() << std::endl;
    std::cout << " # Guard Interval: " << wifiPhy2->GetGuardInterval() << std::endl;
    std::cout << " # # of Rx Antenna: " << wifiPhy2->GetNumberOfReceiveAntennas() << std::endl;
    std::cout << " # # of Tx Antenna: " << wifiPhy2->GetNumberOfTransmitAntennas() << std::endl;
    std::cout << " # Rx Gain: " << wifiPhy2->GetRxGain() << std::endl;
    std::cout << " # Tx Gain: " << wifiPhy2->GetTxGain() << std::endl;
    std::cout << " # Rx noise: " << wifiPhy2->GetRxNoiseFigure() << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "---------------STA 02----------------" << std::endl;
    Ptr<NetDevice> net3 = m_ap.Get(0)->GetDevice(0);
    Ptr<WifiNetDevice> wifi3 = StaticCast<WifiNetDevice>(net2);
    Ptr<WifiPhy> wifiPhy3 = wifi3->GetPhy();
    Ptr<WifiMac> wifiMac3 = wifi3->GetMac();
    std::cout << "Wifi info" << std::endl;
    std::cout << " # Channel: " << wifi3->GetChannel()->GetId() << std::endl;
    std::cout << " # MTU: " << wifi3->GetMtu() << std::endl;
    std::cout << "Phy info" << std::endl;
    std::cout << " # Standard: " << wifiPhy3->GetStandard() << std::endl;
    std::cout << " # Channel width: " << wifiPhy3->GetChannelWidth() << std::endl;
    std::cout << " # Freq: " << wifiPhy3->GetFrequency() << std::endl;
    std::cout << " # Guard Interval: " << wifiPhy3->GetGuardInterval() << std::endl;
    std::cout << " # # of Rx Antenna: " << wifiPhy3->GetNumberOfReceiveAntennas() << std::endl;
    std::cout << " # # of Tx Antenna: " << wifiPhy3->GetNumberOfTransmitAntennas() << std::endl;
    std::cout << " # Rx Gain: " << wifiPhy3->GetRxGain() << std::endl;
    std::cout << " # Tx Gain: " << wifiPhy3->GetTxGain() << std::endl;
    std::cout << " # Rx noise: " << wifiPhy3->GetRxNoiseFigure() << std::endl;
    std::cout << "-------------------------------------" << std::endl;

}


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
        //std::cout << "Throughput: " <<i->second.rxBytes * 8.0 / (1e6 * simulationTime)<<"\n";
        std::cout << "Throughput: " <<i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds()-i->second.timeFirstTxPacket.GetSeconds())/1024/1024<<" Mbps\n";
        std::cout << "Packet Loss Ratio: " << (i->second.txPackets -i->second.rxPackets)*100/(double)i->second.txPackets << std::endl;
    }
    Simulator::Schedule(Seconds(0.5), &ThroughputMonitor, flowHelper, flowMonitor);
}



/*void
Rxview ()
{
  Time now = Simulator::Now ();
  std::cout << now.GetSeconds () << "s: \t" << sink->rxTrace << " Mbit/s" << std::endl;
  Simulator::Schedule (MilliSeconds (1), &Rxview);
}*/


/*int
CalculateCST ()
{
  Time now = Simulator::Now ();
  if (MilliSeconds > 50) {
    CcaMode1Threshold = -75;
  }
}*/


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
bool
operator < (const Room& a, const Room& b)
{
    return ( (a.x < b.x) || ( (a.x == b.x) && (a.y < b.y) ) || ( (a.x == b.x) && (a.y == b.y) && (a.z < b.z) ));
}

double g_signalDbmAvg;
double g_noiseDbmAvg;
uint32_t g_samples;
WifiMacHeader hdr;
Mac48Address addrmp;


int main (int argc, char *argv[])
{
    uint32_t payloadSize = 972;                        /* Transport layer payload size in bytes. */
    std::string dataRate = "72.2Mbps";                 /* Application layer datarate. */
    std::string phyRate = "HtMcs7";                    /* Physical layer bitrate. */
    double simulationTime = 2;                        /* Simulation time in seconds. */
    bool pcapTracing = false;                          /* PCAP Tracing is enabled or not. */
    uint32_t nWifi = 2;
    uint32_t nAP = 1;

    /* Command line argument parser setup. */
    CommandLine cmd;
    cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
    //cmd.AddValue ("udp", "UDP if set to 1, TCP otherwise", udp);
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
    b->SetBoundaries (Box (1.0, 10.0, 1.0, 10.0, 1.0, 3.0));
    b->SetBuildingType (Building::Residential);
    b->SetExtWallsType (Building::ConcreteWithWindows);
    b->SetNFloors(1);
    b->SetNRoomsX(1);
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
    wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-70));
    wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-70 + 3));
    wifiPhy.Set ("ShortGuardEnabled", BooleanValue (true));
    wifiPhy.Set ("ShortPlcpPreambleSupported", BooleanValue (true));
    wifiPhy.SetErrorRateModel ("ns3::YansErrorRateModel");
    wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                        "DataMode", StringValue (phyRate),
                                        "ControlMode", StringValue ("HtMcs0"));

    /*for (uint32_t cAP = 1; cAP < nAP+1; ++cAP)
      {
        wifiPhy.Set ("ChannelNumber", UintegerValue (1 + (cAP % 3) * 5));
        std::cout<<"nAP :"<<cAP<<", Channel number: "<<(1 + (cAP % 3) * 5)<<std::endl;
      }*/

    //wifiHelper.SetRemoteStationManager ("ns3::IdealWifiManager");


    /* Create ALL, AP, STA, P2P, CSMA nodes */
    NodeContainer allNodes, ap, sta;

    sta.Create (nWifi);
    allNodes.Add (sta);

    ap.Create (nAP);
    allNodes.Add (ap);


    /* Device setting */
    NetDeviceContainer apDevices, staDevices;
    Ssid ssid;


    ssid = Ssid ("network");
    /* Configure AP */
    wifiMac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid), "BeaconGeneration", BooleanValue (true), "BeaconInterval", TimeValue (MilliSeconds (100)), "HtSupported", BooleanValue (true));
    apDevices = wifiHelper.Install (wifiPhy, wifiMac, ap);

    /* Configure STA */
    wifiMac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid));
    staDevices = wifiHelper.Install (wifiPhy, wifiMac, sta);






    /* Configure P2P */
    /*p2pNodes.Add (ap);
    p2pNodes.Create (1);
    allNodes.Add (p2pNodes.Get (1));

    PointToPointHelper pointToPoint;
    p2pDevices = pointToPoint.Install (p2pNodes);*/

    /* Configure CSMA */
/*  csmaNodes.Add (p2pNodes.Get (1));
  csmaNodes.Create (1);
  allNodes.Add (csmaNodes.Get (1));

  CsmaHelper csma;
  csmaDevices = csma.Install (csmaNodes);*/



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
    //BuildingsHelper::Install (p2pNodes);
    //BuildingsHelper::Install (csmaNodes);
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

    //Ipv4GlobalRoutingHelper g;
    //Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("RoutingTable", std::ios::out);
    //g.PrintRoutingTableALLAt (Seconds (0.1), routingStream);


    /* Install TCP Receiver on the access point */
//  PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 9));

//  ApplicationContainer sinkApp = sinkHelper.Install (ap);
//  sink = DynamicCast<PacketSink> (sinkApp.Get (0));
    //sink = StaticCast<PacketSink> (sinkApp.Get(0));


    /* Install TCP/UDP Transmitter on the station */
//  OnOffHelper server ("ns3::UdpSocketFactory", (InetSocketAddress (apInterface.GetAddress (0), 9)));
//  server.SetAttribute ("PacketSize", UintegerValue (payloadSize));
//  server.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  server.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  server.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
//  ApplicationContainer serverApp = server.Install (sta);



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
            sinkApp.Add (sinkHelper.Install (ap));
            //sinkApp.Add (sinkHelper.Install (ap.Get (i)));
        }
    }


/*  ApplicationContainer clientApp, serverApp;
  uint32_t portNumber = 9;
  for (uint8_t index = 0; index < nWifi; ++index){
    for (uint8_t i = 0; i < nAP; i++) {
      auto ipv4 = ap.Get(i)->GetObject<Ipv4> ();
      const auto address = ipv4->GetAddress (1, 0).GetLocal ();
      InetSocketAddress sinkSocket (address, portNumber++);

      UdpServerHelper server(portNumber);
      serverApp = server.Install (ap);

      UdpClientHelper client (apInterface.GetAddress (0), portNumber);
      client.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
      client.SetAttribute ("Interval", TimeValue (Time ("0.00001"))); //packets/s
      client.SetAttribute ("PacketSize", UintegerValue (payloadSize));
      ApplicationContainer clientApp = client.Install (sta.Get (index));

    }
  }
*/

    sink = StaticCast<PacketSink> (sinkApp.Get(0));

    for (auto it = allNodes.Begin(); it != allNodes.End(); it++) {
        Ptr<NetDevice> device = (*it)->GetDevice(0);
        NetDevice::ReceiveCallback callback;
        callback = MakeCallback(&ReceivePacket);
        device->SetReceiveCallback(callback);
    }
    /*ApplicationContainer serverApps;
    UdpServerHelper myServer (9);
    serverApps = myServer.Install (sta);

    UdpClientHelper myClient (staInterface.GetAddress (0), 9);
    myClient.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
    myClient.SetAttribute ("Interval", TimeValue (Time ("0.00002"))); //packets/s
    myClient.SetAttribute ("PacketSize", UintegerValue (payloadSize));

    ApplicationContainer clientApps = myClient.Install (ap);*/



//  serverApp.Start (Seconds (0.0));
    sinkApp.Start (Seconds (0.0));
    clientApp.Start (Seconds (1.0));

//  Simulator::Schedule (Seconds (1.1), &CalculateThroughput);

    //Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx)); // This code works. but, can not see in command window.
    //Config::Connect("/NodeList/0/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx)); This code does not works.


    //Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Rx", MakeCallback (&RxTrace)); // This code works.


    /* Enable Traces */
/*  if (pcapTracing)
    {
      wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
      wifiPhy.EnablePcap ("AccessPoint1", apDevices1);
      wifiPhy.EnablePcap ("Station1", staDevices1);
      wifiPhy.EnablePcap ("AccessPoint2", apDevices2);
      wifiPhy.EnablePcap ("Station2", staDevices2);
    }
*/


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

    ShowNodeInformation(ap, sta, b);

/*    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor;
    flowMonitor = flowHelper.InstallAll();
    flowMonitor->SerializeToXmlFile("nt_flow.xml", true, true);

    ThroughputMonitor(&flowHelper, flowMonitor);*/
    Simulator::Schedule(Seconds(1.1), & CalculateThroughput);

    /* Start Simulation */
    Simulator::Stop (Seconds (simulationTime + 1));

    Simulator::Run ();

/*
  flowMonitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowHelper.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
    std::cout << "Flow: "<< i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
    std::cout << "Tx Packets: " <<i->second.txPackets << "\n";
    std::cout << "Rx Packets: " <<i->second.rxPackets << "\n";
    std::cout << "Throughput: " <<i->second.rxBytes * 8.0 / (1e6 * simulationTime)<<"\n";
    //std::cout << "Throughput: " <<i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds()-i->second.timeFirstTxPacket.GetSeconds())/72200<<"\n";
    std::cout << "Packet Loss Ratio: " << (i->second.txPackets -i->second.rxPackets)*100/(double)i->second.txPackets << std::endl;
  }
*/

    uint64_t save = 0;
    double throughput = 0;
    for (unsigned index = 0; index < sinkApp.GetN (); ++index)
    {
        uint64_t totalPacketsThrough = DynamicCast<PacketSink> (sinkApp.Get (index))->GetTotalRx ();
        throughput += ((totalPacketsThrough * 8) / (simulationTime * 1000000.0)); //Mbit/s
        //std::cout << "\nAggregated throughput: " << throughput << " Mbit/s" << std::endl;
        save = save + throughput;
    }
    double averageThroughput = save / sinkApp.GetN ();
    std::cout << "\nAverage throughput: " << averageThroughput << " Mbit/s" << std::endl;


/*  double throughput = 0;
  for (unsigned index = 0; index < serverApp.GetN (); ++index)
    {
      uint64_t totalPacketsThrough = DynamicCast<UdpServer> (serverApp.Get (0))->GetReceived ();
      throughput = totalPacketsThrough * payloadSize * 8 / (simulationTime * 1000000.0); //Mbit/s
      std::cout << "\n throughput: " << throughput << " Mbit/s" << std::endl;
      //save = save + throughput;
    }
*/





    //double averageThroughput2 = ((sink->GetTotalRx () *  8) / (1e6  * simulationTime));
    //std::cout << "\nAverage throughput2: " << averageThroughput2 << " Mbit/s" << std::endl;

    Simulator::Destroy ();
    return 0;
}

