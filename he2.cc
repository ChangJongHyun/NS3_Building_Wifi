
#include "ns3/core-module.h"
#include "ns3/propagation-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/wifi-module.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>
#include <ns3/building.h>
#include <ns3/buildings-module.h>
#include <ns3/basic-energy-source.h>



NS_LOG_COMPONENT_DEFINE ("Main");

using namespace ns3;

#define Downlink true
#define Uplink false
#define PI 3.14159265
#define PI_e5 314158

Ptr<PacketSink> sink;
uint64_t lastTotalRx = 0;




class Experiment
{
    struct Room {
        Room(uint32_t xx, uint32_t yy, uint32_t zz);
        Room();
        uint32_t x;
        uint32_t y;
        uint32_t z;
        int idx;
        static int room_idx;

        NodeContainer nodes;

        uint32_t get_number() const {
            return x*100 + y*10 + z;
        }

        void Add(Ptr<Node> node) {
            nodes.Add(node);
        }

        void print_node_(){
            std::cout<<"ID of All node"<<::std::endl;
            for (uint32_t i = 0; i < nodes.GetN(); i++)
                std::cout<<nodes.Get(i)->GetId()<<std::endl;
        }
    };

public:
    Experiment(bool downlinkUplink);
    void SetRtsCts(bool enableCtsRts);
    void CreateNode(size_t in_ap, size_t in_nodeNumber);
    void InitialExperiment();
    void InstallApplication(size_t in_packetSize, size_t in_dataRate);
    void Run(size_t in_simTime);
    void PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr);
    void PhyRxOkTrace (std::string context, Ptr<const Packet> packet,
                       double snr, enum WifiPreamble preamble);
    void PhyTxTrace (std::string context, Ptr<const Packet> packet,
                     WifiPreamble preamble, uint8_t txPower);
    void ShowNodeInformation();

    void CreateBuilding(Box box, Building::BuildingType_t type,
                            Building::ExtWallsType_t wallsType,
                                 uint16_t RoomX, uint16_t RoomY, uint16_t RoomFloor);
    void AddNodeToRoom();

    ApplicationContainer m_sink_app;
    ApplicationContainer m_server_app;
private:
    void SetWifiChannel();
    void InstallDevices();
    void InstallIp();
    void InstallEnergy();

    int m_totalRoom;
    bool m_enableCtsRts;
    bool m_downlinkUplink;
    size_t m_apNumber;
    size_t m_staNumber;
    size_t m_rxOkCount;
    size_t m_rxErrorCount;
    size_t m_txOkCount;
    NodeContainer m_nodes;  // All node
    NodeContainer m_ap;
    NodeContainer m_sta;
    MobilityHelper m_mobility;
    Ptr<BasicEnergySource> m_energy_source;
    Ptr<Building> m_building;
    Ptr<WifiRadioEnergyModel> m_wifi_energy_model;
    Ptr<RandomRoomPositionAllocator> m_apPosAlloc;
    Ptr<RandomRoomPositionAllocator> m_staPosAlloc;
    YansWifiChannelHelper m_wifiChannel;
    WifiHelper m_wifi;
    YansWifiPhyHelper m_wifiPhy;
    NqosWifiMacHelper m_wifiMac;
    NetDeviceContainer m_ap_device;
    NetDeviceContainer m_sta_device;
    InternetStackHelper m_internet;
    Ipv4AddressHelper m_ipv4;
    Ipv4InterfaceContainer m_ap_interface;
    Ipv4InterfaceContainer m_sta_interface;
    std::string phyRate = "HtMcs7";
    Room rooms[];
};

Experiment::Experiment(bool in_downlinkUplink):
        m_downlinkUplink(in_downlinkUplink)
{
    m_rxOkCount = 0;
    m_rxErrorCount = 0;
    m_txOkCount = 0;
}

void
Experiment::InitialExperiment()
{
    SetWifiChannel();
    InstallDevices();
    InstallIp();
    InstallEnergy();
    SetRtsCts(true);

}

void
Experiment::SetRtsCts(bool in_enableCtsRts)
{
    m_enableCtsRts = in_enableCtsRts;
    UintegerValue ctsThr = (m_enableCtsRts ? UintegerValue (10) : UintegerValue (22000));
    Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);
}

void
Experiment::CreateBuilding(Box box, Building::BuildingType_t type,
                             Building::ExtWallsType_t wallsType,
                                   uint16_t RoomX, uint16_t RoomY, uint16_t RoomFloor) {

    Ptr <Building> b = CreateObject<Building>();
    b->SetBoundaries(box);
    b->SetBuildingType(type);
    b->SetExtWallsType(wallsType);
    b->SetNRoomsX(RoomX);
    b->SetNRoomsY(RoomY);
    b->SetNFloors(RoomFloor);

    m_totalRoom = b->GetNRoomsX() * b->GetNRoomsY() * b->GetNFloors();
    m_building = b;

    std::cout << "\nSize of Building" << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "# of rooms x-axis: " << b->GetNRoomsX() << std::endl;
    std::cout << "# of rooms y-axis: " << b->GetNRoomsY() << std::endl;
    std::cout << "# of rooms z-axis: " << b->GetNFloors() << std::endl;
    std::cout << "# of total rooms : " << m_totalRoom << std::endl;
    std::cout << "-------------------------------------\n" << std::endl;
}

void
Experiment::CreateNode(size_t in_ap, size_t in_staNumber)
{
    m_apNumber = in_ap;
    m_staNumber = in_staNumber;

    // Create Node
    m_ap.Create(m_apNumber);
    m_sta.Create(m_staNumber);
    // Mobile & Position Alloc
    m_apPosAlloc = CreateObject<RandomRoomPositionAllocator> ();
    m_staPosAlloc = CreateObject<RandomRoomPositionAllocator> ();
    //m_mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
    m_mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    m_mobility.SetPositionAllocator(m_apPosAlloc); // ap는고정, sta는 움직이고 싶으면 따로따로 설정!

    m_mobility.Install(m_sta);
    m_mobility.Install(m_ap);

    // make MobilityBuildingInfo to node
    BuildingsHelper::Install(m_sta);
    BuildingsHelper::Install(m_ap);

    m_nodes.Add(m_sta);
    m_nodes.Add(m_ap);
    // insert node to building
    for (auto it = m_nodes.Begin(); it != m_nodes.End(); ++it) {
        Ptr <MobilityModel> mm = (*it)->GetObject<MobilityModel>();
        Ptr <MobilityBuildingInfo> bmm = mm->GetObject<MobilityBuildingInfo>();
        Vector p = mm->GetPosition();
        bmm->SetIndoor(m_building, m_building->GetFloor(p), m_building->GetRoomX(p), m_building->GetRoomY(p));
    }
}

void
Experiment::SetWifiChannel()
{
    m_wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    m_wifiChannel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel", "Frequency", DoubleValue(2.4e9),
                                     "CitySize", EnumValue(1), /*"RooftopLevel", DoubleValue (15),*/ "InternalWallLoss",
                                     DoubleValue(12));
}

void
Experiment::InstallDevices()
{
    m_wifi.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
    Config::SetDefault ("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue (40.046));
    m_wifi.SetRemoteStationManager ("ns3::IdealWifiManager");
   /* m_wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager","DataMode", StringValue (phyRate),
    "ControlMode", StringValue("HtMcs0"));*/


    m_wifiPhy =  YansWifiPhyHelper::Default ();
    m_wifiPhy.SetChannel (m_wifiChannel.Create());
    m_wifiPhy.Set("ChannelWidth", UintegerValue(20));
    m_wifiPhy.Set("TxPowerStart", DoubleValue(16.0));
    m_wifiPhy.Set("TxPowerEnd", DoubleValue(16.0));
    m_wifiPhy.Set("TxPowerLevels", UintegerValue(1));
    m_wifiPhy.Set("TxGain", DoubleValue(1));
    m_wifiPhy.Set("RxGain", DoubleValue(1));
    m_wifiPhy.Set("RxNoiseFigure", DoubleValue(7));
    m_wifiPhy.Set("CcaMode1Threshold", DoubleValue(-80));
    m_wifiPhy.Set("EnergyDetectionThreshold", DoubleValue(-80 + 3));
    m_wifiPhy.Set("ShortGuardEnabled", BooleanValue(true));
    m_wifiPhy.Set("ShortPlcpPreambleSupported", BooleanValue(true));
    m_wifiPhy.SetErrorRateModel("ns3::YansErrorRateModel");


    Ssid ssid = Ssid("networkA");

    m_wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "HtSupported", BooleanValue(true));
    m_ap_device = m_wifi.Install(m_wifiPhy, m_wifiMac, m_ap);
    /* Configure STA */
    m_wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    m_sta_device = m_wifi.Install(m_wifiPhy, m_wifiMac, m_sta);

}

void
Experiment::InstallIp()
{
    m_internet.Install (m_ap);
    m_internet.Install(m_sta);

    m_ipv4.SetBase ("10.0.0.0", "255.0.0.0");

    m_ap_interface = m_ipv4.Assign(m_ap_device);
    m_sta_interface = m_ipv4.Assign(m_sta_device);
}
void
Experiment::InstallEnergy() {
    m_energy_source = CreateObject<BasicEnergySource>();
    m_wifi_energy_model = CreateObject<WifiRadioEnergyModel>();

    m_energy_source -> SetInitialEnergy(300);
    m_wifi_energy_model -> SetEnergySource(m_energy_source);
    m_energy_source -> AppendDeviceEnergyModel(m_wifi_energy_model);

    m_ap.Get(0) -> AggregateObject(m_energy_source);
}

void
Experiment::PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr)
{
    Ptr<Packet> m_currentPacket;
    WifiMacHeader hdr;
    m_currentPacket = packet->Copy();
    m_currentPacket->RemoveHeader (hdr);
    if(hdr.IsData()){
        m_rxErrorCount++;
    }
}

void
Experiment::PhyRxOkTrace (std::string context, Ptr<const Packet> packet,
                          double snr, enum WifiPreamble preamble)
{
    Ptr<Packet> m_currentPacket;
    WifiMacHeader hdr;

    m_currentPacket = packet->Copy();
    m_currentPacket->RemoveHeader (hdr);
    if(hdr.IsData()){
        m_rxOkCount++;
    }
}

void
Experiment::PhyTxTrace (std::string context, Ptr<const Packet> packet
        , WifiPreamble preamble, uint8_t txPower)
{
    Ptr<Packet> m_currentPacket;
    WifiMacHeader hdr;
    m_currentPacket = packet->Copy();
    m_currentPacket->RemoveHeader (hdr);
    if(hdr.IsData()){
        m_txOkCount++;
    }
}



void
Experiment::InstallApplication(size_t in_packetSize, size_t in_dataRate)
{

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    uint16_t port = 9;
    // Install UDP Receiver on ther access point

    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));

    UdpClientHelper client (m_sta_interface.GetAddress (0), port);
    client.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
    client.SetAttribute ("Interval", TimeValue (Time ("0.00001"))); //packets/s
    client.SetAttribute ("PacketSize", UintegerValue (in_packetSize));

    m_sink_app = client.Install (m_ap.Get(0));

    /* Install TCP/UDP Transmitter on the station */
    OnOffHelper server ("ns3::UdpSocketFactory", (InetSocketAddress (m_ap_interface.GetAddress (0), 9)));
    server.SetAttribute ("PacketSize", UintegerValue (in_packetSize));
    server.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
    server.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
    server.SetAttribute ("DataRate", DataRateValue (DataRate (in_dataRate)));
    m_server_app = server.Install(m_sta);

//UDP flow
   // uint16_t port = 9;
    /*UdpServerHelper server (port);
    m_server_app = server.Install (m_sta);


    UdpClientHelper client (m_sta_interface.GetAddress (0), port);
    client.SetAttribute ("Interval", TimeValue (Time ("0.00001"))); //packets/s
    client.SetAttribute ("PacketSize", UintegerValue (in_packetSize));
    m_sink_app = client.Install (m_ap);
    sink = StaticCast<PacketSink>(m_sink_app.Get(0));*/
}


void
Experiment::ShowNodeInformation()
{
    std::cout<<"---------------AP info---------------"<<std::endl;
    for(auto it = m_ap.Begin(); it != m_ap.End(); it++) {
        Ptr<MobilityModel> mm = (*it) -> GetObject<MobilityModel>();
        Ptr<Ipv4> ipv4 = (*it) -> GetObject<Ipv4>();
        Vector p = mm->GetPosition();

        std::cout<<" AP  id => "<<(*it) ->GetId()<<std::endl;
        std::cout<<" AP  ipv4 => " << ipv4->GetAddress(1, 0).GetLocal()<<std::endl;
        //std::cout<<" AP  Mac => "<<mac48->GetBssid()<<std::endl;
        std::cout<<" AP  Room Pos => (" <<m_building->GetRoomX(p)<<
                 ", "<<m_building->GetRoomY(p)<<", "<<m_building->GetFloor(p)<<")"<<std::endl;
        std::cout<<" AP App => "<<(*it)->GetNApplications()<<std::endl;
    }
    std::cout<<"\n---------------STA info---------------"<<std::endl;
    for(auto it = m_sta.Begin(); it != m_sta.End(); it++) {
        Ptr<MobilityModel> mm = (*it) -> GetObject<MobilityModel>();
        Ptr<Ipv4> ipv4 = (*it) -> GetObject<Ipv4>();
        Vector p = mm->GetPosition();

        std::cout<<" STA id => "<<(*it) ->GetId()<<std::endl;
        std::cout<<" STA ipv4 => " << ipv4->GetAddress(1, 0).GetLocal()<<std::endl;
        std::cout<<" STA Room Pos => (" <<m_building->GetRoomX(p)<<
                 ", "<<m_building->GetRoomY(p)<<", "<<m_building->GetFloor(p)<<")"<<std::endl;
        std::cout<<" STA App => "<<(*it)->GetApplication(0)->GetInstanceTypeId()<<std::endl;
    }
    std::cout<<"-------------------------------------\n"<<std::endl;
}

void
Experiment::Run(size_t in_simTime)
{
    // 8. Install FlowMonitor on all nodes
    m_sink_app.Start(Seconds(0.0));
    m_sink_app.Stop(Seconds(in_simTime + 1.0));
    m_server_app.Start(Seconds(0.0));
    m_server_app.Start(Seconds(in_simTime + 1.0));

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

    // 9. Run simulation
    Simulator::Stop (Seconds (in_simTime + 1));
    Simulator::Run ();

   // 10. Print per flow statistics
    monitor->CheckForLostPackets ();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
    double accumulatedThroughput = 0;
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i=stats.begin();
         i!=stats.end(); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        std::cout << "Flow " << i->first<< " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
        std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
        std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
        std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
        std::cout << "  Lost Packets: " << i->second.lostPackets << "\n";
        std::cout << "  Pkt Lost Ratio: " << ((double)i->second.txPackets-(double)i->second.rxPackets)/(double)i->second.txPackets << "\n";
        std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / in_simTime / 1024 / 1024  << " Mbps\n";
        accumulatedThroughput+=(i->second.rxBytes*8.0/in_simTime/1024/1024);
    }
    std::cout << "apNumber=" <<m_apNumber << " nodeNumber=" << m_staNumber << "\n" << std::flush;
    std::cout << "throughput=" << accumulatedThroughput << "\n" << std::flush;
    std::cout << "tx=" << m_txOkCount << " RXerror=" <<m_rxErrorCount <<
              " Rxok=" << m_rxOkCount << "\n" << std::flush;
    std::cout << "===========================\n" << std::flush;

    // 11. Cleanup
    Simulator::Destroy ();
}

void
CalculateThroughput() {
    Time now = Simulator::Now();                                            //* Return the simulator's virtual time. *//*
    double cur =
            (sink->GetTotalRx() - lastTotalRx) * (double) 8 / 1e5;     //* Convert Application RX Packets to MBits. *//*
    std::cout << now.GetSeconds() << "s: \t" << cur << " Mbit/s" << std::endl;
    lastTotalRx = sink->GetTotalRx();
    Simulator::Schedule(MilliSeconds(100), &CalculateThroughput);
}

int main (int argc, char **argv)
{
    size_t payload_size = 1472;
    size_t data_rate = 72200000;
    size_t simulationTime = 4;
 //   size_t numOfAp[6] = {1, 2, 3, 4, 5, 6};
    //double range[4] = {60, 120, 180, 240};

    Experiment exp(Downlink);
    exp.CreateBuilding(Box(1, 100, 1, 20, 1, 15),
            Building::Residential, Building::ConcreteWithWindows, 2, 3, 2);
    exp.CreateNode(12, 36);
    exp.InitialExperiment();
    exp.InstallApplication(payload_size, data_rate);
    exp.ShowNodeInformation();

/*    exp.m_sink_app.Start(Seconds(0.0));
    exp.m_sink_app.Stop(Seconds(simulationTime + 1.0));
    exp.m_server_app.Start(Seconds(1.0));
    exp.m_server_app.Start(Seconds(simulationTime + 1.0));
    Simulator::Schedule(Seconds(1.1), & CalculateThroughput);
    Simulator::Stop (Seconds (simulationTime + 1));
    Simulator::Run ();
    Simulator::Destroy();*/

    exp.Run(4);

    double throughput = 0;

    for (uint32_t index = 0; index < exp.m_sink_app.GetN(); ++index) {
        uint64_t totalPacketsThrough = DynamicCast<PacketSink> (exp.m_sink_app.Get (index))->GetTotalRx ();
        throughput += ((totalPacketsThrough * 8) / (simulationTime * 1000000.0)); //Mbit/s
    }

    if (throughput > 0)
        std::cout << "Aggregated throughput: " << throughput << " Mbit/s" << std::endl;
    else
        NS_LOG_ERROR("Obtained throught is 0!");

    return 0;
}