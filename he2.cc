
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

NS_LOG_COMPONENT_DEFINE ("Main");

using namespace ns3;

#define Downlink true
#define Uplink false
#define PI 3.14159265
#define PI_e5 314158

class Experiment
{
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
    void ShowNodeInformation(NodeContainer in_c, size_t in_numOfNode);

    void CreateBuilding(Box box, Building::BuildingType_t type,
                            Building::ExtWallsType_t wallsType,
                                 uint16_t RoomX, uint16_t RoomY, uint16_t RoomFloor);

private:
    void SetWifiChannel();
    void InstallDevices();
    void InstallIp();

    bool m_enableCtsRts;
    bool m_downlinkUplink;
    size_t m_apNumber;
    size_t m_staNumber;
    double m_radius;
    size_t m_rxOkCount;
    size_t m_rxErrorCount;
    size_t m_txOkCount;
    std::vector<std::vector<double> > m_readChannelGain;
    std::vector<int> m_serveBy;
    NodeContainer m_nodes;  // All node
    NodeContainer m_ap;
    NodeContainer m_sta;
    MobilityHelper m_mobility;
    Ptr<RandomRoomPositionAllocator> m_apPosAlloc;
    Ptr<RandomRoomPositionAllocator> m_staPosAllocPosAlloc;
    Ptr<Building> m_building;
    YansWifiChannelHelper m_wifiChannel;
    WifiHelper m_wifi;
    YansWifiPhyHelper m_wifiPhy;
    NqosWifiMacHelper m_wifiMac;
    NetDeviceContainer m_devices;
    InternetStackHelper m_internet;
    Ipv4AddressHelper m_ipv4;
    ApplicationContainer m_cbrApps;
    ApplicationContainer m_pingApps;
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
   // InstallIp();
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

    Ptr <Building> building = CreateObject<Building>();
    building->SetBoundaries(box);
    building->SetBuildingType(type);
    building->SetExtWallsType(wallsType);
    building->SetNRoomsX(RoomX);
    building->SetNRoomsY(RoomY);
    building->SetNFloors(RoomFloor);

    std::cout << "\nSize of Building" << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "# of rooms x-axis: " << building->GetNRoomsX() << std::endl;
    std::cout << "# of rooms y-axis: " << building->GetNRoomsY() << std::endl;
    std::cout << "# of rooms z-axis: " << building->GetNFloors() << std::endl;
    std::cout << "# of total rooms : " << building->GetNRoomsX() * building->GetNFloors() * building-> GetNRoomsY() << std::endl;
    std::cout << "-------------------------------------\n" << std::endl;

    m_building = building;
}

void
Experiment::CreateNode(size_t in_ap, size_t in_staNumber)
{
    if(m_building == NULL)
        std::cout<<"no"<<std::endl;

    m_apNumber = in_ap;
    m_staNumber = in_staNumber;

    // Create Node
    m_ap.Create(m_apNumber);
    m_sta.Create(m_staNumber);
    // Mobile & Position Alloc
    m_apPosAlloc = CreateObject<RandomRoomPositionAllocator> ();
    m_staPosAllocPosAlloc = CreateObject<RandomRoomPositionAllocator> ();
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
    m_wifi.SetRemoteStationManager ("ns3::IdealWifiManager");

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

    NqosWifiMacHelper m_wifiMac;
    m_wifiMac.SetType ("ns3::AdhocWifiMac"); // use ad-hoc MAC
    m_devices = m_wifi.Install (m_wifiPhy, m_wifiMac, m_ap);
    m_devices = m_wifi.Install (m_wifiPhy, m_wifiMac, m_sta);
    std::cout<<"he "<<m_devices.GetN()<<std::endl;
}

void
Experiment::InstallIp()
{
    m_internet.Install (m_nodes);
    m_ipv4.SetBase ("10.0.0.0", "255.0.0.0");
    m_ipv4.Assign (m_devices);
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
    uint16_t cbrPort = 12345;
    for(size_t j=1; j<=m_apNumber; ++j){
        for(size_t i=m_apNumber+m_staNumber/m_apNumber*(j-1);
            i<m_apNumber+m_staNumber/m_apNumber*j ; ++i){
            std::string s;
            std::stringstream ss(s);
            if(m_downlinkUplink){
                ss << i+1;
            }else
            {
                ss << j;
            }
            s = "10.0.0."+ss.str();
            OnOffHelper onOffHelper ("ns3::UdpSocketFactory",
                                     InetSocketAddress (Ipv4Address (s.c_str()), cbrPort));
            onOffHelper.SetAttribute ("PacketSize", UintegerValue (in_packetSize));
//onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
            std::string s2;
            std::stringstream ss2(s2);
            if(m_downlinkUplink){
                ss2 << in_dataRate+i*100;
            }else
            {
                ss2 << in_dataRate+i*100;
            }
            s2 = ss2.str() + "bps";
            onOffHelper.SetAttribute ("DataRate", StringValue (s2));
            if(m_downlinkUplink){
                onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (1.00+static_cast<double>(i)/100)));
                onOffHelper.SetAttribute ("StopTime", TimeValue (Seconds (50.000+static_cast<double>(i)/100)));
                m_cbrApps.Add (onOffHelper.Install (m_nodes.Get (j-1)));
            }else
            {
                onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (1.00)));
                onOffHelper.SetAttribute ("StopTime", TimeValue (Seconds (50.000+static_cast<double>(j)/100)));
                m_cbrApps.Add (onOffHelper.Install (m_nodes.Get (i)));
            }
        }
    }
    uint16_t  echoPort = 9;
    // again using different start times to workaround Bug 388 and Bug 912
    for(size_t j=1; j<=m_apNumber; ++j){
        for(size_t i=m_apNumber+m_staNumber/m_apNumber*(j-1);
            i<m_apNumber+m_staNumber/m_apNumber*j ; ++i){
            std::string s;
            std::stringstream ss(s);
            if(m_downlinkUplink){
                ss << i+1;
            }else
            {
                ss << j;
            }
            s = "10.0.0."+ss.str();
            UdpEchoClientHelper echoClientHelper (Ipv4Address (s.c_str()), echoPort);
            echoClientHelper.SetAttribute ("MaxPackets", UintegerValue (1));
            echoClientHelper.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
            echoClientHelper.SetAttribute ("PacketSize", UintegerValue (10));
            if(m_downlinkUplink){
                echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001)));
                echoClientHelper.SetAttribute ("StopTime", TimeValue (Seconds (50.001)));
                m_pingApps.Add (echoClientHelper.Install (m_nodes.Get (j-1)));
            }else
            {
                echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001)));
                echoClientHelper.SetAttribute ("StopTime", TimeValue (Seconds (50.001)));
                m_pingApps.Add (echoClientHelper.Install (m_nodes.Get (i)));
            }
        }
    }
}


void
Experiment::ShowNodeInformation(NodeContainer in_c, size_t in_numOfNode)
{
    for(size_t i=0; i<in_numOfNode; ++i){
        Ptr<MobilityModel> mobility = in_c.Get(i)->GetObject<MobilityModel> ();
        Vector nodePos = mobility->GetPosition ();
        // Get Ipv4 instance of the node
        Ptr<Ipv4> ipv4 = in_c.Get(i)->GetObject<Ipv4> ();
        // Get Ipv4 instance of the node
        Ptr<MacLow> mac48 = in_c.Get(i)->GetObject<MacLow> ();
        // Get Ipv4InterfaceAddress of xth interface.
        Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal ();
        //Mac48Address macAddr = mac48->GetAddress();
        std::cout << in_c.Get(i)->GetId() << " " << addr << " (" << nodePos.x << ", " <<
                  nodePos.y << ")" << std::endl;
    }
}

void
Experiment::Run(size_t in_simTime)
{
    // 8. Install FlowMonitor on all nodes
    ShowNodeInformation(m_nodes, m_apNumber+m_staNumber);

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

    // 9. Run simulation
    Simulator::Stop (Seconds (in_simTime));
    Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxError",
                     MakeCallback (&Experiment::PhyRxErrorTrace, this));
    Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxOk",
                     MakeCallback (&Experiment::PhyRxOkTrace, this));
    Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/Tx",
                     MakeCallback (&Experiment::PhyTxTrace, this));
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

int main (int argc, char **argv)
{

 //   size_t numOfAp[6] = {1, 2, 3, 4, 5, 6};
    //double range[4] = {60, 120, 180, 240};

    Experiment exp(Downlink);
    exp.CreateBuilding(Box(1, 100, 1, 20, 1, 15),
            Building::Residential, Building::ConcreteWithWindows, 2, 3, 1);
    exp.CreateNode(6, 12);
    exp.InitialExperiment();
    return 0;
}