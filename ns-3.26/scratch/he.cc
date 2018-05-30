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

NS_LOG_COMPONENT_DEFINE ("HelloSimulator");
using namespace ns3;

Ptr <PacketSink> sink;
uint64_t lastTotalRx = 0;

Ipv4Address
getIpv4FromNoe(Ptr<Node> node) {
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1,0);
    return iaddr.GetLocal();
}

uint32_t
get_room_to_node(Ptr<Node> node) {
    Ptr <MobilityModel> mm = node->GetObject<MobilityModel>();
    Ptr <MobilityBuildingInfo> bmm = mm->GetObject<MobilityBuildingInfo>();

    return bmm->GetRoomNumberX()*100 + bmm->GetRoomNumberY()*10 + bmm->GetFloorNumber();
}

void
CalculateThroughput() {
    Time now = Simulator::Now();                                            /* Return the simulator's virtual time. */
    double cur =
            (sink->GetTotalRx() - lastTotalRx) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
    std::cout << now.GetSeconds() << "s: \t" << cur << " Mbit/s" << std::endl;
    lastTotalRx = sink->GetTotalRx();
    Simulator::Schedule(MilliSeconds(100), & CalculateThroughput);
}


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

int Room::room_idx = 1;

Room::Room(uint32_t xx, uint32_t yy, uint32_t zz)
        : x(xx), y(yy), z(zz){
    idx =room_idx;
    room_idx++;
}
Room::Room() {
    x = 0;
    y = 0;
    z = 0;
    idx = 0;
}

bool
operator<(const Room &a, const Room &b) {
    return ((a.x < b.x) || ((a.x == b.x) && (a.y < b.y)) || ((a.x == b.x) && (a.y == b.y) && (a.z < b.z)));
}

int
main(int argc, char *argv[]) {

    //uint32_t payloadSize = 1472;                     /* Transport layer payload size in bytes. */
    uint32_t payloadSize = 972;                        /* Transport layer payload size in bytes. */
    std::string dataRate = "72.2Mbps";                 /* Application layer datarate. */
    //std::string tcpVariant = "TcpNewReno";             /* TCP variant type. */
    std::string phyRate = "HtMcs7";                    /* Physical layer bitrate. */
    double simulationTime = 2;                        /* Simulation time in seconds. */

    int staNum = 5;

    Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("999999"));
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("999999"));

    // AP Setting
    WifiMacHelper wifiMac;
    WifiHelper wifiHelper;
    wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211n_2_4GHZ);

    /* Set up Legacy Channel */
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    //wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (5e9));
    wifiChannel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel", "Frequency", DoubleValue(2.4e9),
                                   "CitySize", EnumValue(1), /*"RooftopLevel", DoubleValue (15),*/ "InternalWallLoss",
                                   DoubleValue(12));

    YansWifiChannelHelper wifiChannel2;
    wifiChannel2.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel2.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel", "Frequency", DoubleValue(2.4e9),
                                    "CitySize", EnumValue(1), /*"RooftopLevel", DoubleValue (15),*/ "InternalWallLoss",
                                    DoubleValue(12));

    /* Setup Physical Layer */
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    //wifiPhy.Set ("Frequency", UintegerValue (2400000000));
    //wifiPhy.set ("ChannelNumber", UintegerValue (1, 6, 11));
    wifiPhy.Set("ChannelWidth", UintegerValue(20));
    wifiPhy.Set("TxPowerStart", DoubleValue(16.0));
    wifiPhy.Set("TxPowerEnd", DoubleValue(16.0));
    wifiPhy.Set("TxPowerLevels", UintegerValue(1));
    wifiPhy.Set("TxGain", DoubleValue(1));
    wifiPhy.Set("RxGain", DoubleValue(1));
    wifiPhy.Set("RxNoiseFigure", DoubleValue(7));
    wifiPhy.Set("CcaMode1Threshold", DoubleValue(-80));
    wifiPhy.Set("EnergyDetectionThreshold", DoubleValue(-80 + 3));
    wifiPhy.Set("ShortGuardEnabled", BooleanValue(true));
    wifiPhy.Set("ShortPlcpPreambleSupported", BooleanValue(true));
    wifiPhy.SetErrorRateModel("ns3::YansErrorRateModel");
    /*wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                        "DataMode", StringValue (phyRate),
                                        "ControlMode", StringValue ("HtMcs0"));*/
    wifiHelper.SetRemoteStationManager("ns3::IdealWifiManager");


    YansWifiPhyHelper wifiPhy2 = YansWifiPhyHelper::Default();
    wifiPhy2.SetChannel(wifiChannel2.Create());
    //wifiPhy.Set ("Frequency", UintegerValue (2400000000));
    //wifiPhy.set ("ChannelNumber", UintegerValue (1, 6, 11));
    wifiPhy2.Set("ChannelWidth", UintegerValue(20));
    wifiPhy2.Set("TxPowerStart", DoubleValue(16.0));
    wifiPhy2.Set("TxPowerEnd", DoubleValue(16.0));
    wifiPhy2.Set("TxPowerLevels", UintegerValue(1));
    wifiPhy2.Set("TxGain", DoubleValue(2));
    wifiPhy2.Set("RxGain", DoubleValue(2));
    wifiPhy2.Set("RxNoiseFigure", DoubleValue(7));
    wifiPhy2.Set("CcaMode1Threshold", DoubleValue(-80));
    wifiPhy2.Set("EnergyDetectionThreshold", DoubleValue(-80 + 3));
    wifiPhy2.Set("ShortGuardEnabled", BooleanValue(true));
    wifiPhy2.Set("ShortPlcpPreambleSupported", BooleanValue(true));
    wifiPhy2.SetErrorRateModel("ns3::YansErrorRateModel");
    /*wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                        "DataMode", StringValue (phyRate),
                                        "ControlMode", StringValue ("HtMcs0"));*/
    wifiHelper.SetRemoteStationManager("ns3::IdealWifiManager");



    // make building
    Ptr <Building> b = CreateObject<Building>();
    b->SetBoundaries(Box(1, 100, 1, 20, 1, 15));
    b->SetBuildingType(Building::Residential);
    b->SetExtWallsType(Building::ConcreteWithWindows);
    b->SetNFloors(1);
    b->SetNRoomsX(2);
    b->SetNRoomsY(3);

    // show building property
    uint16_t x = b->GetNRoomsX();
    uint16_t y = b->GetNRoomsY();
    uint16_t z = b->GetNFloors();
    Box box = b->GetBoundaries();
    double box_x = box.xMax;
    double box_y = box.yMax;
    double box_z = box.zMax;
    uint32_t totalRoom = x * y * z;
    std::cout << "\nSize of Building" << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "# of rooms x-axis: " << x << std::endl;
    std::cout << "# of rooms y-axis: " << y << std::endl;
    std::cout << "# of rooms z-axis: " << z << std::endl;
    std::cout << "Room size        : " << box_x / x << "X" << box_y / y << "X" << box_z / z << std::endl;
    std::cout << "# of total rooms : " << totalRoom << std::endl;
    std::cout << "-------------------------------------\n" << std::endl;

    // Make Node staGroup, ap, AllNodes
    NodeContainer staGroup[staNum], ap, allNodes;


    // Mobility Setting -> RandomRoomPositAlloc & ConstantPositionMobility
    MobilityHelper mobility;
    Ptr <PositionAllocator> positionAlloc = CreateObject<RandomRoomPositionAllocator>();
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // Make staGroup & and attach building
    for (int i = 0; i < staNum; i++) {
        staGroup[i].Create(totalRoom);
        std::cout << i << " group node size = " << staGroup[i].GetN() << std::endl;
        allNodes.Add(staGroup[i]);
        mobility.Install(staGroup[i]);
        BuildingsHelper::Install(staGroup[i]);
    }


    // Make ap Node & attach building
    ap.Create(totalRoom);
    allNodes.Add(ap);
    mobility.Install(ap);
    BuildingsHelper::Install(ap);

    std::cout << "\n# of All nodes: " << allNodes.GetN() << "\n" << std::endl;

    NetDeviceContainer apDevices, staDevices, p2pDevices, csmaDevices;
    Ssid ssid;
    ssid = Ssid("networkA");

    /* Configure AP */
    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "HtSupported", BooleanValue(true));
    apDevices = wifiHelper.Install(wifiPhy2, wifiMac, ap);

    /* Configure STA */
    wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    staDevices = wifiHelper.Install(wifiPhy2, wifiMac, staGroup[0]);


    // std::cout << apDevices.Get(1) -> GetChannel() -> GetId()<< std::endl; Channel id
    // std::cout << staDevices.Get(1) -> GetChannel() -> GetNDevices()<< std::endl; # of nodes connected to this channel

    BuildingsHelper::MakeMobilityModelConsistent();

    std::map <Room, uint32_t> roomCounter;
    Room rooms[totalRoom];

    /*// calculate node position(Room)
    for (NodeContainer::Iterator it = allNodes.Begin(); it != allNodes.End(); ++it) {
        Ptr <MobilityModel> mm = (*it)->GetObject<MobilityModel>();
        Ptr <MobilityBuildingInfo> bmm = mm->GetObject<MobilityBuildingInfo>();

        Room r(bmm->GetRoomNumberX(), bmm->GetRoomNumberY(), bmm->GetFloorNumber());
        ++(roomCounter[r]);
    }*/

    for (uint32_t i = 0; i < allNodes.GetN(); i++) {
        Ptr <Node> node = allNodes.Get(i);
        Ptr <MobilityModel> mm = (node)->GetObject<MobilityModel>();
        Ptr <MobilityBuildingInfo> bmm = mm->GetObject<MobilityBuildingInfo>();

        if(i <  totalRoom) {
            rooms[i] = Room(bmm->GetRoomNumberX(), bmm->GetRoomNumberY(), bmm->GetFloorNumber());
            rooms[i].Add(node);
        }
        else {
            for(uint32_t j = 0; j < totalRoom; j++) {
                if(rooms[j].get_number() == get_room_to_node(node))
                    rooms[j].Add(node);
            }
        }
    }

    for(uint32_t i = 0; i < totalRoom; i++) {
        std::cout<<"Room index # "<<rooms[i].idx<<" Room position: "<<rooms[i].get_number()<<std::endl;
        rooms[i].print_node_();
    }

    // show # of node per room
   /* int i = 1;
    for (std::map<Room, uint32_t>::iterator it = roomCounter.begin(); it != roomCounter.end(); ++it) {
        if (i < 10)
            std::cout << "# of nodes in   " << i << " Room: " << it->second << std::endl;
        else if ( i < 100)
            std::cout << "# of nodes in  " << i << " Room: " << it->second << std::endl;
        else
            std::cout << "# of nodes in " << i << " Room: " << it->second << std::endl;
        i++;
    } */


    Ptr <BasicEnergySource> energySource = CreateObject<BasicEnergySource>();
    Ptr <WifiRadioEnergyModel> energyModel = CreateObject<WifiRadioEnergyModel>();

    energySource->SetInitialEnergy(300);
    energyModel->SetEnergySource(energySource);
    energySource->AppendDeviceEnergyModel(energyModel);

    ap.Get(0)->AggregateObject(energySource);

    InternetStackHelper stack;
    stack.Install(allNodes);

    Ipv4AddressHelper address;

    address.SetBase("192.0.1.0", "255.255.255.0");
    Ipv4InterfaceContainer staInterface;
    staInterface = address.Assign(staDevices);

    Ipv4InterfaceContainer apInterface;
    apInterface = address.Assign(apDevices);

    //std::cout<<apDevices.Get(0) -> Send(sink, staDevices.Get(0) -> GetAddress(), 1)<<std::endl;

    /* Populate routing table */
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    /* Install TCP Receiver on the access point */
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 9));
    ApplicationContainer sinkApp = sinkHelper.Install(ap);
    sink = StaticCast<PacketSink>(sinkApp.Get(0));

    /* Install TCP/UDP Transmitter on the station */
    OnOffHelper server("ns3::UdpSocketFactory", (InetSocketAddress(apInterface.GetAddress(0), 9)));
    server.SetAttribute("PacketSize", UintegerValue(payloadSize));
    server.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    server.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    server.SetAttribute("DataRate", DataRateValue(DataRate(dataRate)));
    ApplicationContainer serverApp = server.Install(staGroup[0]);


  /*  Ptr<Packet> empty = Create<Packet>();
    int j =0;
    for (uint8_t i =0; i < staDevices.GetN(); i++) {
        //Ipv4Address ipaddr = getIpv4FromNoe(staDevices.Get(i) -> GetNode());
        //std::cout<<apDevices.Get(0) -> GetMulticast(ipaddr)<<std::endl;
        if(apDevices.Get(0) -> Send(empty, staDevices.Get(i)->GetAddress(),0x0800))
            j++;
        std::cout<<j<<std::endl;
    } */


    sinkApp.Start(Seconds(0.0));
    serverApp.Start(Seconds(1.0));
    Simulator::Schedule(Seconds(1.1), & CalculateThroughput);

    std::string base = "he_cj";
    std::string ext = "xml";
    std::string filename = base + "." + ext;

    AnimationInterface anim(filename);
    // Draw Sta Node
    for (int i = 0; i < staNum; i++) {
        for (uint32_t j = 0; j < staGroup[i].GetN(); ++j) {
            anim.UpdateNodeDescription(staGroup[i].Get(j), "STA"); // Optional
            if (i == 0)
                anim.UpdateNodeColor(staGroup[i].Get(j), 255, 0, 0); // Optional
            else if (i == 1)
                anim.UpdateNodeColor(staGroup[i].Get(j), 0, 0, 255); // Optional
            else
                anim.UpdateNodeColor(staGroup[i].Get(j), 127, 128, 0); // Optional
        }
    }

    // Draw Ap Node
    for (uint32_t i = 0; i < ap.GetN(); ++i) {
        anim.UpdateNodeDescription(ap.Get(i), "STA"); // Optional
        anim.UpdateNodeColor(ap.Get(i), 0, 255, 0); // Optional
    }

    // simulator
    Simulator::Stop(Seconds(simulationTime + 1));
    Simulator::Run();
    Simulator::Destroy();


    double averageThroughput = ((sink->GetTotalRx() * 8) / (1e6 * simulationTime));
    std::cout << "\nAverage throughput: " << averageThroughput << " Mbit/s" << std::endl;

    return 0;
}