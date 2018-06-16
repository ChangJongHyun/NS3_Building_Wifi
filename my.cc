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
#include <utility>
#include <vector>
#include <ns3/building.h>
#include <ns3/buildings-module.h>
#include <ns3/basic-energy-source.h>
#include <ns3/netanim-module.h>
#include "time.h"
#include "ns3/mesh-information-element-vector.h"
#include <string>
#include <cstdint>
#include <algorithm>
#include <stdlib.h>


NS_LOG_COMPONENT_DEFINE ("Main");

using namespace ns3;

#define Downlink true
#define Uplink false
#define PI 3.14159265
#define PI_e5 314158
#define INTERVAL 1
#define MESH_WIFI_BEACON_H

Ptr<PacketSink> sink;
Ptr<PacketSink> sink2;
Ptr<PacketSink> sink3;

uint64_t lastTotalRx = 0;
/*
#define MAX_TOK 1000
string* StringSplit(string strTarget, string strTok)
{
  uint32_t nCutPos;
  uint32_t nIndex = 0;
  string* strResult = new string[D_MAX_ARRAY_SIZE];

  while ((nCutPos = strTarget.find_first_of(strTok)) != strTarget.npos)
  {
    if (nCutPos > 0)
    {
      strResult[nIndex++] = strTarget.substr(0, nCutPos);
    }
    strTarget = strTarget.substr(nCutPos+1);
  }

  if(strTarget.length() > 0)
  {
    strResult[nIndex++] = strTarget.substr(0, nCutPos);
  }

  return strResult;
}
*/

/*        uint64_t convert_mac(std::string mac){
          mac.erase(std::remove(mac.begin(), mac.end(), ':'), mac.end());
          return strtoul(mac.c_str(), NULL, 16);
          std::cout<<"mac :"<<strtoul(mac.c_str(), NULL, 16)<<std::endl;
        }
*/


template <typename Type>
Type max(Type a, Type b){
    return a>b ? a: b;
}
template <typename Type>
Type min(Type a, Type b){
    return a<b ? a: b;
}

class MeshWifiBeacon {
public:
    MeshWifiBeacon(Ssid ssid, SupportedRates rates, uint64_t us);
    MgtBeaconHeader BeaconHeader() const {return m_header;}
    void AddInformationElement(Ptr<WifiInformationElement> ie);

    WifiMacHeader CreateHeader(Mac48Address address, Mac48Address mpAddress);
    Time GetBeaconInterval() const;
    Ptr<Packet> CreatePakcet();

private:
    MgtBeaconHeader m_header;
    MeshInformationElementVector m_element;
};



class MyNode {
private:
    Ptr<Node> m_node;
    Ptr<PacketSink> m_sink;
    Ipv4Address m_ipv4;
    NodeContainer m_all_node;

    double m_Maxrssi1 = 0, m_Maxrssi2= 0, m_Maxrssi3= 0, m_Maxrssi4= 0;
    double m_Minrssi1= 0, m_Minrssi2= 0, m_Minrssi3= 0, m_Minrssi4= 0;
    double maxx1 = -150, maxx2= -150, maxx3= -150, maxx4= -150, minn1 = 0, minn2 = 0, minn3 = 0, minn4= 0;
    double m_packet;

    WifiNetDevice::ReceiveCallback m_receiveCallback;

    void InstallMonitorSnifferRxCallback() {
        std::ostringstream s;
        s << "/NodeList/" << m_node->GetId() << "/DeviceList/*/Phy/MonitorSnifferRx";
        Config::ConnectWithoutContext(s.str(), MakeCallback(&MyNode::MonitorSniffRx, this));
    }

    /*Callback Method*/
    bool
    ReceivePacket(Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol, const Address &address) {
/*
        Ptr<Node> source = MyNode::FindSrcNode(address);
        Ptr<Ipv4> srcipv4 = source->GetObject<Ipv4>();
        Ptr<Ipv4> taripv4 = device->GetNode()->GetObject<Ipv4>();
        Ptr<MobilityModel> srcmm = source->GetObject<MobilityModel>();
        Ptr<MobilityModel> tarmm = device->GetNode()->GetObject<MobilityModel>();
        Vector sender = srcmm->GetPosition();
        Vector receiver = tarmm->GetPosition();
        std::cout << "Send " << srcipv4->GetAddress(1, 0).GetLocal()
                  << " (" << sender.x << ", " << sender.y << ", " << sender.z << ") to "
                  << taripv4->GetAddress(1, 0).GetLocal() << " (" << receiver.x << ", " << receiver.y << ", " << receiver.z
                  << ")" << std::endl;
        std::cout << "Distance: " << CalculateDistance(sender, receiver) << " " <<packet->GetSize()<< "(Byte)" << std::endl;
*/
        std::cout<<packet->GetSize()<<std::endl;

        InstallMonitorSnifferRxCallback();

        return true;
    }

    double i = 0;

    void
    MonitorSniffRx(Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector,
                   MpduInfo aMpdu, SignalNoiseDbm signalNoise) {

        i++;

        WifiMacHeader header;
        packet->PeekHeader(header);
        //packet->Print(std::cout);
        //std::cout<<m_ipv4<<"<--"<<header.GetAddr2()<<" RSSI: "<<signalNoise.signal<<std::endl;
        m_packet+=packet->GetSize();

        if (header.GetAddr2() == "00:00:00:00:00:01" || header.GetAddr2() == "00:00:00:00:00:02" || header.GetAddr2() == "00:00:00:00:00:03" || header.GetAddr2() == "00:00:00:00:00:04"){
            if (m_ipv4 == "10.0.0.1"){
                //std::cout<<"AP addr: "<<m_ipv4<<std::endl;
                m_Maxrssi1 = max(maxx1, signalNoise.signal);
                maxx1 = m_Maxrssi1;
                //std::cout<<"AP1 Max RSSI: "<<m_Maxrssi1<<"\n"<<std::endl;
            } else if (m_ipv4 == "10.0.0.2"){
                m_Maxrssi2 = max(maxx2, signalNoise.signal);
                maxx2 = m_Maxrssi2;
                //std::cout<<"AP2 Max RSSI: "<<m_Maxrssi2<<"\n"<<std::endl;
            } else if (m_ipv4 == "10.0.0.3"){
                m_Maxrssi3 = max(maxx3, signalNoise.signal);
                maxx3 = m_Maxrssi3;
                //std::cout<<"AP3 Max RSSI: "<<m_Maxrssi3<<"\n"<<std::endl;
            } else {
                m_Maxrssi4 = max(maxx4, signalNoise.signal);
                maxx4 = m_Maxrssi4;
                //std::cout<<"AP4 Max RSSI: "<<m_Maxrssi4<<"\n"<<std::endl;
            }
        }
        else {
            if (m_ipv4 == "10.0.0.1"){
                m_Minrssi1 = min(minn1, signalNoise.signal);
                minn1 = m_Minrssi1;
                //std::cout<<"AP1 Min RSSI: "<<m_Minrssi1<<"\n"<<std::endl;
            } else if (m_ipv4 == "10.0.0.2"){
                m_Minrssi2 = min(minn2, signalNoise.signal);
                minn2 = m_Minrssi2;
                //std::cout<<"AP2 Min RSSI: "<<m_Minrssi2<<"\n"<<std::endl;
            } else if (m_ipv4 == "10.0.0.3"){
                m_Minrssi3 = min(minn3, signalNoise.signal);
                minn3 = m_Minrssi3;
                //std::cout<<"AP3 Min RSSI: "<<m_Minrssi2<<"\n"<<std::endl;
            } else {
                m_Minrssi4 = min(minn4, signalNoise.signal);
                minn4 = m_Minrssi4;
                //std::cout<<"AP4 Min RSSI: "<<m_Minrssi3<<"\n"<<std::endl;
            }
        }

        //std::ostringstream s;
        //s << "/NodeList/" << m_node->GetId() << "/DeviceList/*/Phy/EnergyDetectionThreshold";
        //Config::Set(s.str(), DoubleValue(-5.0));

        double CST1, CST2, CST3, CST4;

        if ( i == 70){
            if (m_Maxrssi1 != 0 && m_Minrssi1 != 0){
                CST1 = min (max (m_Maxrssi1, m_Minrssi1) -25, m_Minrssi1);

                if (CST1 < -80){
                    CST1 = -80;
                }
                else if (CST1 > -40){
                    CST1 = -40;
                }

                Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/CcaMode1Threshold", DoubleValue (CST1));
//              Config::Set("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/EnergyDetectionThreshold", DoubleValue(CST1 + 3));
                //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/CcaMode1Threshold", DoubleValue (-3));
                //Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/EnergyDetectionThreshold", DoubleValue(-3));
                std::cout<<"AP1 CST: "<<CST1<<"\n"<<std::endl;
            }

            else if (m_Maxrssi2 != 0 && m_Minrssi2 != 0){
                CST2 = min (max (m_Maxrssi2, m_Minrssi2) -25, m_Minrssi2);

                if (CST2 < -80){
                    CST2 = -80;
                }
                else if (CST2 > -40){
                    CST2 = -40;
                }

                Config::Set("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Phy/CcaMode1Threshold", DoubleValue (CST2));
//              Config::Set("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Phy/EnergyDetectionThreshold", DoubleValue(CST2 + 3));
                std::cout<<"AP2 CST: "<<CST2<<"\n"<<std::endl;
            }

            else if (m_Maxrssi3 != 0 && m_Minrssi3 != 0){
                CST3 = min (max (m_Maxrssi3, m_Minrssi3) -25, m_Minrssi3);

                if (CST3 < -80){
                    CST3 = -80;
                }
                else if (CST3 > -40){
                    CST3 = -40;
                }

                Config::Set("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Phy/CcaMode1Threshold", DoubleValue (CST3));
//              Config::Set("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Phy/EnergyDetectionThreshold", DoubleValue(CST3 + 3));
                std::cout<<"AP3 CST: "<<CST3<<"\n"<<std::endl;

            }
            else if (m_Maxrssi4 != 0 && m_Minrssi4 != 0){
                CST4 = min (max (m_Maxrssi4, m_Minrssi4) -25, m_Minrssi4);

                if (CST4 < -80){
                    CST4 = -80;
                }
                else if (CST4 > -40){
                    CST4 = -40;
                }

                Config::Set("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/Phy/CcaMode1Threshold", DoubleValue (CST4));
//              Config::Set("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/Phy/EnergyDetectionThreshold", DoubleValue(CST4 + 3));
                std::cout<<"AP4 CST: "<<CST4<<"\n"<<std::endl;
            }

            maxx1 = -150, maxx2= -150, maxx3= -150, maxx4= -150, minn1 = 0, minn2= 0, minn3= 0, minn4= 0;
            m_Maxrssi1 = 0, m_Maxrssi2 = 0, m_Maxrssi3 = 0, m_Maxrssi4 = 0;
            m_Minrssi1 = 0, m_Minrssi2 = 0, m_Minrssi3 = 0, m_Minrssi4 = 0;
            std::cout<<"***********Reset RSSI***********"<<std::endl;

            i = 0;
        }







    }


    Ptr<Node>
    FindSrcNode(const Address &address) {
        for (uint16_t i = 0; i <m_all_node.GetN(); i++) {
            if (m_all_node.Get(i)->GetDevice(0)->GetAddress() == address)
                return m_all_node.Get(i);
        }
        return NULL;
    }

public:
    MyNode(){};
    MyNode(Ptr<Node> node, NodeContainer nodes) {
        m_node = node;
        m_all_node = std::move(nodes);
        //m_Maxrssi = -150;
        //m_Minrssi = 0;
        m_packet = 0;
        m_ipv4 = m_node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    }

    void
    Initialize(Ptr<Node> node, NodeContainer nodes) {
        m_node = node;
        m_all_node = std::move(nodes);
        //m_Maxrssi = -150;
        //m_Minrssi = 0;
        m_packet = 0;
        m_ipv4 = m_node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    }

    void
    SetNodeContainer(NodeContainer all) {
        m_all_node = all;
    }

//    double
//    GetMaxRSSI() {
//        return this->m_Maxrssi;
//    }

//    double
//    GetMinRSSI() {
//        return this->m_Minrssi;
//    }

    double
    GetTotalPacket() {
        return this->m_packet;
    }

    double
    GetThroughtput(double time) {
        return (this->m_packet * 8) / (1000000 * time);
    }

    void
    InstallMonitorSniffet() {
        InstallMonitorSnifferRxCallback();
    }

    void InstallReceiveCallback() {
        m_receiveCallback = MakeCallback(&MyNode::ReceivePacket, this);

        // 모든 노드의 0번 device는 wifinetdevice!
        m_node->GetDevice(0)->SetReceiveCallback(m_receiveCallback);

    }

    void
    Run() {
/*        std::cout<<"My Ipv4: "<<m_ipv4<<std::endl;
        std::cout<<m_node->GetId()<<"'s MaxRSSI: "<<m_Maxrssi<<std::endl;
        std::cout<<m_node->GetId()<<"'s MinRSSI: "<<m_Minrssi<<std::endl;
        std::cout<<"Total Packet: "<<m_packet<<std::endl;*/

        Simulator::Schedule(MilliSeconds(250), &MyNode::Run, this);
    }

    //void CSTchange(YansWifiPhyHelper m_wifiPhy, NetDeviceContainer m_ap_device, WifiMacHelper m_wifiMac, NodeContainer m_ap){
    /*void CSTchange(){

        std::ostringstream s;
       // s << "/NodeList/" << m_node->GetId() << "/DeviceList//Phy/EnergyDetectionThreshold";
          Config::Set(s.str(), DoubleValue(-5.0));
        std::cout<<"Done!"<<std::endl;
          Simulator::Schedule(MilliSeconds(250), &MyNode::CSTchange, this);
    }*/

    void Reset(){
        maxx1 = -150, maxx2= -150, maxx3= -150, maxx4= -150, minn1 = 0, minn2= 0, minn3= 0, minn4= 0;
        std::cout<<"***********Reset RSSI***********"<<std::endl;
        Simulator::Schedule (MilliSeconds(250), &MyNode::Reset, this);
    }

};





/**
 * 실험 객체
 */
class Experiment {


    /* 방 객체 (struct나 class나 또이또이) */
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
            return x * 100 + y * 10 + z;
        }

        void Add(Ptr<Node> node) {
            nodes.Add(node);
        }

        void print_node_() {
            std::cout << "ID of All node" << ::std::endl;
            for (uint32_t i = 0; i < nodes.GetN(); i++)
                std::cout << nodes.Get(i)->GetId() << std::endl;
        }
    };

public:
    Experiment(bool downlinkUplink);    // 생성자 Uplink인지 downlink인지 설정
    void SetRtsCts(bool enableCtsRts);  // RTS,CTS를 사용할 것인지 설정 InitialExperiment()에서 선언
    void CreateNode(size_t in_ap, size_t in_sta);    // 노드 생성 ap, sta 갯수 설정
    void InitialExperiment();   // 생성한 Experiment 객체는 초기화 해줌(채널, IP 등등.. 설정)
    void InstallApplication(size_t in_packetSize, std::string in_dataRate);  // 노드에 Application insert(신호 보내는..?)
    //void CSTchange(YansWifiPhyHelper m_wifiPhy, NetDeviceContainer m_ap_device, WifiMacHelper m_wifiMac, NodeContainer m_ap);
    void CSTchange();

    void InstallSignalMonitor();

    void ReceiveBeacon(Ptr<Socket> p);
    void Run(size_t in_simTime);    // in_simTime 만큼 시뮬레이션 돌림
    void Reset();
    /*------------------------------------------------------------------------*/
    void PhyRxErrorTrace(std::string context, Ptr<const Packet> packet, double snr);

    void PhyRxOkTrace(std::string context, Ptr<const Packet> packet,
                      double snr, enum WifiPreamble preamble);

    void PhyTxTrace(std::string context, Ptr<const Packet> packet,
                    WifiPreamble preamble, uint8_t txPower);
    /*-------------------------------------------------- 콜백함수 인데 잘 작동안해서... 안보셔도 됩니다!*/

    void ShowNodeInformation(); // 노드의 정보를 출력

    double
    GetAvgThroughtPut(double time) {
        double tmp = 0;
        int size = m_ap.GetN();
        for(int i = 0; i < size; i++)
            tmp+= m_callbackNode->GetThroughtput(time+1);

        return tmp / size;
    }

    /*빌딩 생성
     * box -> 빌딩의 크기
     * type -> 빌딩의 타입(Residential..)
     * wallsType -> 빌딩 벽의 타입
     * RoomX, RoomY, RoomFloor -> x,y,z의 방의 갯수
     * */
    void CreateBuilding(Box box, Building::BuildingType_t type,
                        Building::ExtWallsType_t wallsType,
                        uint16_t RoomX, uint16_t RoomY, uint16_t RoomFloor);

    void AddNodeToRoom();

    NodeContainer m_nodes;  // All node
    ApplicationContainer m_client_app;
    ApplicationContainer m_server_app;


    NodeContainer m_ap;     // ap 노드 컨테이너
    YansWifiChannelHelper m_wifiChannel;    // wifi channel 만들어주기 위함
    WifiHelper m_wifi;  // wifi의 standard, model 설정
    YansWifiPhyHelper m_wifiPhy;    // physical layer 설정하고 할당
    WifiMacHelper m_wifiMac;    // mac을 설정하고 할당

    NetDeviceContainer m_ap_device;


private:
    void SetWifiChannel();  // wifi channel 설정
    void InstallDevices();  // device 설치 (wifiphy, wifi standard)
    void InstallIp();   // 노드에 IP 할당
    void InstallReceiveCallBack();
    void
    MakeCallbackNode() {
        int apsize = m_ap.GetN();
        m_callbackNode = new MyNode[apsize];

        for(int i = 0; i < apsize; i++) {
            m_callbackNode[i].Initialize(m_ap.Get(i), m_nodes);
            m_callbackNode[i].InstallMonitorSniffet();
            //Simulator::Schedule(Seconds(0), &MyNode::Run, &m_callbackNode[i]);
            //Simulator::Schedule(Seconds (1.1), &MyNode::Reset, &m_callbackNode[i]);
        }
    }


    std::map<uint16_t, double> m_MaxRSSI, m_MinRSSI;

    int m_totalRoom;
    bool m_enableCtsRts;
    bool m_downlinkUplink; // 다운링크인지 업링크인지. Downlink, Uplink를 인자로 넣어주면 된다.
    size_t m_apNumber;  // ap의 갯수
    size_t m_staNumber; // sta의 갯수

    /*--------------------------*/
    size_t m_rxOkCount;
    size_t m_rxErrorCount;
    size_t m_txOkCount;
    /*------------------------- 몰라도 되는 변수!*/

    MyNode *m_callbackNode;

    NodeContainer m_sta;    // sta 노드 컨테이너
    MobilityHelper m_mobility;  // mobilityhepler 생성! 노드들에게 mobility 모델(움직이는지 안움직이는지)할당, 위치할당(랜덤 포지션)
    Ptr<Building> m_building;   // 빌딩의 객체
    Ptr<RandomRoomPositionAllocator> m_apPosAlloc;  // 랜덤으로 위치 할당해줌!
    Ptr<RandomRoomPositionAllocator> m_staPosAlloc;

    NetDeviceContainer m_sta_device;
    InternetStackHelper m_internet; // internet stack을 install
    Ipv4AddressHelper m_ipv4;   // ipv4를 만들고, 노드에 할당해줌 (정확하게는 디바이스에)
    Ipv4InterfaceContainer m_ap_interface;  // ap의 주소를 보관
    Ipv4InterfaceContainer m_sta_interface; // sta를 보관
    std::string phyRate = "HtMcs7";
    Room rooms[];


};


/**
 * @param in_downlinkUplink
 *      Downlink --> true
 *      Uplink --> false
 *      Downlink, Uplink가 선언되 있어서 그냥 Uplink, Downlink로 선언하면됨!
 */
Experiment::Experiment(bool in_downlinkUplink) :
        m_downlinkUplink(in_downlinkUplink) {
    m_rxOkCount = 0;
    m_rxErrorCount = 0;
    m_txOkCount = 0;
}

/**
 * Experiment 객체를 초기화
 */
void
Experiment::InitialExperiment() {
    SetWifiChannel();
    InstallDevices();
    InstallIp();
    SetRtsCts(false);
}

/**
 * Rts,Cts 쓸껀지 설정
 */
void
Experiment::SetRtsCts(bool in_enableCtsRts) {
    m_enableCtsRts = in_enableCtsRts;

    // m_enableCtsRts가 true -> 10, false -> 999999 이 밸류를 설정
    UintegerValue ctsThr = (m_enableCtsRts ? UintegerValue(10) : UintegerValue(999999));
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);
}

/**
 * @param box  빌딩의 크기
 * @param type 빌딩의 거주 타입
 * @param wallsType 빌딩의 벽 타입
 * @param RoomX  x축 방의 갯수
 * @param RoomY  y축 방의 갯수
 * @param RoomFloor  건물 층의 갯수
 */
void
Experiment::CreateBuilding(Box box, Building::BuildingType_t type,
                           Building::ExtWallsType_t wallsType,
                           uint16_t RoomX, uint16_t RoomY, uint16_t RoomFloor) {

    // 설정한 파라미터로 빌딩을 만들어 준다.
    Ptr<Building> b = CreateObject<Building>();
    b->SetBoundaries(box);
    b->SetBuildingType(type);
    b->SetExtWallsType(wallsType);
    b->SetNRoomsX(RoomX);
    b->SetNRoomsY(RoomY);
    b->SetNFloors(RoomFloor);

    /* 객체 변수에 만들어준 빌딩값을 할당 */
    m_totalRoom = b->GetNRoomsX() * b->GetNRoomsY() * b->GetNFloors();
    m_building = b;

    /* 방의 정보 출력 */
    std::cout << "\nBuilding" << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    std::cout << "# Building size: " << box.xMax << "x" << box.yMax << "x" << box.zMax << std::endl;
    std::cout << "# Room size: " << box.xMax / b->GetNRoomsX() << "x" << box.yMax / b->GetNRoomsY() << "x"
              << box.zMax / b->GetNFloors() << std::endl;
    std::cout << "# # of rooms x-axis: " << b->GetNRoomsX() << std::endl;
    std::cout << "# # of rooms y-axis: " << b->GetNRoomsY() << std::endl;
    std::cout << "# # of rooms z-axis: " << b->GetNFloors() << std::endl;
    std::cout << "# # of total rooms : " << m_totalRoom << std::endl;
    std::cout << "-------------------------------------\n" << std::endl;
}

/**
 * 1. 노드를 만들어 준다.
 * 2. mobilityhelper를 이용해서 mobilitymodel과 위치를 할당해준다.
 * 3. mobilityhelper를 노드에 할당한다.
 * 4. BuildingHelper를 sta와 ap에 할당한다.
 * 5. ap, sta를 m_nodes (모든 노드)에 넣는다.
 * 6. m_nodes를 이용해서 ap, sta를 빌딩에 넣어준다.
 *
 * @param in_ap
 * @param in_sta
 */
void
Experiment::CreateNode(size_t in_ap, size_t in_sta) {
    m_apNumber = in_ap;
    m_staNumber = in_sta;

    // Create Node
    m_ap.Create(m_apNumber);
    m_sta.Create(m_staNumber);
    // Mobile & Position Alloc
    m_apPosAlloc = CreateObject<RandomRoomPositionAllocator>();
    m_staPosAlloc = CreateObject<RandomRoomPositionAllocator>();
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
        Ptr<MobilityModel> mm = (*it)->GetObject<MobilityModel>();
        Ptr<MobilityBuildingInfo> bmm = mm->GetObject<MobilityBuildingInfo>();
        Vector p = mm->GetPosition();
        bmm->SetIndoor(m_building, m_building->GetFloor(p), m_building->GetRoomX(p), m_building->GetRoomY(p));
    }
}

/**
 * wifi 채널을 만든다.
 */
void
Experiment::SetWifiChannel() {
    m_wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    m_wifiChannel.AddPropagationLoss("ns3::HybridBuildingsPropagationLossModel", "Frequency", DoubleValue(2.4e9),
                                     "CitySize", EnumValue(1), /*"RooftopLevel", DoubleValue (15),*/ "InternalWallLoss",
                                     DoubleValue(12));
}


/*void
//Experiment::CSTchange(YansWifiPhyHelper m_wifiPhy, NetDeviceContainer m_ap_device, WifiMacHelper m_wifiMac, NodeContainer m_ap) {
Experiment::CSTchange() {

//TODO
      for(int i = 0; i < 4; ++i){
        double const s = -5;
        m_wifiPhy.Set ("CcaMode1Threshold", DoubleValue (s));
        m_wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (s + 3));
        m_ap_device = m_wifi.Install (m_wifiPhy, m_wifiMac, m_ap.Get(i));
        std::cout<<"\nAP"<<i<<" CST: "<<s<<std::endl;
      }
      Simulator::Schedule(MilliSeconds(250), &CSTchange, MyNode);
}*/

/**
 * 노드에 wifi device를 설정해준다.
 * m_wifi - wifiHelper 와이파이 설정
 * m_wifiPhy - YansWifiHelper 물리계층 설정
 * ssid - 네트워크 만들어 줌
 * m_wifiMac - mac을 ap, sta에 할당
 */


void
Experiment::InstallDevices() {
    m_wifi.SetStandard(WIFI_PHY_STANDARD_80211n_2_4GHZ);
    //Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue(40.046));
    m_wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    /* m_wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager","DataMode", StringValue (phyRate),
     "ControlMode", StringValue("HtMcs0"));*/
    m_wifiPhy = YansWifiPhyHelper::Default();
    m_wifiPhy.SetChannel(m_wifiChannel.Create());
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



/**
 * m_internet - 인터넷 스택을 node에 할당한다.
 * m_ipv4를 이용해서 ip를 만들고, 디바이스에 할당
 *
 */
void
Experiment::InstallIp() {
    m_internet.Install(m_ap);
    m_internet.Install(m_sta);

    m_ipv4.SetBase("10.0.0.0", "255.0.0.0");

    m_ap_interface = m_ipv4.Assign(m_ap_device);
    m_sta_interface = m_ipv4.Assign(m_sta_device);
}

/*-----------------------------------------------------------------------------------------------*/
void
Experiment::PhyRxErrorTrace(std::string context, Ptr<const Packet> packet, double snr) {
    Ptr<Packet> m_currentPacket;
    WifiMacHeader hdr;
    m_currentPacket = packet->Copy();
    m_currentPacket->RemoveHeader(hdr);
    if (hdr.IsData()) {
        m_rxErrorCount++;
    }
}

void
Experiment::PhyRxOkTrace(std::string context, Ptr<const Packet> packet,
                         double snr, enum WifiPreamble preamble) {
    Ptr<Packet> m_currentPacket;
    WifiMacHeader hdr;

    m_currentPacket = packet->Copy();
    m_currentPacket->RemoveHeader(hdr);
    if (hdr.IsData()) {
        m_rxOkCount++;
    }
}

void
Experiment::PhyTxTrace(std::string context, Ptr<const Packet> packet, WifiPreamble preamble, uint8_t txPower) {
    Ptr<Packet> m_currentPacket;
    WifiMacHeader hdr;
    m_currentPacket = packet->Copy();
    m_currentPacket->RemoveHeader(hdr);
    if (hdr.IsData()) {
        m_txOkCount++;
    }
}

void
Experiment::ReceiveBeacon(Ptr<Socket> p) {
    std::cout<<"Hello!"<<std::endl;
}


/*-----------------------------------------------------------------------------------------------*/
/**
 * 노드에 어플리케이션 추가
 * Uplink에 맞게 코드를 수정해야할 필요가 있어보임..
 *
 * @param in_packetSize 패킷사이즈
 * @param in_dataRate 데이터 전송률
 */
void
Experiment::InstallApplication(size_t in_packetSize, std::string in_dataRate) {

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    uint16_t port = 9;

/*    for (uint8_t index = 0; index < m_sta.GetN(); ++index) {
        for (uint8_t i = 0; i < m_ap.GetN(); i++) {
            // Install UDP Receiver on ther access point
            auto ipv4 = m_sta.Get(0)->GetObject<Ipv4>();
            const auto addr = ipv4->GetAddress(1, 0).GetLocal();

            InetSocketAddress sinkSocket(addr, port++);
            OnOffHelper client("ns3::UdpSocketFactory", sinkSocket);
            client.SetAttribute("PacketSize", UintegerValue(in_packetSize)); //bytes
            client.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            client.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
            client.SetAttribute("DataRate", DataRateValue(DataRate(in_dataRate)));
            m_client_app.Add(client.Install(m_ap.Get(index)));

            PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", sinkSocket);
            m_server_app.Add(sinkHelper.Install(m_sta));
        }
    }
*/


    // TODO send udp packet TODO
    auto ipv4 = m_sta.Get(0)->GetObject<Ipv4>();
    const auto addr = ipv4->GetAddress(1, 0).GetLocal();

    InetSocketAddress sinkSocket(addr, port);
    OnOffHelper client("ns3::UdpSocketFactory", sinkSocket);
    client.SetAttribute("PacketSize", UintegerValue(in_packetSize)); //bytes
    client.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    client.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    client.SetAttribute("DataRate", DataRateValue(DataRate(in_dataRate)));
    m_client_app.Add(client.Install(m_ap));

    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", sinkSocket);
    m_server_app.Add(sinkHelper.Install(m_sta));

    sink = StaticCast<PacketSink>(m_server_app.Get(0));



    //server_app --> 패킷 받는애 sink_app --> 패킷 보내는 애 (onoffhelper)

}

/* 노드의 정보를 출력 */
void
Experiment::ShowNodeInformation() {
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
            for (uint32_t i = 0; i < (*it)->GetNDevices(); i++) {
                std::cout << " STA Device => (" << i << ")" << (*it)->GetDevice(i)->GetInstanceTypeId() << std::endl;
            }
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
    std::cout << " # Rx Gain: " << wifiPhy->GetRxGain() << std::endl;
    std::cout << " # Tx Gain: " << wifiPhy->GetTxGain() << std::endl;
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
    std::cout << " # Rx Gain: " << wifiPhy2->GetRxGain() << std::endl;
    std::cout << " # Tx Gain: " << wifiPhy2->GetTxGain() << std::endl;
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
    std::cout << " # Rx Gain: " << wifiPhy3->GetRxGain() << std::endl;
    std::cout << " # Tx Gain: " << wifiPhy3->GetTxGain() << std::endl;
    std::cout << "-------------------------------------" << std::endl;

}

/**
 * 시뮬레이션을 돌림
 * @param in_simTime 몇초동안 시뮬레이션을 돌릴것인가 + Flow monitor
 * */
void
Experiment::Run(size_t in_simTime) {
    // 8. Install FlowMonitor on all nodes
    m_client_app.Start(Seconds(1.0));
    m_server_app.Start(Seconds(0.0));

    this->MakeCallbackNode();



    AnimationInterface anim ("wifi-tcp-nt6.xml"); // Mandatory
    for (uint32_t i = 0; i < m_sta.GetN (); ++i)
    {
        anim.UpdateNodeDescription (m_sta.Get (i), "STA"); // Optional
        anim.UpdateNodeColor (m_sta.Get (i), 255, 0, 0); // Optional
        anim.SetMaxPktsPerTraceFile(500000);
    }
    for (uint32_t i = 0; i < m_ap.GetN (); ++i)
    {
        anim.UpdateNodeDescription (m_ap.Get (i), "AP"); // Optional
        anim.UpdateNodeColor (m_ap.Get (i), 0, 255, 0); // Optional
        anim.SetMaxPktsPerTraceFile(500000);
    }

    anim.EnablePacketMetadata (); // Optional
    anim.EnableWifiMacCounters (Seconds (0), Seconds (11)); //Optional
    anim.EnableWifiPhyCounters (Seconds (0), Seconds (11)); //Optional


    // 9. Run simulation
    Simulator::Stop(Seconds(in_simTime + 1));
    Simulator::Run();


    // 10. Print per flow statistics
    /*   monitor->CheckForLostPackets();
     *     FlowMonitorHelper flowmon;
       Ptr<FlowMonitor> monitor = flowmon.InstallAll();
       Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
       std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
       double accumulatedThroughput = 0;
       for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
            i != stats.end(); ++i) {
           Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
           std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
           std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
           std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
           std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
           std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
           std::cout << "  Lost Packets: " << i->second.lostPackets << "\n";
           std::cout << "  Pkt Lost Ratio: "
                     << ((double) i->second.txPackets - (double) i->second.rxPackets) / (double) i->second.txPackets
                     << "\n";
           std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / in_simTime / 1024 / 1024 << " Mbps\n";
           accumulatedThroughput += (i->second.rxBytes * 8.0 / in_simTime / 1024 / 1024);
       }
       std::cout << "apNumber=" << m_apNumber << " nodeNumber=" << m_staNumber << "\n" << std::flush;
       std::cout << "throughput=" << accumulatedThroughput << "\n" << std::flush;
       std::cout << "tx=" << m_txOkCount << " RXerror=" << m_rxErrorCount <<
                 " Rxok=" << m_rxOkCount << "\n" << std::flush;
       std::cout << "===========================\n" << std::flush;
   */
    // 11. Cleanup
    Simulator::Destroy();

    double throughtput = this->GetAvgThroughtPut(in_simTime);

    std::cout<<"Avg Throughtput: "<<throughtput<<"(Mbps)"<<std::endl;
}

/* 쓰루풋 계산 이걸사용하려면 Run을 사용하지 않고, main에 주석처리한 부분을 제거해 주어야함! */

void
CalculateThroughput() {
    Time now = Simulator::Now();
    //double car = sink->GetTotalRx ();
    double cor = (sink->GetTotalRx() - lastTotalRx);
    double cur =
            (sink->GetTotalRx() - lastTotalRx) * (double) 8 / 1e5;
    //std::cout << now.GetSeconds () << "s: \t" << "RX Total Packets= " << car << std::endl;
    std::cout << now.GetSeconds() << "s: \t" << "RX Packets= " << cor << ",  Throughput= " << cur << " Mbit/s"
              << std::endl;
    lastTotalRx = sink->GetTotalRx();
    Simulator::Schedule(MilliSeconds(100), &CalculateThroughput);
}

/**
 * 실제 실험 환경d
 * 1. Uplink, Downlink에 맞게 객체를 만든다.
 * 2. 빌딩을 내가 원하는 형태로 만든다.
 * 3. ap와 sta 노드를 만든다.
 * 4. 초기화 해준다. (채널 할당, ip할당, 디바이스 할당 등등)
 * 5. Application을 설치한다. (udp, tcp같은 전송을 위함)
 * 6. 노드 정보를 출력한다.
 * 7. 시뮬레이션을 돌린다.
 */
int main(int argc, char **argv) {
    size_t payload_size = 1472;
    /*size_t data_rate = 72200000;*/
    std::string dataRate = "72.2Mbps";
    size_t simulationTime = 9;
    //   size_t numOfAp[6] = {1, 2, 3, 4, 5, 6};
    //double range[4] = {60, 120, 180, 2

    Experiment exp(Downlink);
    exp.CreateBuilding(Box(1.0, 40, 1.0, 10, 1.0, 3),
                       Building::Residential, Building::ConcreteWithWindows, 4, 1, 1);
    exp.CreateNode(4, 20);
    exp.InitialExperiment();
    exp.InstallApplication(payload_size, dataRate);
    exp.ShowNodeInformation();
    //exp.CSTchange();

    /*exp.m_client_app.Start(Seconds(0.0));
    exp.m_server_app.Start(Seconds(1.0));
    Simulator::Schedule(Seconds(1.1), &CalculateThroughput);
    Simulator::Stop(Seconds(simulationTime + 1));
    Simulator::Run();
    Simulator::Destroy();
*/
    exp.Run(simulationTime);
/*
    uint64_t save = 0;
    double throughput = 0;
    for (unsigned index = 0; index < exp.m_server_app.GetN(); ++index) {
        uint64_t totalPacketsThrough = DynamicCast<PacketSink>(exp.m_server_app.Get(index))->GetTotalRx();
        throughput += ((totalPacketsThrough * 8) / (simulationTime * 1000000.0)); //Mbit/s
        std::cout << "\nAggregated throughput: " << throughput << " Mbit/s" << std::endl;
        save = save + throughput;
    }
    double averageThroughput = save / exp.m_server_app.GetN();
    std::cout << "\nAverage throughput: " << averageThroughput << " Mbit/s" << std::endl;
*/

    return 0;
}
