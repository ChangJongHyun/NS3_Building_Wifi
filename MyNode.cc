#include "ns3/core-module.h"
#include <ns3/building.h>
#include <ns3/buildings-module.h>
#include <ns3/wifi-module.h>
#include <ns3/ipv4.h>
#include "windows.h"

#define INTERVAL 1

using namespace ns3;

class MyNode {
private:
    Ptr<Node> m_node;
    Ipv4Address m_ipv4;
    static NodeContainer *m_all_node;

    double m_rssi;
    double m_packet;

    WifiNetDevice::ReceiveCallback m_receiveCallback;
    WifiPhy::MonitorSnifferRxCallback m_monitorSnifferRxCallback;

    void InstallMonitorSnifferRxCallback() {
        std::ostringstream s;
        s << "/NodeList/" << m_node->GetId() << "/DeviceList/*/Phy/MonitorSnifferRx";
        Config::ConnectWithoutContext(s.str(), MakeCallback(&MyNode::MonitorSniffRx, this));
    }

    /*Callback Method*/
    bool
    ReceivePacket(Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol, const Address &address) {
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

        this->InstallMonitorSnifferRxCallback();

        return true;
    }

    void
    MonitorSniffRx(Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector,
                   MpduInfo aMpdu, SignalNoiseDbm signalNoise) {
        m_packet+=packet->GetSize();
        if(m_rssi != signalNoise.signal)
            m_rssi = signalNoise.signal;
    }

    Ptr<Node>
    static FindSrcNode(const Address &address) {
        for (uint16_t i = 0; i <m_all_node->GetN(); i++) {
            if (m_all_node->Get(i)->GetDevice(0)->GetAddress() == address)
                return m_all_node->Get(i);
        }
    }

public:
    MyNode(Ptr<Node> node) {
        m_node = node;
        m_rssi = 0;
        m_packet = 0;
        m_ipv4 = m_node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    }

     void
     static SetNodeContainer(NodeContainer *all) {
         m_all_node = all;
     }

    void InstallReceiveCallback() {
        m_receiveCallback = MakeCallback(&MyNode::ReceivePacket, this);

        // 모든 노드의 0번 device는 wifinetdevice!
            try {
                m_node->GetDevice(0)->SetReceiveCallback(m_receiveCallback);
            } catch (_exception e) {
                std::cout<<e.name<<std::endl;
            }
        }

        void
        Run(int simulation_time) {
            int time = 0;
            while(time < simulation_time) {

                // TODO MYCODE UPDATE RSSI
                std::cout<<m_node->GetId()<<"'s RSSI: "<<m_rssi<<std::endl;
                time += 1;
                Sleep(INTERVAL);
            }

            std::cout<<"Total Packet: "<<m_packet<<std::endl;
        }
};