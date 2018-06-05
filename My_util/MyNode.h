#include "ns3/core-module.h"
#include <ns3/building.h>
#include <ns3/buildings-module.h>

NS_LOG_COMPONENT_DEFINE ("Main");

using namespace ns3;

class MyNode {
private:
    Ptr<Node> m_node;
    double m_rssi;


};