
#include "ns3/core-module.h"
#include <ns3/building.h>
#include <ns3/buildings-module.h>

NS_LOG_COMPONENT_DEFINE ("Main");

using namespace ns3;



class MyBuilding {
private:
    Ptr<Building> m_building;
public:
    MyBuilding();
    Ptr<Building> CreateBuilding(Box box, Building::BuildingType_t type,
                                        Building::ExtWallsType_t wallsType,
                                        uint16_t RoomX, uint16_t RoomY, uint16_t RoomFloor);

    void printBuildingInfo();


};


class Room {
public:

private:
    double m_x;
    double m_y;
    double m_z;
    int index;
};