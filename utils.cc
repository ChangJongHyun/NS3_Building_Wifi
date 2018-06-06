#include "My_util/MyCreateUtils.h"

MyBuilding::MyBuilding() {
    Ptr<Building> b = CreateObject<Building>();
    m_building = b;
}

Ptr<Building>
MyBuilding::CreateBuilding(Box box, Building::BuildingType_t type, Building::ExtWallsType_t wallsType, uint16_t RoomX,
                         uint16_t RoomY, uint16_t RoomFloor) {

    m_building->SetBoundaries(box);
    m_building->SetBuildingType(type);
    m_building->SetExtWallsType(wallsType);
    m_building->SetNRoomsX(RoomX);
    m_building->SetNRoomsY(RoomY);
    m_building->SetNFloors(RoomFloor);

    return m_building;
}
