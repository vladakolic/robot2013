#pragma once

#include <utility>
#include <vector>
#include <tr1/memory>

#include <map_creation/Map.h>

/**
 * The implementation of an editor for a map structure.
 */
class MapModel
{
private:
    float m_fWidth;
    float m_fHeight;
    float m_fCellSize;
    size_t m_nWidth;
    size_t m_nHeight;

    std::tr1::shared_ptr<char> m_pMapArray;
    size_t m_nMapSizeBytes;

    float m_fRobotPositionX;
    int m_nRobotPositionX;
    float m_fRobotPositionY;
    int m_nRobotPositionY;
    float m_fRobotOrientation;
    std::vector< std::pair<float, float> > m_vecObjects;

    void removePoint(const float& fX, const float& fY);
    void addPoint(const float& fX, const float& fY);

public:
    MapModel();

    ~MapModel();

    void create(const float& fWidth, const float& fHeight, const float& fCellSize);

    std::tr1::shared_ptr<char> const getMapArray();

    void addObstacle(const float& fPositionX, const float& fPositionY);

    void placeRobot(const float& fPositionX, const float& fPositionY, const float& fOrientation);

    void placeObject(const float& fPositionX, const float& fPositionY);

    void exportMap(std::tr1::shared_ptr<map_creation::Map>& pMapMsg);

}; // class MapModel
