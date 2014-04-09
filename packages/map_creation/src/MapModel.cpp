#include <ros/ros.h>

#include <map_creation/MapModel.h>

MapModel::MapModel()
{
}

MapModel::~MapModel()
{
}

void MapModel::create(const float& fWidth, const float& fHeight, const float& fCellSize)
{
    m_fWidth = fWidth;
    m_fHeight = fHeight;
    m_nWidth = (size_t) (fWidth / fCellSize);
    m_nHeight = (size_t) (fHeight / fCellSize);
    m_fCellSize = fCellSize;
    ROS_INFO("Created new map: [%gm x %gm] @ %gm aka [%dpx x %dpx]", m_fWidth, m_fHeight, m_fCellSize, m_nWidth, m_nHeight);

    /*
     * Create a data structure for the map.
     */
    m_nMapSizeBytes = m_nWidth * m_nHeight * sizeof(char);
    m_pMapArray = std::tr1::shared_ptr<char>(new char[m_nMapSizeBytes]);

    /*
     * Set every cell to status unvisited.
     */
    char* pCell = m_pMapArray.get();
    for (size_t y = 0; y < m_nHeight; ++y)
        for (size_t x = 0; x < m_nWidth; ++x)
            pCell[y * m_nWidth + x] = -1;
}

std::tr1::shared_ptr<char> const MapModel::getMapArray()
{
    return m_pMapArray;
}

void MapModel::removePoint(const float& fX, const float& fY)
{
    /*
     * Convert point from global space to grid space that has its origin in the image center.
     */
    size_t nX = (size_t) (fX / m_fCellSize + 0.5f * float(m_nWidth));
    size_t nY = (size_t) (fY / m_fCellSize + 0.5f * float(m_nHeight));

    /*
     * Check if point is inside the map.
     */
    if (nX >= m_nWidth || nY >= m_nHeight)
    {
        ROS_WARN("Map: could not remove object at position [%d,%d].", nX, nY);
        return;
    }

    /*
     * Remove the point by applying a wheighted zero to its position.
     */
    char* pCell = m_pMapArray.get();
    char nValue = pCell[nY * m_nWidth + nX] * 0.25 + 0 * 0.25;
    pCell[nY * m_nWidth + nX] = nValue <= 100 ? nValue : 100;
}

void MapModel::addPoint(const float& fX, const float& fY)
{
    /*
     * Convert point from global space to grid space that has its origin in the image center.
     */
    size_t nX = (size_t) (fX / m_fCellSize + 0.5f * float(m_nWidth));
    size_t nY = (size_t) (fY / m_fCellSize + 0.5f * float(m_nHeight));

    /*
     * Check if point is inside the map.
     */
    if (nX >= m_nWidth || nY >= m_nHeight)
    {
        ROS_WARN("Map: could not plot object at position [%d,%d].", nX, nY);
        return;
    }

    /*
     * Plot the point into the map. The new value of the affected cell will be set to a weighted mean between the old 
     * and the new value.
     *
     * !Notice: A cell's value always lies between 0 and 100, representing a probability percentage for occupation, or 
     *          at -1, for undiscovered.
     */
    char* pCell = m_pMapArray.get();
    char nValue = pCell[nY * m_nWidth + nX] * 0.75 + 100 * 0.25;
    pCell[nY * m_nWidth + nX] = nValue <= 100 ? nValue : 100;
}

void MapModel::addObstacle(const float& fPositionX, const float& fPositionY)
{
    this->addPoint(fPositionX, fPositionY);
}

void MapModel::placeRobot(const float& fPositionX, const float& fPositionY, const float& fOrientation)
{
    m_fRobotPositionX = fPositionX;
    m_nRobotPositionX = m_nWidth * 0.5 + fPositionX / m_fCellSize;
    m_fRobotPositionY = fPositionY;
    m_nRobotPositionY = m_nHeight * 0.5 + fPositionY / m_fCellSize;
    m_fRobotOrientation = fOrientation;

    /*
     * Set the cell that the robot is currently in to unoccupied.
     */
    removePoint(fPositionX, fPositionY);
}

void MapModel::placeObject(const float& fPositionX, const float& fPositionY)
{
    m_vecObjects.push_back(std::pair<float, float>(fPositionX, fPositionY));
}

void MapModel::exportMap(std::tr1::shared_ptr<map_creation::Map>& pMapMsg)
{
    if (m_pMapArray)
    {
        pMapMsg = std::tr1::shared_ptr<map_creation::Map>(new map_creation::Map());
        pMapMsg->nWidth = m_nWidth;
        pMapMsg->nHeight = m_nHeight;
        pMapMsg->fWidth = m_fWidth;
        pMapMsg->fHeight = m_fHeight;
        pMapMsg->fCellSize = m_fCellSize;
        pMapMsg->gridMap.assign(m_pMapArray.get(), m_pMapArray.get() + m_nMapSizeBytes);
        pMapMsg->fRobotPositionX = m_fRobotPositionX;
        pMapMsg->nRobotPositionX = m_nRobotPositionX;
        pMapMsg->fRobotPositionY = m_fRobotPositionY;
        pMapMsg->nRobotPositionY = m_nRobotPositionY;
        pMapMsg->fRobotOrientation = m_fRobotOrientation;
    }
}
