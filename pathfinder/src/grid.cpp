#include "include/grid.hpp"

Grid::Grid(int width, int height) : GVD(width, height)
{
    width = width;
    height = height;
}
void Grid::addObstacle(int x, int y)
{
    GVD::setCell(x, y, false);
    GVD::update();
}
void Grid::removeObstacle(int x, int y)
{
    GVD::unsetCell(x, y, false);
    GVD::update();
}
void Grid::refreshMap()
{
    GVD::update();
}
vector<vector<float>> Grid::returnDistMap()
{
    vector<vector<float>> result;
    for (auto row = distMap.begin(); row != distMap.end(); row++) {
        vector<float> pushRow;
        for (auto col = row->begin(); col != row->end(); col++) {
            Cell *cell = *col;
            float dist = cell->dist;
            pushRow.push_back(dist);
        }
        result.push_back(pushRow);
    }
    return result;
}
vector<vector<float>> Grid::returnVoroDistMap()
{
    vector<vector<float>> result;
    for (auto row = voroDistMap.begin(); row != voroDistMap.end(); row++) {
        vector<float> pushRow;
        for (auto col = row->begin(); col != row->end(); col++) 
        {
            Cell *cell = *col;
            float dist = cell->dist;
            pushRow.push_back(dist);
        }
        result.push_back(pushRow);
    }
    return result;
}
vector<vector<bool>> Grid::returnVoroMap()
{
    vector<vector<bool>> result;
    for (auto row = voro.begin(); row != voro.end(); row++) {
        vector<bool> pushRow;
        for (auto col = row->begin(); col != row->end(); col++) 
        {
            pushRow.push_back(*col);
        }
        result.push_back(pushRow);
    }
    return result;
}
bool Grid::isSafe(float safetyFactor)
{
    return false;
}

