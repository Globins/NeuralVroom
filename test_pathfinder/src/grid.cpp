#include "grid.h"

Grid::Grid(int width, int height)
{
    GVD(width, height);
}
void Grid::addObstacle(int x, int y)
{
    GVD.setCell(x, y, false);
}
void Grid::removeObstacle(int x, int y)
{
    GVD.unsetCell(x, y, false);
}
void Grid::refreshMap()
{

}
vector<vector<float>> Grid::returnDistMap()
{

}
vector<vector<float>>Grid::returnVoroDistMap()
{

}
bool Grid::isSafe(float safetyFactor)
{
    return false;
}