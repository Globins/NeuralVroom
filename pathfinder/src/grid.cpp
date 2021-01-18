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
    obstacleList.push_back(Coordinates2D{(float)x, (float)y});

}
void Grid::removeObstacle(int x, int y)
{
    GVD::unsetCell(x, y, false);
    GVD::update();
    removeFromObstacleList(x, y);

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
vector<vector<Coordinates2D>> Grid::returnNearest()
{
    vector<vector<Coordinates2D>> result;
    for (auto row = voroDistMap.begin(); row != voroDistMap.end(); row++) {
        vector<Coordinates2D> pushRow;
        for (auto col = row->begin(); col != row->end(); col++) 
        {
            Cell *cell = *col;
            pushRow.push_back(Coordinates2D{(float)cell->nearestX, (float)cell->nearestY});
        }
        result.push_back(pushRow);
    }
    return result;
}
bool Grid::isSafe(VehicleState state, float safetyFactor)
{
    int x = (int)state.posX;
    int y = (int)state.posY;
    return distMap[x][y]->dist >= safetyFactor;
}
void Grid::removeFromObstacleList(int x, int y)
{
    int index = -1;
    for (auto it = obstacleList.begin(); it != obstacleList.end(); ++it) {
        index = std::distance(obstacleList.begin(), it);
        if(it->x == x && it->y == y)
        {
            break;
        }
    }
    if(index != -1)
    {
        obstacleList.erase(obstacleList.begin() + index);
    }
}


Coordinates2D Grid::getNearestObstDist(int x, int y)
{
    return Coordinates2D{(float)distMap[x][y]->nearestX, (float)distMap[x][y]->nearestY};
}
Coordinates2D Grid::getNearestVoroDist(int x, int y)
{
    return Coordinates2D{(float)voroDistMap[x][y]->nearestX, (float)voroDistMap[x][y]->nearestY};
}

vector<vector<float>> Grid::nonHolonomicRelaxedCostMap(VehicleState goal)
{
    vector<vector<float>> heuristicMap;
    heuristicMap.resize(width,vector<float>(height, numeric_limits<float>::max()));
    priority_queue<DijkstraStruct, vector<DijkstraStruct>, DijkstraStructComp> heuristicQueue;

    DijkstraStruct start = DijkstraStruct{goal.posX, goal.posY, 0};
    heuristicMap[goal.posX][goal.posY] = 0;

    heuristicQueue.push(start);
    while(!heuristicQueue.empty())
    {
        DijkstraStruct current = heuristicQueue.top();
        heuristicQueue.pop();
        for(Coordinates2D n : GetNeighbors(current.x, current.y, width, height))
        {
            float dist = current.dist + 1;
            if(dist < heuristicMap[n.x][n.y] && !isOccupied(n.x, n.y, false))
            {
                DijkstraStruct neighbor = DijkstraStruct{n.x, n.y, dist};
                heuristicMap[n.x][n.y] = dist;
                heuristicQueue.push(neighbor);
            }
        }
    }
    return heuristicMap;
}