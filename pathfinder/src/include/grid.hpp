#ifndef GRID_H
#define GRID_H
#include "gvd.hpp"

class Grid : public GVD
{
    vector<Coordinates2D> obstacleList;
public:
    Grid(int width, int height);
    void addObstacle(int x, int y);
    void removeObstacle(int x, int y);
    void refreshMap();
    vector<vector<float>> returnDistMap();
    vector<vector<float>> returnVoroDistMap();
    vector<vector<bool>> returnVoroMap();
    vector<vector<Coordinates2D>> returnNearest();
    bool isSafe(VehicleState state, float safetyFactor);
    void removeFromObstacleList(int x, int y);

    Coordinates2D getNearestObstDist(int x, int y);
    Coordinates2D getNearestVoroDist(int x, int y);

};

#endif
