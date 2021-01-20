#include "include/pathfinder/smoother.hpp"

Smoother::Smoother(Grid* grid)
{
    this->grid = grid;
}
vector<VehicleState> Smoother::smooth(vector<VehicleState> path)
{
    if(path.size() < 5)
    {
        return path;
    }
    int count = 0;
    vector<VehicleState> current = path;
    while(count < maxIterations)
    {
        for(int i = 2; i < current.size()-2; i++)
        {
            if((current[i-2].gear != current[i-1].gear) || (current[i-1].gear != current[i].gear) || (current[i].gear != current[i+1].gear) || (current[i+1].gear != current[i+2].gear))
            {
                continue;
            }
            Coordinates2D position = Coordinates2D{path[i].posX, path[i].posY};
            Coordinates2D correction = Coordinates2D();
            
            // Coordinates2D obstCorrection = obstDeriv(current[i]);
            // Coordinates2D voroCorrection = voroDeriv(current[i]);
            // Coordinates2D curveCorrection = curveDeriv(current[i]);
            // Coordinates2D smoothCorrection = smoothDeriv(current[i]);
        }
        count++;
    }
    return current;
}


Coordinates2D Smoother::obstDeriv(VehicleState current)
{
    Coordinates2D nearestObst = grid->getNearestObstDist((int)current.posX, (int)current.posX);
    Coordinates2D nearestObstDiff = Coordinates2D{current.posX - nearestObst.x, current.posY - nearestObst.y};
    float obstDist = 0;
    if(obstDist <= obstDMax)
    {
        float scale = weightObst * (obstDist - obstDMax);
        return Coordinates2D{scale*nearestObstDiff.x/obstDist, scale*nearestObstDiff.y/obstDist};
    }
    return Coordinates2D{0,0};
}
Coordinates2D Smoother::voroDeriv(VehicleState current)
{
    Coordinates2D nearestVoro = grid->getNearestVoroDist((int)current.posX, (int)current.posX);
    
    return Coordinates2D{0,0};
}
Coordinates2D Smoother::curveDeriv(VehicleState current)
{
    return Coordinates2D{0,0};
}
Coordinates2D Smoother::smoothDeriv(VehicleState current)
{
    return Coordinates2D{0,0};
}