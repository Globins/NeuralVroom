// #include "include/pathfinder/smoother.hpp"

// Smoother::Smoother(Grid* grid)
// {
//     this->grid = grid;
// }
// vector<VehicleState> Smoother::smooth(vector<VehicleState> path)
// {
//     if(path.size() < 5)
//     {
//         return path;
//     }
//     int count = 0;
//     int delta = 0;
//     while(count < maxIterations && delta > eps)
//     {
//         for(int i = 2; i < path.size()-2; i++)
//         {
            
//             Coordinates2D correction = Coordinates2D();
//             Coordinates2D obstCorrection = obstDeriv(path[i]);
//             Coordinates2D voroCorrection = voroDeriv(path[i]);
//             Coordinates2D curveCorrection = curveDeriv(path[i]);
//             Coordinates2D smoothCorrection = smoothDeriv(path[i]);
//         }
//         count++;
//     }
//     return path;
// }


// Coordinates2D Smoother::obstDeriv(VehicleState current)
// {
//     Coordinates2D nearestObst = grid->getNearestObstDist((int)current.posX, (int)current.posX);
//     Coordinates2D nearestObstDiff = Coordinates2D{current.posX - nearestObst.x, current.posY - nearestObst.y};
//     float obstDist = 0;
//     if(obstDist <= obstDMax)
//     {
//         float scale = weightObst * (obstDist - obstDMax);
//         return Coordinates2D{scale*nearestObstDiff.x/obstDist, scale*nearestObstDiff.y/obstDist};
//     }
//     return Coordinates2D{0,0};
// }
// Coordinates2D Smoother::voroDeriv(VehicleState current)
// {
//     Coordinates2D nearestVoro = grid->getNearestVoroDist((int)current.posX, (int)current.posX);
    
//     return Coordinates2D{0,0};
// }
// Coordinates2D Smoother::curveDeriv(VehicleState current)
// {
//     return Coordinates2D{0,0};
// }
// Coordinates2D Smoother::smoothDeriv(VehicleState current)
// {
//     return Coordinates2D{0,0};
// }