#include "include/grid.hpp"
#include "include/pathfinder/hybridAStar.hpp"
#include <iostream>
#include <fstream>
//grid measurements of 10x10 c
int main()
{
    Grid grid = Grid(40, 40);
    grid.addObstacle(0,0);
    Vehicle testcar = Vehicle(1, 1);
    HybridAStar pathfinder = HybridAStar(&grid);
    ReedsSheppsCurves curves = ReedsSheppsCurves();
    VehicleState current = VehicleState{35, 5, deg2rad(90), Forward, Straight};
    VehicleState goal = VehicleState{35, 35, deg2rad(260), Forward, Straight};
    Coordinates3D start = {35, 5, deg2rad(90)};
    Coordinates3D end = {35, 35, deg2rad(260)};
    vector<ReedsSheppsAction> rspath = curves.findOptimalPath(start, end);
    vector<VehicleState> path = curves.discretizePath(current, rspath, 1, 1);


    // vector<VehicleState> path = pathfinder.run(start, end, testcar);
    ofstream myfile;
    myfile.open("output.txt");
    for(VehicleState state : path)
    {
        myfile << "(" << state.posX << ", " << state.posY << ", " << state.ori << ", " << state.steer << ", " << state.gear << ")\n";
    }
    myfile.close();
}