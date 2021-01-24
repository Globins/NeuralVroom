#include "include/grid.hpp"
#include "include/pathfinder/hybridAStar.hpp"
#include "include/mapGenerator.hpp"

#include <iostream>
#include <fstream>
//grid measurements of 10x10 c
int main()
{
    //mapGenerator map = mapGenerator(50, 50, vector<int>{5,5},7);
    Grid grid = Grid(40, 40);
    grid.addObstacle(35,25);
    grid.addObstacle(36,25);
    grid.addObstacle(37,25);
    grid.addObstacle(38,25);
    grid.addObstacle(39,25);
    grid.addObstacle(34,25);
    grid.addObstacle(33,25);
    grid.addObstacle(32,25);
    grid.addObstacle(31,25);
    grid.addObstacle(30,25);
    Vehicle testcar = Vehicle(1, 1);
    HybridAStar pathfinder = HybridAStar(&grid);
    VehicleState start = VehicleState{35, 5, deg2rad(90), Forward, Straight};
    VehicleState end = VehicleState{4, 31, deg2rad(180), Forward, Straight};
    vector<VehicleState> path = pathfinder.run(start, end, testcar, true);
    ofstream myfile;
    myfile.open("output.txt");
    cout << "TEST" << endl;
    for(VehicleState state : path)
    {
        myfile << "(" << state.posX << ", " << state.posY << ", " << state.ori << ", " << state.steer << ", " << state.gear << ")\n";
    }
    myfile.close();
}