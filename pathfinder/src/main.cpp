#include "include/grid.hpp"
#include "include/pathfinder/hybridAStar.hpp"
//grid measurements of 10x10 c
int main()
{
    Grid grid = Grid(40, 40);
    Vehicle testcar = Vehicle(1, 1);
    HybridAStar pathfinder = HybridAStar(&grid);
    VehicleState start = VehicleState{5, 35, deg2rad(0), Forward, Straight};
    VehicleState end = VehicleState{35, 35, deg2rad(0), Forward, Straight};
    vector<VehicleState> path = pathfinder.run(start, end, testcar);
    // for(VehicleState state : path)
    // {
    //     cout << "(" << state.posX << ", " << state.posY << ", " << state.ori << ", " << state.steer << ", " << state.gear << ")" << endl;
    // }
}