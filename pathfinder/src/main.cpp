#include "include/grid.hpp"
#include "include/pathfinder/hybridAStar.hpp"
#include "include/mapGenerator.hpp"

#include <iostream>
#include <fstream>
#include <map>
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
    HybridAStar pathfinder = HybridAStar(&grid);
    //Neural Network training will go here
    //it will run randomly positioned car positions of varied amount of cars in a random map for 10k times
    map<int, Vehicle*> vehicleIDMap;


    // PATH WILL BE (X, Y, ORI, GEAR, STEER, TIMEATNODE)
    
    // while(1)
    // {
        //SERVER: Will run while loop that will take requests in form of:
        // VehicleID ObstUpdate [coords relative to current position]
        // VehicleID (StartPos) (EndPos)
        int vehicleID = 0;
        bool obstRequest = false;
        if(obstRequest)
        {
            //Find obst positon relative to current
        }
        else
        {
            VehicleState start = VehicleState{35, 5, deg2rad(90), Forward, Straight};
            VehicleState end = VehicleState{4, 31, deg2rad(180), Forward, Straight};
            if(!vehicleIDMap.count(vehicleID))
            {
                vehicleIDMap[vehicleID] = new Vehicle(1, 1);
            }
            vehicleIDMap[vehicleID]->current_path = pathfinder.run(start, end, *vehicleIDMap[vehicleID], true);
            //COMPARE ALL PATHS IN IDMAP
            bool collision = false;

            if(collision)
            {
                //RUN ANN
                //SEND PATHS TO VEHICLES AFFECTED
            }
            else
            {
                // SERVER: WILL SEND PATH TO VEHICLE FORM OF:
                // X Y ORI GEAR STEER
                ofstream myfile;
                myfile.open("output.txt");
                cout << "TEST" << endl;
                for(VehicleState state : vehicleIDMap[vehicleID]->current_path)
                {
                    myfile << "(" << state.posX << ", " << state.posY << ", " << state.ori << ", " << state.steer << ", " << state.gear << ")\n";
                }
                myfile.close();
            }
        }
    // }
}