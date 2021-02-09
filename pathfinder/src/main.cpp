#include "include/grid.hpp"
#include "include/pathfinder/hybridAStar.hpp"
#include "include/mapGenerator.hpp"
#include "include/pathrouter/neuralnetwork.hpp"

#include <iostream>
#include <fstream>
#include <map>

//grid measurements of 10x10 c
vector<vector<float>> getSurroundingCoords(vector<float> currentPos, float dist){
    float currentOrientation = currentPos[2];
    vector<vector<float>> surroundingCoords;
    for(int i = 0; i < 24; i++){
        if(currentOrientation < 90){
            float y = sin(currentOrientation*(M_PI/180))*dist;
            float x = cos(currentOrientation*(M_PI/180))*dist;
            // cout << x << " " << y  << " " << currentOrientation << endl;
            surroundingCoords.push_back(vector<float>{currentPos[0]+x, currentPos[1]-y, currentOrientation});
        }
        else if(currentOrientation >= 90 && currentOrientation < 180){
            float tempOrientation = currentOrientation - 90;
            float y = cos(tempOrientation*(M_PI/180))*dist;
            float x = sin(tempOrientation*(M_PI/180))*dist;
            // cout << x << " " << y  << " " << currentOrientation << endl;
            surroundingCoords.push_back(vector<float>{currentPos[0]-x, currentPos[1]-y, currentOrientation});
        }
        else if(currentOrientation >= 180 && currentOrientation < 270){
            float tempOrientation = currentOrientation - 180;
            float y = sin(tempOrientation*(M_PI/180))*dist;
            float x = cos(tempOrientation*(M_PI/180))*dist;
            // cout << x << " " << y  << " " << currentOrientation << endl;
            surroundingCoords.push_back(vector<float>{currentPos[0]-x, currentPos[1]+y, currentOrientation});
        }
        else if(currentOrientation >= 270 && currentOrientation < 360){
            float tempOrientation = currentOrientation - 270;
            float y = cos(tempOrientation*(M_PI/180))*dist;
            float x = sin(tempOrientation*(M_PI/180))*dist;
            // cout << x << " " << y  << " " << currentOrientation << endl;
            surroundingCoords.push_back(vector<float>{currentPos[0]+x, currentPos[1]+y, currentOrientation});
        }
        currentOrientation += 15;
        if (currentOrientation >= 360)
            currentOrientation -= 360;
    }
    return surroundingCoords;
}

vector<vector<float>> getSlopes(vector<float> start, vector<vector<float>> surroundingCoords){
    vector<vector<float>> slopes;
    for(vector<float> coord : surroundingCoords){
        float rise = abs(coord[1] - start[1]);
        float run = abs(coord[0] - start[0]);
        slopes.push_back(vector<float>{abs(rise), abs(run)});        
    }
    return slopes;
}

vector<double> getInputs(vector<vector<int>> m, Vehicle* vehicle, vector<float> currentPos, vector<float> endPos){
    float dist = 3.1;
    vector<float> inputs;
    vector<vector<float>> surroundingCoords = getSurroundingCoords(currentPos, dist);
    vector<vector<float>> slopes = getSlopes(currentPos, surroundingCoords);
    vector<double> distances;
    for(int i = 0; i < slopes.size(); i++){
        // cout << "("<< surroundingCoords[i][0] << "," << surroundingCoords[i][1] << "),";
        float tempX = currentPos[0];
        float tempY = currentPos[1];
        for(int j = 0; j < dist*2; j++){
            if(m[roundf(tempY)][roundf(tempX)] == 1){
                distances.push_back( (dist*2 - j)/(dist*2)  );
                cout << tempY << " " << tempX << " " << surroundingCoords[i][2] << endl;
                break;
            }
            if(surroundingCoords[i][2] > 0 && surroundingCoords[i][2] < 180 ){
                tempY -= slopes[i][0]/(dist*2);
            }
            else{
                tempY += slopes[i][0]/(dist*2);
            }
            if(surroundingCoords[i][2] > 90 && surroundingCoords[i][2] < 270 ){
                tempX -= slopes[i][1]/(dist*2);
            }
            else{
                tempX += slopes[i][1]/(dist*2);
            }
            if(tempX >= 50 || tempY >= 50 || tempX < 0 || tempY < 0){
                distances.push_back(1);
                cout << tempY << " " << tempX << " " << surroundingCoords[i][2] << endl;
                break;
            }
        }
        if(distances.size() != i+1){
            distances.push_back(0);
            
        }
    }
    m[currentPos[1]][currentPos[0]] = 9;
    cout << currentPos[0] << " " << currentPos[1] << endl;
    for(int i = 0; i < distances.size(); i++){
        cout <<  surroundingCoords[i][0] << " " << surroundingCoords[i][1] << " " << surroundingCoords[i][2] << endl;
        cout << "\t" << slopes[i][0] << " " << slopes[i][1] << endl;
        cout << "\t" << distances[i] << endl;
    }
    m[currentPos[1]][currentPos[0]] = 9;
    for(int i = 0; i < m.size(); i++){
        for(int y = 0; y < m[i].size(); y++){
            cout << m[i][y] << " ";
        }
        cout << endl;
    }
    return distances;
}


int main()
{
    int numVehicles = 1;
    mapGenerator m = mapGenerator(50, 50, vector<int>{3,3},7,numVehicles);
    // cout << m << endl;
    Grid grid = Grid(55, 55);
    
    cout << "adding obstacles" << endl;
    for(vector<int> obstacle: m.getObstacles()){
        grid.addObstacle(obstacle[0] ,obstacle[1]);
    }
    cout << "finished adding obs" << endl;

    HybridAStar pathfinder = HybridAStar(&grid);
    //Neural Network training will go here
    //it will run randomly positioned car positions of varied amount of cars in a random map for 10k times
    map<int, Vehicle*> vehicleIDMap;
    vector<unsigned> topology {24, 12, 6, 3, 3};

    // PATH WILL BE (X, Y, ORI, GEAR, STEER, TIMEATNODE)
    
    // while(1)
    // {
        //SERVER: Will run while loop that will take requests in form of:
        // VehicleID ObstUpdate [coords relative to current position]
        // VehicleID (StartPos) (EndPos)
        vector<int> vehiclesToUpdate;
        for(int i = 0; i < numVehicles; i++){
            vehiclesToUpdate.push_back(i);
        }

        bool obstRequest = false;
        if(obstRequest)
        {
            //Find obst positon relative to current
        }
        else
        {
            cout << "creating paths" << endl;
            vector<vector<float>> startPoints = m.getStartPoints();
            vector<vector<float>> endPoints = m.getEndPoints();
            for(int id: vehiclesToUpdate){
                VehicleState start = VehicleState{startPoints[id][0], startPoints[id][1], deg2rad(startPoints[id][2]), Forward, Straight};
                VehicleState end = VehicleState{endPoints[id][0], endPoints[id][1], deg2rad(endPoints[id][2]), Forward, Straight};
                if(!vehicleIDMap.count(id))
                {
                    vehicleIDMap[id] = new Vehicle(1, 1);
                }
                vector<double> inputs = getInputs(m.getMap(), vehicleIDMap[id], startPoints[id], endPoints[id]);
                vector<double> results;
                NeuralNet net = NeuralNet(topology);
                net.feedForward(inputs);
                net.getResults(results);
                for(int i = 0; i < results.size(); i++){
                    cout << results[i] << endl;
                }
                vehicleIDMap[id]->current_path = pathfinder.run(start, end, *vehicleIDMap[id], true);
            }
            cout << "paths created" << endl;
        }
        //COMPARE ALL PATHS IN IDMAP
        bool collision = false;
        
        if(collision)
        {
            //RUN ANN
        }
        ofstream myfile;
        myfile.open("output.txt");
        for(int id: vehiclesToUpdate)
        {
            // SERVER: WILL SEND PATH TO VEHICLE FORM OF:
            // X Y ORI STEER GEAR 
            
            
            cout << "writing path" << endl;
            myfile << "\n\n" << id << "\n";
            for(VehicleState state : vehicleIDMap[id]->current_path)
            {
                myfile << "(" << state.posX << ", " << state.posY << ", " << state.ori << ", " << state.steer << ", " << state.gear << "),";
            }
            
        }
        myfile.close();

        

    // }
}