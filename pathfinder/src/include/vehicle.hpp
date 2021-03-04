#ifndef VEHICLE_H
#define VEHICLE_H

#include "utils.hpp"

class Vehicle
{
public:
    float width;
    float length;
    float maxTurnAngle;
    float turnRadius = .6;
    float velocity = 1;
    vector<VehicleState> current_path;
    Vehicle(float width, float length);
    VehicleState getNextState(VehicleState current, Steer steer, Gear gear, float delta_time);
    //MAP DETECTION
    vector<double> getDistanceFromObstacles(vector<vector<int>> m, VehicleState currentState);
    bool areEquivalentStates(VehicleState comp, VehicleState other);
private:
    vector<Coordinates3D> getSurroundingCoords(Coordinates3D currentPos, float dist);
    vector<vector<float>> getSlopes(Coordinates3D start, vector<Coordinates3D> surroundingCoords);
};







#endif