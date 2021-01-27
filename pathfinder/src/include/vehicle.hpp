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
};







#endif