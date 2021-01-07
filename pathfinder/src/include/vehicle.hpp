#ifndef VEHICLE_H
#define VEHICLE_H

#include "utils.hpp"

class Vehicle
{
    float width;
    float length;
    float maxTurnAngle;
    float turnRadius;
    float velocity;
public:
    Vehicle();
    VehicleState getNextState(VehicleState current, Steer steer, Gear gear, float delta_time);
};







#endif