#ifndef VEHICLE_H
#define VEHICLE_H

#include "utils.hpp"

enum Gear{Forward = 0, Backward = 0};
enum Steer{Straight = 0, Left = 1, Right = 2};


struct VehicleState
{
    float posX;
    float posY;
    float ori;
    Gear gear;
    Steer steer;
};

class Vehicle
{
    float width;
    float length;
    int numGears = 2;
    int numSteers = 3;
    float maxTurnAngle;
    float turnRadius;
    float velocity;
public:
    Vehicle();
    VehicleState getNextState(VehicleState current, Steer steer, Gear gear, float delta_time);
};







#endif