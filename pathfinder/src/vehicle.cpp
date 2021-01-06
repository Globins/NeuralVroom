#include "include/vehicle.hpp"

VehicleState Vehicle::getNextState(VehicleState current, Steer steer, Gear gear, float delta_time)
{
    float length = velocity*delta_time;
    float xPos, yPos, angle = 0;
    if(steer == Straight)
    {
        xPos = length;
    }
    else
    {
        angle = length / turnRadius;
        float sinAngle = sin(angle/2);
        float l = 2 * sinAngle * turnRadius;
        xPos = l* cos(turnRadius);
        yPos = l * sinAngle;
    }
    if(steer == Right)
    {
        yPos *= -1;
        angle *= -1;
    }
    if(gear == Backward)
    {
        xPos *= -1;
        angle *= -1;
    }
    Coordinates correctedPos = rotate(xPos, yPos, current.ori);
    VehicleState copy = current;
    copy.posX = correctedPos.x;
    copy.posY = correctedPos.y;
    copy.ori = angle + current.ori;
    return copy;
}