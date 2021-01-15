#include "include/vehicle.hpp"

Vehicle::Vehicle(float width, float length)
{
    this->width = width;
    this->length = length;
}
VehicleState Vehicle::getNextState(VehicleState current, Steer steer, Gear gear, float delta_time)
{
    float length = velocity*delta_time;
    float xPos = 0;
    float yPos = 0;
    float angle = 0;
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

    Coordinates2D correctedPos = rotate(xPos, yPos, current.ori);
    current.posX += correctedPos.x;
    current.posY += correctedPos.y;
    current.ori += angle;
    current.steer = steer;
    current.gear = gear;
    return current;
}