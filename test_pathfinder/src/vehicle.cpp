#include "vehicle.hpp"

Vehicle::Vehicle(float width, float length)
{
    this->width = width;
    this->length = length;
}
VehicleState Vehicle::getNextVehicleState(VehicleState currentState, Steer steer, Gear gear, float delta_time)
{
    float lengthtraveled = velocity * delta_time;
    float xPos, yPos = 0;
    float angle = 0;
    if(steer == 0) //Straight
    {
        xPos = lengthtraveled;
    }
    else
    {
        angle = lengthtraveled / turnRadius;
        float sinangle = sin(angle/2);
        float l = 2 * sinangle * turnRadius;
        xPos = l * cos(turnRadius);
        yPos = l * sinangle;
    }
    if(steer == 2)
    {
        yPos *= -1;
        angle *= -1;
    }
    if(gear == 1)
    {
        xPos *= -1;
        angle *= -1;
    }
    Coordinates correctPos = rotate(xPos, yPos, currentState.orientation);
    return currentState(current.x + correctPos.x, current.y + corectPos.y, angle + current.orientation, gear, steer);
}