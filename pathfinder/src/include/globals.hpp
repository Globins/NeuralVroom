#ifndef GLOBALS_H
#define GLOBALS
/* Commonly used structures and enums used throughout the program.*/
enum Gear{Forward = 0, Backward = 1};
enum Steer{Straight = 0, Left = 1, Right = 2};
const int NUM_GEARS = 2;
const int NUM_STEERS = 3;
struct PolarCoordinates
{
  float radius;
  float theta;
};

/* For Coordinates, x and y will always be discrete.*/
struct Coordinates2D
{
  float x;
  float y;
};
struct Coordinates3D
{
  float x;
  float y;
  float radians;
};
struct Coordinates4D
{
  float x;
  float y;
  float radians;
  Gear gear;
};

struct VehicleState
{
    float posX;
    float posY;
    float ori;
    Gear gear;
    Steer steer;
};


#endif