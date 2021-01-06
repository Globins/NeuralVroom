#ifndef UTILS_H
#define UTILS_H
#define _USE_MATH_DEFINES
#include <vector>
#include <math.h>
#include <iostream>

using namespace std;
struct Coordinates
{
  int x;
  int y;
};
struct PolarCoordinates
{
  float radius;
  float theta;
};
struct CoordinatesWithDirection
{
  float x;
  float y;
  float radians;
};
vector<Coordinates> GetNeighbors(int x, int y, int dimX, int dimY);
bool pointInGrid(int x, int y, int dimX, int dimY);
Coordinates rotate(float x, float y, float radians);
PolarCoordinates cart2pol(float x, float y);
float mod2PI(float theta);
CoordinatesWithDirection changeOfBasis(CoordinatesWithDirection start, CoordinatesWithDirection end);


#endif
