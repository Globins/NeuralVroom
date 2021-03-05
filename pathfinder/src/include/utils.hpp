#ifndef UTILS_H
#define UTILS_H
#define _USE_MATH_DEFINES
#include <vector>
#include <math.h>
#include <iostream> 
#include <algorithm>
#include <set>
#include <stdlib.h>
#include "globals.hpp"
#include <iomanip> 
#include <fstream>

using namespace std;

vector<Coordinates2D> GetNeighbors(int x, int y, int dimX, int dimY);

bool pointInGrid(int x, int y, int dimX, int dimY);

Coordinates2D rotate(float x, float y, float radians);

PolarCoordinates cart2pol(float x, float y);

float mod2PI(float theta);

Coordinates3D changeOfBasis(Coordinates3D start, Coordinates3D end);

float deg2rad(float deg);

#endif
