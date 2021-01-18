#include "include/utils.hpp"

vector<Coordinates2D> GetNeighbors(int x, int y, int dimX, int dimY)
{
  vector<Coordinates2D> coords;
  if(pointInGrid(x, y, dimX, dimY))
  {
    if(x - 1 >= 0)
    {
      Coordinates2D location;
      location.x = x-1;
      location.y = y;
      coords.push_back(location);
      if(y - 1 >= 0)
      {
        Coordinates2D otherLocation;
        otherLocation.x = x-1;
        otherLocation.y = y-1;
        coords.push_back(otherLocation);
      }
      if(y + 1 < dimY)
      {
        Coordinates2D otherLocation;
        otherLocation.x = x-1;
        otherLocation.y = y+1;
        coords.push_back(otherLocation);
      }
    }
    if(x + 1 < dimX)
    {
      Coordinates2D location;
      location.x = x+1;
      location.y = y;
      coords.push_back(location);
      if(y - 1 >= 0)
      {
        Coordinates2D otherLocation;
        otherLocation.x = x+1;
        otherLocation.y = y-1;
        coords.push_back(otherLocation);
      }
      if(y + 1 < dimY)
      {
        Coordinates2D otherLocation;
        otherLocation.x = x+1;
        otherLocation.y = y+1;
        coords.push_back(otherLocation);
      }
    }
    if(y - 1 >= 0)
    {
      Coordinates2D location;
      location.x = x;
      location.y = y-1;
      coords.push_back(location);
    }
    if(y + 1 < dimY)
    {
      Coordinates2D location;
      location.x = x;
      location.y = y+1;
      coords.push_back(location);
    }
  }
  return coords;
}

bool pointInGrid(int x, int y, int dimX, int dimY)
{
  return (x >= 0 && y >= 0) && (x < dimX && y < dimY);
}
Coordinates2D rotate(float x, float y, float radians)
{
  float c = cos(radians);
  float s = sin(radians);
  Coordinates2D rotatedPts;
  rotatedPts.x = x*c - y*s;
  rotatedPts.y = x*s + y*c;
  return rotatedPts;
}

PolarCoordinates cart2pol(float x, float y)
{
  float angle = atan2(y, x);
  PolarCoordinates result = {sqrt(x*x + y*y), angle};
  return result;
}

float mod2PI(float theta)
{
  theta = fmod(theta, M_PI*2);
  if(theta < -M_PI)
  {
    return theta + M_PI*2;
  }
  if(theta >= M_PI)
  {
    return theta - M_PI*2;
  }
  return theta;
}

Coordinates3D changeOfBasis(Coordinates3D start, Coordinates3D end)
{
  float dx = end.x - start.x;
  float dy = end.y - start.y;
  float new_x = dx * cos(start.radians) + dy*sin(start.radians);
  float new_y = -dx * sin(start.radians) + dy * cos(start.radians);
  float new_theta = end.radians - start.radians;
  return Coordinates3D{new_x, new_y, new_theta};
}

// #These functions are general functions can be used in mult. files

//
// def rpoint(x, y, r, deg):
//     """ Get a point on a radius at deg degrees."""
//     rang = np.deg2rad(deg)
//     x1 = x + r * np.cos(rang)
//     y1 = y + r * np.sin(rang)
//     return (x1, y1)


float deg2rad(float deg)
{
  return deg * M_PI / 180.0;
}