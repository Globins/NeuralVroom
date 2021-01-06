#include "include/utils.hpp"

vector<Coordinates> GetNeighbors(int x, int y, int dimX, int dimY)
{
  vector<Coordinates> coords;
  if(pointInGrid(x, y, dimX, dimY))
  {
    if(x - 1 >= 0)
    {
      Coordinates location;
      location.x = x-1;
      location.y = y;
      coords.push_back(location);
      if(y - 1 >= 0)
      {
        Coordinates otherLocation;
        otherLocation.x = x-1;
        otherLocation.y = y-1;
        coords.push_back(otherLocation);
      }
      if(y + 1 < dimY)
      {
        Coordinates otherLocation;
        otherLocation.x = x-1;
        otherLocation.y = y+1;
        coords.push_back(otherLocation);
      }
    }
    if(x + 1 < dimX)
    {
      Coordinates location;
      location.x = x+1;
      location.y = y;
      coords.push_back(location);
      if(y - 1 >= 0)
      {
        Coordinates otherLocation;
        otherLocation.x = x+1;
        otherLocation.y = y-1;
        coords.push_back(otherLocation);
      }
      if(y + 1 < dimY)
      {
        Coordinates otherLocation;
        otherLocation.x = x+1;
        otherLocation.y = y+1;
        coords.push_back(otherLocation);
      }
    }
    if(y - 1 >= 0)
    {
      Coordinates location;
      location.x = x;
      location.y = y-1;
      coords.push_back(location);
    }
    if(y + 1 < dimY)
    {
      Coordinates location;
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
Coordinates rotate(float x, float y, float radians)
{
  float c = cos(radians);
  float s = sin(radians);
  Coordinates rotatedPts;
  rotatedPts.x = x*c + x*(s*-1);
  rotatedPts.y = y*s+y*c;
  return rotatedPts;
}

PolarCoordinates cart2pol(float x, float y)
{
  PolarCoordinates result = {sqrt(x*x + y*y), atan2(y, x)};
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

CoordinatesWithDirection changeOfBasis(CoordinatesWithDirection start, CoordinatesWithDirection end)
{
  float dx = end.x - start.x;
  float dy = end.y - start.y;
  float new_x = dx * cos(start.radians) + dy*sin(start.radians);
  float new_y = -dx * sin(start.radians) + dy * cos(start.radians);
  float new_theta = end.radians - start.radians;
  return CoordinatesWithDirection{new_x, new_y, new_theta};
}

// #These functions are general functions can be used in mult. files

//
// def rpoint(x, y, r, deg):
//     """ Get a point on a radius at deg degrees."""
//     rang = np.deg2rad(deg)
//     x1 = x + r * np.cos(rang)
//     y1 = y + r * np.sin(rang)
//     return (x1, y1)
