#include "utils.hpp"
#include <vector>

Coordinates::Coordinates(int x, int y)
{
  x = x;
  y = y;
}
vector<Coordinates> GetNeighbors(int x, int y, int dimX, int dimY)
{
  vector<Coordinates> coords;
  if(pointInGrid(x, y, dimX, dimY))
  {
    if(x - 1 >= 0)
    {
      coords.push_back(Coordinates(x-1, y));
      if(y - 1 >= 0)
      {
        coords.push_back(Coordinates(x-1, y-1));
      }
      if(y + 1 < dimY)
      {
        coords.push_back(Coordinates(x-1, y+1));
      }
    }
    if(x + 1 < dimX)
    {
      coords.push_back(Coordinates(x+1, y));
      if(y - 1 >= 0)
      {
        coords.push_back(Coordinates(x+1, y-1));
      }
      if(y + 1 < dimY)
      {
        coords.push_back(Coordinates(x+1, y+1));
      }
    }
    if(y - 1 >= 0)
    {
      coords.push_back(Coordinates(x, y-1));
    }
    if(y + 1 < dimY)
    {
      coords.push_back(Coordinates(x, y+1));
    }
  }
  return coords;
}

bool pointInGrid(int x, int y, int dimX, int dimY)
{
  return (x >= 0 && y >= 0) && (x < dimX && y < dimY);
}

// #These functions are general functions can be used in mult. files
//
// def cart2pol(x, y):
//     """returns polar coords of point (x, y) in relation to (0,0)"""
//     r = np.sqrt(x*x + y*y)
//     theta = np.arctan2(y, x)
//     return r, theta
//
// def mod2Pi(theta):
//     """ returns angle phi = theta%2pi with contraint -pi <= theta < pi"""
//     theta = theta % (2*np.pi)
//     if theta < -np.pi: return theta + 2*np.pi
//     if theta >= np.pi: return theta - 2*np.pi
//     return theta
//
// def change_of_basis(start:tuple, end:tuple):
//     """translates the end point to be in start's coordinate system with start being (0,0)
//     start and end are passed in(x, y, orientation)"""
//     theta1 = np.deg2rad(start[2])
//     dx = end[0] - start[0]
//     dy = end[1] - start[1]
//     new_x = dx * np.cos(theta1) + dy*np.sin(theta1)
//     new_y = -dx * np.sin(theta1) + dy*np.cos(theta1)
//     new_theta = end[2] - start[2]
//     return new_x, new_y, new_theta


// def rotate(xy, radians):
//     """dot product of position and orientation"""
//     x, y = xy
//     c, s = np.cos(radians), np.sin(radians)
//     j = np.matrix([[c, -s], [s, c]])
//     m = np.dot(j, [x, y])
//
//     finalx = float(m.T[0])
//     finaly = float(m.T[1])
//     return finalx, finaly
//
// def contToCell(self):
//     """Converts the continuous value into a discrete-friendly value"""
//     pass
//
// def rpoint(x, y, r, deg):
//     """ Get a point on a radius at deg degrees."""
//     rang = np.deg2rad(deg)
//     x1 = x + r * np.cos(rang)
//     y1 = y + r * np.sin(rang)
//     return (x1, y1)
