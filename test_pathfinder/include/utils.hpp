#ifndef UTILS_H
#define UTILS_H

#include <vector>

using namespace std;
struct Coordinates
{
  int x;
  int y;
  Coordinates(int x, int y);
};
vector<Coordinates> GetNeighbors(int x, int y, int dimX, int dimY);
bool pointInGrid(int x, int y, int dimX, int dimY);

#endif
