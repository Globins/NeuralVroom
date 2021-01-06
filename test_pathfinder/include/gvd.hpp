#ifndef GVD_H
#define GVD_H

#include "utils.hpp"
#include <queue>
#include <limits>
#include <functional>
#include <cmath>


class GVD{
public:
  GVD(int width, int height);
  void update();
  void setCell(int x, int y, bool isVoroMap);
  void unsetCell(int x, int y, bool isVoroMap);
  ~GVD();
private:
  int width;
  int height;

  struct Cell
  {
    float dist = numeric_limits<float>::max();
    int x, y = -1;
    int nearestX, nearestY = -1;
    bool toRaise, toProcess = false;
    bool operator>(const Cell &cell) const{
      return dist > cell.dist;
  }
  };

    //queue = (distance, (x,y))
  priority_queue<Cell, vector<Cell>, greater<Cell>> distOpen;
  vector<vector<Cell*>> distMap;
  priority_queue<Cell, vector<Cell>, greater<Cell>> voroOpen;
  vector<vector<Cell*>> voroDistMap;
  vector<vector<bool>> voro;
  //self.voroQ = PriorityQueue()
  vector<vector<int>> comp;

  vector<vector<int>> cost;

  const int alphaDefault = 20;
  const int dmaxDefault = 30;
  
  bool isOccupied(Cell* current);
  bool isUnknown(int x, int y);

  void updateDist();
  void raiseDist(Cell* s_cell);
  void lowerDist(Cell* s_cell);

  void updateVoro();
  void raiseVoro(Cell* s_cell);
  void lowerVoro(Cell* s_cell);
  void chkVoro(Cell* s_cell, Cell* n_cell);

};
#endif
