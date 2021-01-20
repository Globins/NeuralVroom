#ifndef GVD_H
#define GVD_H

#include "utils.hpp"
#include <queue>
#include <limits>
#include <functional>
//http://www.first-mm.eu/files/lau10iros.pdf
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

struct GVDcomparator
{
    bool operator()(const Cell * a, const Cell *b)
    {
        return a->dist > b->dist;
    }
};

class GVD{
public:
  GVD(int width, int height);
  void update();
  void setCell(int x, int y, bool isVoroMap);
  void unsetCell(int x, int y, bool isVoroMap);
  bool isOccupied(int x, int y, bool isVoroMap);
  bool isUnknown(int x, int y);
  ~GVD();
private:

  priority_queue<Cell*, vector<Cell*>, GVDcomparator> distOpen;
  priority_queue<Cell*, vector<Cell*>, GVDcomparator> voroOpen;
  
 
  

  void updateDist();
  void raiseDist(Cell* s_cell);
  void lowerDist(Cell* s_cell);

  void updateVoro();
  void raiseVoro(Cell* s_cell);
  void lowerVoro(Cell* s_cell);

  void chkVoro(Cell* s_cell, Cell* n_cell);

  void updateCostMap();

public:
  int width;
  int height;
  const int alphaDefault = 20;
  const int dmaxDefault = 30;
  
  vector<vector<bool>> voro;
  vector<vector<int>> comp;

  vector<vector<float>> cost;
  vector<vector<Cell*>> distMap; 
  vector<vector<Cell*>> voroDistMap;
};
#endif
