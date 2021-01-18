#include "include/gvd.hpp"

// cost formula from https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf

GVD::GVD(int x_length, int y_length)
{
  width = y_length;
  height = x_length;
  distMap.resize(width,vector<Cell*>(height));
  voroDistMap.resize(width,vector<Cell*>(height));
  for(int i = 0; i < width; i++)
  {
    for(int q = 0; q < height; q++)
    {   
        distMap[i][q] = new Cell();
        distMap[i][q]->x = i;
        distMap[i][q]->y = q;

        voroDistMap[i][q] = new Cell();
        voroDistMap[i][q]->x = i;
        voroDistMap[i][q]->y = q;
    }
  }
  voro.resize(width,vector<bool>(height, false));
  comp.resize(width,vector<int>(height, -1));
  
  cost.resize(width,vector<float>(height, numeric_limits<float>::max()));
}

void GVD::update()
{
  updateDist();
  updateVoro();
  updateCostMap();
}
void GVD::setCell(int x, int y, bool isVoroMap)
{
  if(isVoroMap)
  {
    voroDistMap[x][y]->nearestX = x;
    voroDistMap[x][y]->nearestY = y;
    voroDistMap[x][y]->dist = 0;
    voroDistMap[x][y]->toProcess = true;
    voroOpen.push(voroDistMap[x][y]);
  }
  else
  {
    distMap[x][y]->nearestX = x;
    distMap[x][y]->nearestY = y;
    distMap[x][y]->dist = 0;
    distMap[x][y]->toProcess = true;
    distOpen.push(distMap[x][y]);
  }
}
void GVD::unsetCell(int x, int y, bool isVoroMap)
{
  if(isVoroMap)
  {
    voroDistMap[x][y]->nearestX = -1;
    voroDistMap[x][y]->nearestY = -1;
    voroDistMap[x][y]->dist = numeric_limits<float>::max();
    voroDistMap[x][y]->toProcess = true;
    voroDistMap[x][y]->toRaise = true;
    voroOpen.push(voroDistMap[x][y]);
  }
  else
  {
    distMap[x][y]->nearestX = -1;
    distMap[x][y]->nearestY = -1;
    distMap[x][y]->dist = numeric_limits<float>::max();
    distMap[x][y]->toProcess = true;
    distMap[x][y]->toRaise = true;
    distOpen.push(distMap[x][y]);
  }
}
bool GVD::isOccupied(int x, int y, bool isVoroMap)
{
  if(isVoroMap)
  {
    return !isUnknown(x, y) && voroDistMap[x][y]->nearestX == x && voroDistMap[x][y]->nearestY == y;
  }
  return !isUnknown(x, y) && distMap[x][y]->nearestX == x && distMap[x][y]->nearestY == y;
}
bool GVD::isUnknown(int x, int y)
{
  return (x < 0 || y < 0) || (x > width || y > height);
}
void GVD::updateDist()
{
  while(distOpen.size())
  {
    auto current = distOpen.top();
    distOpen.pop();
    if(current->toProcess)
    {
      if(current->toRaise)
      {
          raiseDist(current);
      }
      else if(isOccupied(current->nearestX, current->nearestY, false))
      {
        voro[current->x][current->y] = false;
        unsetCell(current->x, current->y, true);
        current->toProcess = false;
        lowerDist(current);
      }
    }
  }
}
void GVD::updateVoro()
{
  while(voroOpen.size())
  {
    auto current = voroOpen.top();
    voroOpen.pop();
    if(current->toProcess)
    {
      if(current->toRaise)
      {
          raiseVoro(current);
      }
      else if(isOccupied(current->nearestX, current->nearestY, true))
      {
        current->toProcess = false;
        lowerVoro(current);
      }
    }
  }
}

void GVD::raiseDist(Cell* s_cell)
{
  for(Coordinates2D c : GetNeighbors(s_cell->x, s_cell->y, width, height))
  {
    int x = (int)c.x;
    int y = (int)c.y;
    int tempX = distMap[x][y]->nearestX;
    int tempY = distMap[x][y]->nearestY;
    if(tempX != -1 && tempY != -1 && !distMap[x][y]->toRaise)
    {
      if(!isOccupied(tempX, tempY, false))
      {
        distMap[x][y]->nearestX = -1;
        distMap[x][y]->nearestY = -1;
        distMap[x][y]->dist = numeric_limits<float>::max();
        distMap[x][y]->toRaise = true;
      }
      distMap[x][y]->toProcess = true;
      distOpen.push(distMap[x][y]);
    }
  }
  distMap[s_cell->x][s_cell->y]->toRaise = false;
}

void GVD::raiseVoro(Cell* s_cell)
{
  for(Coordinates2D c : GetNeighbors(s_cell->x, s_cell->y, width, height))
  {
    int x = (int)c.x;
    int y = (int)c.y;
    int tempX = voroDistMap[x][y]->nearestX;
    int tempY = voroDistMap[x][y]->nearestY;
    if(tempX != -1 && tempY != -1 && !voroDistMap[x][y]->toRaise)
    {
      if(!isOccupied(tempX, tempY, true))
      {
        voroDistMap[x][y]->nearestX = -1;
        voroDistMap[x][y]->nearestY = -1;
        voroDistMap[x][y]->dist = numeric_limits<float>::max();
        voroDistMap[x][y]->toRaise = true;
      }
      voroDistMap[x][y]->toProcess = true;
      voroOpen.push(voroDistMap[x][y]);
    }
  }
  voroDistMap[s_cell->x][s_cell->y]->toRaise = false;
}

void GVD::lowerDist(Cell* s_cell)
{
  for(Coordinates2D c : GetNeighbors(s_cell->x, s_cell->y, width, height))
  {
    int x = (int)c.x;
    int y = (int)c.y;
    if(!distMap[x][y]->toRaise)
    {
      int obstacleX = distMap[s_cell->x][s_cell->y]->nearestX;
      int obstacleY = distMap[s_cell->x][s_cell->y]->nearestY;
      float dx = obstacleX - x;
      float dy = obstacleY - y;
      float distance = sqrt(dx*dx + dy*dy);
      if(distance < distMap[x][y]->dist)
      {
        distMap[x][y]->dist = distance;
        distMap[x][y]->nearestX = s_cell->nearestX;
        distMap[x][y]->nearestY = s_cell->nearestY;
        distMap[x][y]->toProcess = true;
        distOpen.push(distMap[x][y]);
      }
      else
      {
        chkVoro(s_cell, distMap[x][y]);
      }
    }
  }
}

void GVD::lowerVoro(Cell* s_cell)
{
  for(Coordinates2D c : GetNeighbors(s_cell->x, s_cell->y, width, height))
  {
    int x = (int)c.x;
    int y = (int)c.y;
    if(!voroDistMap[x][y]->toRaise)
    {
      int obstacleX = voroDistMap[s_cell->x][s_cell->y]->nearestX;
      int obstacleY = voroDistMap[s_cell->x][s_cell->y]->nearestY;
      float dx = obstacleX - x;
      float dy = obstacleY - y;
      float distance = sqrt(dx*dx + dy*dy);
      if(distance < voroDistMap[x][y]->dist)
      {
        
        voroDistMap[x][y]->dist = distance;
        voroDistMap[x][y]->nearestX = s_cell->nearestX;
        voroDistMap[x][y]->nearestY = s_cell->nearestY;
        voroDistMap[x][y]->toProcess = true;
        voroOpen.push(voroDistMap[x][y]);
      }
    }
  }
}

void GVD::chkVoro(Cell* s_cell, Cell* n_cell)
{
  if((distMap[s_cell->x][s_cell->y]->dist > 1 || distMap[n_cell->x][n_cell->y]->dist > 1)
    && (distMap[s_cell->x][s_cell->y]->nearestX !=  distMap[n_cell->x][n_cell->y]->nearestX)
    && (distMap[s_cell->x][s_cell->y]->nearestY !=  distMap[n_cell->x][n_cell->y]->nearestY))
  {
    int obstacleNX = distMap[n_cell->x][n_cell->y]->nearestX;
    int obstacleNY = distMap[n_cell->x][n_cell->y]->nearestY;
    float ndx = s_cell->x - obstacleNX;
    float ndy = s_cell->y - obstacleNY;
    float nDist = sqrt(ndx*ndx + ndy*ndy);

    int obstacleSX = distMap[s_cell->x][s_cell->y]->nearestX;
    int obstacleSY = distMap[s_cell->x][s_cell->y]->nearestY;
    float sdx = n_cell->x - obstacleSX;
    float sdy = n_cell->y - obstacleSY;
    float sDist = sqrt(sdx*sdx + sdy*sdy);

    float compS = nDist - distMap[s_cell->x][s_cell->y]->dist;
    float compN = sDist - distMap[n_cell->x][n_cell->y]->dist;

    if(compS <= compN)
    {
      voro[s_cell->x][s_cell->y] = true;
      setCell(s_cell->x, s_cell->y, true);
    }
    if(compN <= compS)
    {
      voro[n_cell->x][n_cell->y] = true;
      setCell(n_cell->x, n_cell->y, true);
    }
  }

}

void GVD::updateCostMap()
{
  for(int i = 0; i < width; i++)
  {
    for(int q = 0; q < height; q++)
    {   
      if(distMap[i][q]->dist >= dmaxDefault || voroDistMap[i][q]->dist == numeric_limits<float>::max())
      {
        cost[i][q] = 0;
      }
      else
      {
         cost[i][q] = (alphaDefault / (alphaDefault + distMap[i][q]->dist)) * (voroDistMap[i][q]->dist / (distMap[i][q]->dist + voroDistMap[i][q]->dist)) * ((dmaxDefault - distMap[i][q]->dist) / (dmaxDefault * dmaxDefault));
      }
      
    }
  }
}
GVD::~GVD() 
{ 
  for(int i = 0; i < width; i++)
  {
    for(int q = 0; q < height; q++)
    {
      delete distMap[i][q];
      delete voroDistMap[i][q];
    }
  }
}