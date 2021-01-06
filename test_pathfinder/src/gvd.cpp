#include "gvd.hpp"

GVD::GVD(int width, int height)
{
  this->width = width;
  this->height = height;
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

  cost.resize(width,vector<int>(height, numeric_limits<int>::max()));
}
void GVD::update()
{
  updateDist();
  updateVoro();
}
void GVD::setCell(int x, int y, bool isVoroMap)
{
  if(isVoroMap)
  {
    voroDistMap[x][y]->nearestX = x;
    voroDistMap[x][y]->nearestY = y;
    voroDistMap[x][y]->dist = 0;
    voroDistMap[x][y]->toProcess = true;
    voroOpen.push(*voroDistMap[x][y]);
  }
  else
  {
    distMap[x][y]->nearestX = x;
    distMap[x][y]->nearestY = y;
    distMap[x][y]->dist = 0;
    distMap[x][y]->toProcess = true;
    distOpen.push(*distMap[x][y]);
  }
}
void GVD::unsetCell(int x, int y, bool isVoroMap)
{
  if(isVoroMap)
  {
    voroDistMap[x][y]->nearestX = x;
    voroDistMap[x][y]->nearestY = y;
    voroDistMap[x][y]->dist = numeric_limits<float>::max();
    voroDistMap[x][y]->toProcess = true;
    voroDistMap[x][y]->toRaise = true;
    voroOpen.push(*voroDistMap[x][y]);
  }
  else
  {
    distMap[x][y]->nearestX = x;
    distMap[x][y]->nearestY = y;
    distMap[x][y]->dist = numeric_limits<float>::max();
    distMap[x][y]->toProcess = true;
    distMap[x][y]->toRaise = true;
    distOpen.push(*distMap[x][y]);
  }
}
bool GVD::isOccupied(Cell* current)
{
  return !isUnknown(current->x, current->y) and current->nearestX == current->x and current->nearestY == current->y;
}
bool GVD::isUnknown(int x, int y)
{
  return (x < 0 or y < 0) or (x > height or y > width);
}

void GVD::updateDist()
{
  while(distOpen.size())
  {
    Cell* current = &distOpen.top();
    if(current->toProcess)
    {
      if(current->toRaise)
      {
          raiseDist(current);
      }
      else if(isOccupied(current))
      {
        voro[current->x][current->y] = false;
        unsetCell(current->x, current->y, true);
        current->toProcess = false;
        lowerDist(current);
      }
    }
    distOpen.pop();
  }
}

void GVD::updateVoro()
{
  while(voroOpen.size())
  {
    Cell* current = &voroOpen.top();
    if(current->toProcess)
    {
      if(current->toRaise)
      {
          raiseVoro(current);
      }
      else if(isOccupied(current))
      {
        current->toProcess = false;
        lowerVoro(current);
      }
    }
    voroOpen.pop();
  }
}

void GVD::raiseDist(Cell* s_cell)
{
  for(Coordinates c : GetNeighbors(s_cell->x, s_cell->y, height, width))
  {
    if(!isUnknown(c.x, c.y) && !distMap[c.x][c.y]->toRaise)
    {
      if(!isOccupied(distMap[c.x][c.y]))
      {
        distMap[c.x][c.y]->nearestX = -1;
        distMap[c.x][c.y]->nearestY = -1;
        distMap[c.x][c.y]->dist = numeric_limits<float>::max();
        distMap[c.x][c.y]->toRaise = true;
        distOpen.push(*distMap[c.x][c.y]);
      }
    }
  }
  s_cell->toRaise = false;
}

void GVD::raiseVoro(Cell* s_cell)
{
  for(Coordinates c : GetNeighbors(s_cell->x, s_cell->y, height, width))
  {
    if(!isUnknown(c.x, c.y) && !voroDistMap[c.x][c.y]->toRaise)
    {
      if(!isOccupied(voroDistMap[c.x][c.y]))
      {
        voroDistMap[c.x][c.y]->nearestX = -1;
        voroDistMap[c.x][c.y]->nearestY = -1;
        voroDistMap[c.x][c.y]->dist = numeric_limits<float>::max();
        voroDistMap[c.x][c.y]->toRaise = true;
        voroOpen.push(*voroDistMap[c.x][c.y]);
      }
    }
  }
  s_cell->toRaise = false;
}

void GVD::lowerDist(Cell* s_cell)
{
  for(Coordinates c : GetNeighbors(s_cell->x, s_cell->y, height, width))
  {
    if(!distMap[c.x][c.y]->toRaise)
    {
      int obstacleX = distMap[c.x][c.y]->nearestX;
      int obstacleY = distMap[c.x][c.y]->nearestY;
      float dx = obstacleX - c.x;
      float dy = obstacleY - c.y;
      float distance = sqrt(dx*dx + dy*dy);
      if(distance < distMap[c.x][c.y]->dist)
      {
        distMap[c.x][c.y]->dist = distance;
        distMap[c.x][c.y]->nearestX = s_cell->x;
        distMap[c.x][c.y]->nearestY = s_cell->y;
        distMap[c.x][c.y]->toProcess = true;
        distOpen.push(*distMap[c.x][c.y]);
        
      }
      else
      {
        chkVoro(s_cell, distMap[c.x][c.y]);
      }
    }
  }
}

void GVD::lowerVoro(Cell* s_cell)
{
  for(Coordinates c : GetNeighbors(s_cell->x, s_cell->y, height, width))
  {
    if(!voroDistMap[c.x][c.y]->toRaise)
    {
      int obstacleX = voroDistMap[c.x][c.y]->nearestX;
      int obstacleY = voroDistMap[c.x][c.y]->nearestY;
      float dx = obstacleX - c.x;
      float dy = obstacleY - c.y;
      float distance = sqrt(dx*dx + dy*dy);
      if(distance < voroDistMap[c.x][c.y]->dist)
      {
        voroDistMap[c.x][c.y]->dist = distance;
        voroDistMap[c.x][c.y]->nearestX = s_cell->x;
        voroDistMap[c.x][c.y]->nearestY = s_cell->y;
        voroDistMap[c.x][c.y]->toProcess = true;
        voroOpen.push(*voroDistMap[c.x][c.y]);
      }
    }
  }
}

void GVD::chkVoro(Cell* s_cell, Cell* n_cell)
{
  if((distMap[s_cell->x][s_cell->y]->dist > 1 || distMap[n_cell->x][n_cell->y]->dist)
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