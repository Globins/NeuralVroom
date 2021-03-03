#ifndef MAPGENERATOR_H
#define MAPGENERATOR_H

#include "utils.hpp"
#include "grid.hpp"
#include <time.h> 
#include <map>



class mapGenerator
{
private:
    vector<vector<float>> startPoints;
    vector<vector<float>> endPoints;
    vector<int> blocks;
    vector<int> blockDimensions;
    set<vector<vector<int>>> streets;
    set<vector<vector<int>>> intersections;
    vector<vector<int>> mp;


    int vehicleNum;
    int spacer;
    int rows;
    int cols;

    Grid *grid;

    void generateMap();
    void setObsRanges();
    void setObsCoordinates(vector<vector<int>> xRanges,  vector<vector<int>> yRanges);
    void setStreetsIntersections(set<vector<int>> freeSpaceX,set<vector<int>> freeSpaceY);
    void setStartEndPoints();
public:

    mapGenerator(int rows, int cols, vector<int> blocks, int spacer, int vehicleNum);
    friend std::ostream& operator<<(std::ostream &out, const mapGenerator &m);
    vector<vector<float>> getStartPoints();
    vector<vector<float>> getEndPoints();
    Grid* getGrid();
    set<vector<vector<int>>> getStreets();
    set<vector<vector<int>>> getIntersections();
    vector<vector<int>> getMap(){ return this->mp; };
    ~mapGenerator(){delete this->grid;};
    
    
};


#endif