#ifndef MAPGENERATOR_H
#define MAPGENERATOR_H

#include "utils.hpp"



class mapGenerator
{
private:
    vector<int> startPoint;
    vector<int> endPoint;
    vector<int> blocks;
    vector<int> blockDimensions;
    set<vector<int>> obstacles;
    set<vector<vector<int>>> streets;
    set<vector<vector<int>>> intersections;
    int spacer;
    int rows;
    int cols;

    void generateMap();
    void setObsRanges();
    void setObsCoordinates(vector<vector<int>> xRanges,  vector<vector<int>> yRanges);
    void setStreetsIntersections(set<vector<int>> freeSpaceX,set<vector<int>> freeSpaceY);
    void setStartEndPoints();
public:

    mapGenerator(int rows, int cols, vector<int> blocks, int spacer);
    friend std::ostream& operator<<(std::ostream &out, const mapGenerator &m);
    vector<int> getStartPoint();
    vector<int> getEndPoint();
    set<vector<int>> getObstacles();
    set<vector<vector<int>>> getStreets();
    set<vector<vector<int>>> getIntersections();
    ~mapGenerator() = default;
    
    
};


#endif