#include "include/mapGenerator.hpp"

mapGenerator::mapGenerator(int rows, int cols, vector<int> blocks, int spacer)
{
    this->rows = rows;
    this->cols = cols;
    this->blocks = blocks;
    this->spacer = spacer;
    generateMap();
}

void mapGenerator::generateMap()
{
    int col_space = this->cols - (this->blocks[1]-1)*this->spacer;
    int row_space = this->rows - (this->blocks[0]-1)*this->spacer;
    this->blockDimensions[1] = col_space/this->blocks[1];
    this->blockDimensions[0] = row_space/this->blocks[0];
    setObsRanges();
}

void mapGenerator::setObsRanges()
{
    vector<vector<int>> obsX;
    vector<vector<int>> obsY;
    for(int i = 0; i<this->blocks[1]; i++){
        int start = this->blockDimensions[1]*i + this->spacer*i;
        int end = start+this->blockDimensions[1];
        obsX.push_back(vector<int>{start,end});
    }
    for(int i = 0; i<this->blocks[0]; i++){
        int start = this->blockDimensions[0]*i + this->spacer*i;
        int end = start+this->blockDimensions[0];
        obsY.push_back(vector<int>{start,end});
    }
    setObsCoordinates(obsX,obsY);
}


void mapGenerator::setObsCoordinates(vector<vector<int>> xRanges,  vector<vector<int>> yRanges)
{
    set<vector<int>> freeSpaceX;
    set<vector<int>> freeSpaceY;
    for(vector<int> xRange : xRanges){
        freeSpaceX.insert(vector<int>{xRange[1]+1, xRange[1]+this->spacer-1});
        for(vector<int> yRange: yRanges){
            freeSpaceY.insert(vector<int>{yRange[1]+1, yRange[1]+this->spacer-1});
            for(int x = xRange[0]; x < xRange[1]+1; x++){
                this->obstacles.insert(vector<int>{x,yRange[0]});
                this->obstacles.insert(vector<int>{x,yRange[1]});
            }
            for(int y = yRange[0]; y < yRange[1]+1; y++){
                this->obstacles.insert(vector<int>{y,xRange[0]});
                this->obstacles.insert(vector<int>{y,xRange[1]});
            }
        }
    }
    freeSpaceX.erase(vector<int>{ xRanges[xRanges.size()-1][1]+1, xRanges[xRanges.size()-1][1]+this->spacer-1});
    freeSpaceX.erase(vector<int>{ yRanges[yRanges.size()-1][1]+1, yRanges[yRanges.size()-1][1]+this->spacer-1});
    setStreetsIntersections(freeSpaceX,freeSpaceY);
}


void mapGenerator::setStreetsIntersections(set<vector<int>> freeSpaceX,set<vector<int>> freeSpaceY)
{
    for(vector<int> x : freeSpaceX){
        for(vector<int> y : freeSpaceY){
            vector<int> bottom = {y[0]-this->blockDimensions[0]-1, y[0]-1};
            vector<int> top = {y[1]+1, y[1]+this->blockDimensions[0]+1};

            vector<int> left = {x[0]-this->blockDimensions[1]-1, x[0]-1};
            vector<int> right = {x[1]+1, x[1]+this->blockDimensions[1]+1};

            vector<vector<int>> intersection = { vector<int>{x[0]-1, x[1]+1}, vector<int>{y[0]-1, y[1]+1} };
            this->intersections.insert(intersection);
            this->streets.insert(vector<vector<int>>{x,bottom});
            this->streets.insert(vector<vector<int>>{x,top});
            this->streets.insert(vector<vector<int>>{left,y});
            this->streets.insert(vector<vector<int>>{right,y});
        }
    }
    setStartEndPoints();
}


void mapGenerator::setStartEndPoints()
{
    auto itStart = this->streets.cbegin();
    auto itEnd = this->streets.cbegin();
    advance(itStart, rand()%this->streets.size());
    advance(itEnd, rand()%this->streets.size());
    vector<vector<int>> randomStreetStart = *itStart;
    vector<vector<int>> randomStreetEnd = *itEnd;

    this->startPoint.push_back(rand()%(randomStreetStart[0][1] - randomStreetStart[0][0]) + randomStreetStart[0][0]);
    this->startPoint.push_back(rand()%(randomStreetStart[1][1] - randomStreetStart[1][0]) + randomStreetStart[1][0]);
    this->startPoint.push_back(rand()%360);

    this->endPoint.push_back(rand()%(randomStreetEnd[0][1] - randomStreetEnd[0][0]) + randomStreetEnd[0][0]);
    this->endPoint.push_back(rand()%(randomStreetEnd[1][1] - randomStreetEnd[1][0]) + randomStreetEnd[1][0]);
    this->endPoint.push_back(rand()%360);
}


vector<int> mapGenerator::getStartPoint(){
    return this->startPoint;
}
vector<int> mapGenerator::getEndPoint(){
    return this->endPoint;
}
set<vector<int>> mapGenerator::getObstacles(){
    return this->obstacles;
}
set<vector<vector<int>>> mapGenerator::getStreets(){
    return this->streets;
}
set<vector<vector<int>>> mapGenerator::getIntersections(){
    return this->intersections;
}

std::ostream& operator<<(std::ostream &out, const mapGenerator &m){
    out << "Start Point: X: " << m.startPoint[0] << " Y: " << m.startPoint[1] << " O: " << m.startPoint[2]<< "\n";
    out << "End Point: X: " << m.endPoint[0] << " Y: " << m.endPoint[1] << " O: " << m.endPoint[2] << "\n";
    out << "Block Dimensions" << m.blockDimensions[0] << " " << m.blockDimensions[1] << "\n";
    out << "obs coordinates: \n";
    for(vector<int> obstacle: m.obstacles){
        out << "\tx: " << obstacle[0] << " y: " << obstacle[1] << "\n";
    }
    return out;
}