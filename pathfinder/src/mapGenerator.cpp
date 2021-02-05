#include "include/mapGenerator.hpp"

mapGenerator::mapGenerator(int rows, int cols, vector<int> blocks, int spacer, int vehicleNum)
{
    this->rows = rows;
    this->cols = cols;
    this->blocks = blocks;
    this->spacer = spacer;
    this->vehicleNum = vehicleNum;
    this->mp = vector<vector<int>>(rows+1, vector<int> (cols+1,0))  ;
    generateMap();
}

void mapGenerator::generateMap()
{
    int col_space = this->cols - (this->blocks[1]-1)*this->spacer;
    int row_space = this->rows - (this->blocks[0]-1)*this->spacer;
    this->blockDimensions.push_back(row_space/this->blocks[0]);
    this->blockDimensions.push_back(col_space/this->blocks[1]);
    
    setObsRanges();
    for(vector<int> obs : this->obstacles){
        this->mp[obs[0]][obs[1]] = 1;
    }
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
                this->obstacles.insert(vector<int>{xRange[0],y});
                this->obstacles.insert(vector<int>{xRange[1],y});
            }
        }
    }
    freeSpaceX.erase(vector<int>{ xRanges[xRanges.size()-1][1]+1, xRanges[xRanges.size()-1][1]+this->spacer-1});
    freeSpaceY.erase(vector<int>{ yRanges[yRanges.size()-1][1]+1, yRanges[yRanges.size()-1][1]+this->spacer-1});
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

            vector<vector<int>> intersection = { vector<int>{x[0], x[1]}, vector<int>{y[0], y[1]} };
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
    srand (time(NULL));
    for(int i = 0; i < vehicleNum; i++){
        vector<float> s;
        vector<float> e;

        auto itStart = this->streets.begin();
        auto itEnd = this->streets.begin();

        advance(itStart, rand()%this->streets.size());
        advance(itEnd, rand()%this->streets.size());

        vector<vector<int>> randomStreetStart = *itStart;
        vector<vector<int>> randomStreetEnd = *itEnd;

        s.push_back((rand()%(randomStreetStart[0][1]-1 - randomStreetStart[0][0]+1) + randomStreetStart[0][0]+1) );
        s.push_back(rand()%(randomStreetStart[1][1]-1 - randomStreetStart[1][0]+1) + randomStreetStart[1][0]+1);
        s.push_back(rand()%360);

        e.push_back(rand()%(randomStreetEnd[0][1]-1 - randomStreetEnd[0][0]+1) + randomStreetEnd[0][0]+1);
        e.push_back(rand()%(randomStreetEnd[1][1]-1 - randomStreetEnd[1][0]+1) + randomStreetEnd[1][0]+1);
        e.push_back(rand()%360);

        this->startPoints.push_back(s);
        this->endPoints.push_back(e);
    }
}


vector<vector<float>> mapGenerator::getStartPoints(){
    return this->startPoints;
}
vector<vector<float>> mapGenerator::getEndPoints(){
    return this->endPoints;
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
    out << "Start Points: " << "\n";
    for(vector<float> startPoint: m.startPoints){
        out << "(" << startPoint[0] << ',' << startPoint[1] << "),";
    }
    out << "\n";
    out << "End Points:" << "\n";
    for(vector<float> endPoint: m.endPoints){
        out << "(" << endPoint[0] << ',' << endPoint[1] << "),";
    }
    out << "\n";
    out << "Block Dimensions: " << m.blockDimensions[0] << " " << m.blockDimensions[1] << "\n";
    out << "obs coordinates: \n";
    for(vector<int> obstacle: m.obstacles){
        out << "(" << obstacle[0] << "," << obstacle[1] << "),";
    }
    out << "\n";
    out << "Streets: " << m.streets.size() << "\n";
    for(vector<vector<int>> street : m.streets){
        for(int x = street[0][0]; x < street[0][1]+1; x++){
            cout << "(" << x << "," << street[1][0] << "), ";
            cout << "(" << x << "," << street[1][1] << "), ";
        }
        for(int y = street[1][0]; y < street[1][1]+1; y++){
            cout << "(" << street[0][0] << "," << y << "), ";
            cout << "(" << street[0][1] << "," << y << "), ";
        }
    }
    out << "\n";
    out << "Intersections: " << m.intersections.size() << "\n";
    for(vector<vector<int>> intersection : m.intersections){
        for(int x = intersection[0][0]; x < intersection[0][1]+1; x++){
            cout << "(" << x << "," << intersection[1][0] << "),";
            cout << "(" << x << "," << intersection[1][1] << "),";
        }
        for(int y = intersection[1][0]; y < intersection[1][1]+1; y++){
            cout << "(" << intersection[0][0] << "," << y << "),";
            cout << "(" << intersection[0][1] << "," << y << "),";
        }
    }
    return out;
}