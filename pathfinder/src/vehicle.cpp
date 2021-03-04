#include "include/vehicle.hpp"

Vehicle::Vehicle(float width, float length)
{
    this->width = width;
    this->length = length;
}
VehicleState Vehicle::getNextState(VehicleState current, Steer steer, Gear gear, float delta_time)
{
    float length = velocity*delta_time;
    float xPos = 0;
    float yPos = 0;
    float angle = 0;
    if(steer == Straight)
    {
        xPos = length;
    }
    else
    {
        angle = length / turnRadius;
        float sinAngle = sin(angle/2);
        float l = 2 * sinAngle * turnRadius;
        xPos = l* cos(turnRadius);
        yPos = l * sinAngle;
    }
    if(steer == Right)
    {
        yPos *= -1;
        angle *= -1;
    }
    if(gear == Backward)
    {
        xPos *= -1;
        angle *= -1;
    }
    Coordinates2D correctedPos = rotate(xPos, yPos, current.ori);
    current.posX += correctedPos.x;
    current.posY += correctedPos.y;
    current.ori += angle;
    current.steer = steer;
    current.gear = gear;
    return current;
}

vector<Coordinates3D> Vehicle::getSurroundingCoords(Coordinates3D currentPos, float dist){
    float currentOrientation = currentPos.radians;
    vector<Coordinates3D> surroundingCoords;
    for(int i = 0; i < 24; i++){
        if(currentOrientation < M_PI_2){
            float y = sin(currentOrientation)*dist;
            float x = cos(currentOrientation)*dist;
            surroundingCoords.push_back(Coordinates3D{currentPos.x + x, currentPos.y - y, currentOrientation});
        }
        else if(currentOrientation >= M_PI_2 && currentOrientation < M_PI){
            float tempOrientation = currentOrientation - M_PI_2;
            float y = cos(tempOrientation)*dist;
            float x = sin(tempOrientation)*dist;
            surroundingCoords.push_back(Coordinates3D{currentPos.x - x, currentPos.y-y, currentOrientation});
        }
        else if(currentOrientation >= M_PI && currentOrientation < 1.5*M_PI){
            float tempOrientation = currentOrientation - M_PI;
            float y = sin(tempOrientation)*dist;
            float x = cos(tempOrientation)*dist;
            surroundingCoords.push_back(Coordinates3D{currentPos.x - x, currentPos.y + y, currentOrientation});
        }
        else if(currentOrientation >= 1.5*M_PI && currentOrientation < 2*M_PI){
            float tempOrientation = currentOrientation - 1.5*M_PI;
            float y = cos(tempOrientation)*dist;
            float x = sin(tempOrientation)*dist;
            surroundingCoords.push_back(Coordinates3D{currentPos.x + x, currentPos.y + y, currentOrientation});
        }
        currentOrientation += M_PI/12;
        if (currentOrientation >= 2*M_PI)
            currentOrientation -= 2*M_PI;
    }
    return surroundingCoords;
}

vector<vector<float>> Vehicle::getSlopes(Coordinates3D start, vector<Coordinates3D> surroundingCoords){
    vector<vector<float>> slopes;
    for(Coordinates3D coord : surroundingCoords){
        float rise = abs(coord.y - start.y);
        float run = abs(coord.x - start.x);
        slopes.push_back(vector<float>{abs(rise), abs(run)});        
    }
    return slopes;
}

vector<double> Vehicle::getDistanceFromObstacles(vector<vector<int>> m, VehicleState currentState){
    float dist = 3.1;
    Coordinates3D currentPos = Coordinates3D{currentState.posX, currentState.posY, currentState.ori};
    vector<Coordinates3D> surroundingCoords = getSurroundingCoords(currentPos, dist);
    vector<vector<float>> slopes = getSlopes(currentPos, surroundingCoords);
    vector<double> distances;
    for(int i = 0; i < slopes.size(); i++){
        float tempX = currentPos.x;
        float tempY = currentPos.y;
        for(int j = 0; j < dist*2; j++){
            if(m[roundf(tempY)][roundf(tempX)] == 1){
                distances.push_back( (dist*2 - j)/(dist*2)  );
                break;
            }
            if(surroundingCoords[i].radians > 0 && surroundingCoords[i].radians < M_PI ){
                tempY -= slopes[i][0]/(dist*2);
            }
            else{
                tempY += slopes[i][0]/(dist*2);
            }
            if(surroundingCoords[i].radians > M_PI_2 && surroundingCoords[i].radians < 1.5*M_PI ){
                tempX -= slopes[i][1]/(dist*2);
            }
            else{
                tempX += slopes[i][1]/(dist*2);
            }
            //cout << tempX << " " << tempY << endl;
            if(tempX >= 55 || tempY >= 55 || tempX < 0 || tempY < 0){
                cout << "hi " << endl;
                distances.push_back(1);
                break;
            }
        }
        //cout << endl;
        if(distances.size() != i+1){
            distances.push_back(0);
            
        }
    }
    for(double dist : distances)
    {
        cout << dist << " ";
    }
    cout << endl;
    return distances;
}

bool Vehicle::areEquivalentStates(VehicleState comp, VehicleState other)
{
    return comp.posX == other.posX && comp.posY ==  other.posY && comp.ori == other.ori && comp.gear == other.gear;
}