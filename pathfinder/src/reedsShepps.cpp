#include "include/pathfinder/reedsShepps.hpp"


ReedsSheppsAction::ReedsSheppsAction(Steer steer, Gear gear, float length)
{
    this->steer = steer;
    this->gear = gear;
    this->length = length;
}
void ReedsSheppsAction::revSteer()
{
    if(steer == Left)
    {
        steer = Right;
    }
    else if(steer == Right)
    {
        steer = Left;
    }
}
void ReedsSheppsAction::revGear()
{
    if(gear == Backward)
    {
        gear = Forward;
    }
    else
    {
        gear = Backward;
    }
}


float ReedsSheppsCurves::calculatePathLength(vector<ReedsSheppsAction> path)
{
    float pathlength = 0;
    for(ReedsSheppsAction rsa : path)
    {
        pathlength += rsa.length;
    }
    return pathlength;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::findOptimalPath(Coordinates3D start, Coordinates3D end)
{
    vector<vector<ReedsSheppsAction>> allPaths = findAllPaths(start, end);
    int index = 0;
    float smallestLength = calculatePathLength(allPaths[0]);
    for(vector<vector<ReedsSheppsAction>>::size_type i = 1; i < allPaths.size(); i++) {
        float length = calculatePathLength(allPaths[i]);
        if(length <= smallestLength)
        {
            index = i;
            smallestLength = length;
        }
    }
    return allPaths[index];
}
vector<vector<ReedsSheppsAction>> ReedsSheppsCurves::findAllPaths(Coordinates3D start, Coordinates3D end)
{
    Coordinates3D calculateBasedOffThis = changeOfBasis(start, end);
    float x = calculateBasedOffThis.x;
    float y = calculateBasedOffThis.y;
    float radians = calculateBasedOffThis.radians;
    vector<vector<ReedsSheppsAction>> paths;
    for(int i = 0; i < 12; i++) //Total 12 different paths, each with 4 variations
    {
        paths.push_back((this->*pathFunctions[i])(x,y,radians));
        vector<ReedsSheppsAction> revG = (this->*pathFunctions[i])(-x,y,-radians);
        for(int i = 0; i < revG.size(); i++)
        {
            revG[i].revGear();
        }
        paths.push_back(revG);
        vector<ReedsSheppsAction> revS = (this->*pathFunctions[i])(x,-y,-radians);
        for(int i = 0; i < revS.size(); i++)
        {
            revS[i].revSteer();
        }
        paths.push_back(revS);
        vector<ReedsSheppsAction> revSG = (this->*pathFunctions[i])(-x,-y,radians);
        for(int i = 0; i < revSG.size(); i++)
        {
            revSG[i].revGear();
            revSG[i].revSteer();
        }
        paths.push_back(revSG);
    }
    vector<vector<ReedsSheppsAction>> filteredpaths;
    for(auto path : paths)
    {
        if(calculatePathLength(path))
        {
            filteredpaths.push_back(path);
        }
    }
    return filteredpaths;
}

vector<VehicleState> ReedsSheppsCurves::discretizePath(VehicleState current, vector<ReedsSheppsAction> rsPath, float unit, float length)
{
    VehicleState state = current;
    vector<VehicleState> states;
    states.push_back(state);
    //length is both arc angle and length
    for(ReedsSheppsAction action : rsPath)
    {
        int steps = (int)(ceil(action.length * unit / length));
        if(action.steer != Straight)
        {
            float angle = action.length / steps;
            float dx = unit*sin(angle);
            float dy = unit-unit*cos(angle);
            if(action.steer == Right)
            {
                dy *= -1;
                angle *= -1;
            }
            if(action.gear == Backward)
            {
                dx *= -1;
                angle *= -1;
            }
            for(int i = 0; i < steps; i++)
            {
                Coordinates2D rotatedPoints = rotate(dx, dy, current.ori);
                current = VehicleState{rotatedPoints.x + current.posX, rotatedPoints.y + current.posY, current.ori + angle, action.gear};
                states.push_back(current);
            }
        }
        else
        {
            float newLength = action.length * unit / steps;
            float dx = newLength * cos(current.ori);
            float dy = newLength * sin(current.ori);

            if(action.gear == Backward)
            {
                dx = -dx;
                dy = -dy;
            }
            for(int i = 0; i < steps; i++)
            {
                current = VehicleState{dx + current.posX, dy + current.posY, current.ori, action.gear};
                states.push_back(current);
            }
        }
    }
    return states;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::one_CSC_SameTurns(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    PolarCoordinates rAndTheta = cart2pol(x - sin(radians), y - 1 + cos(radians)); //U and T
    
    float v = mod2PI(radians - rAndTheta.theta);
    if(rAndTheta.theta >= 0 &&  rAndTheta.radius >= 0 && v >= 0)
    {
        segment.push_back(ReedsSheppsAction(Left, Forward, rAndTheta.theta));
        segment.push_back(ReedsSheppsAction(Straight, Forward, rAndTheta.radius));
        segment.push_back(ReedsSheppsAction(Left, Forward, v));
    }
    return segment;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::two_CSC_DiffTurns(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    PolarCoordinates rAndTheta = cart2pol(x + sin(radians), y - 1 - cos(radians));
    if(rAndTheta.radius*rAndTheta.radius >= 4)
    {
        float u = sqrt(rAndTheta.radius*rAndTheta.radius -4);
        float t = mod2PI(rAndTheta.theta + atan2(2, u));
        float v = mod2PI(t - radians);
        if(t >= 0 && u >= 0 && v >= 0)
        {
            segment.push_back(ReedsSheppsAction(Left, Forward, t));
            segment.push_back(ReedsSheppsAction(Straight, Forward, u));
            segment.push_back(ReedsSheppsAction(Right, Forward, v));
        }
    }
    return segment;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::three_CCC(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    PolarCoordinates rAndTheta = cart2pol(x - sin(radians), y - 1 + cos(radians));
    if(rAndTheta.radius <= 4)
    {
        float a = acos(rAndTheta.radius/4);
        float t = mod2PI(rAndTheta.theta + M_PI/2 + a);
        float u = mod2PI(M_PI - 2*a);
        float v = mod2PI(radians - t - u);
        if(t >= 0 && u >= 0 && v >= 0)
        {
            segment.push_back(ReedsSheppsAction(Left, Forward, t));
            segment.push_back(ReedsSheppsAction(Right, Backward, u));
            segment.push_back(ReedsSheppsAction(Left, Forward, v));
        }
    }
    return segment;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::four_C_CC(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    PolarCoordinates rAndTheta = cart2pol(x - sin(radians), y - 1 + cos(radians));
    if(rAndTheta.radius <= 4)
    {
        float a = acos(rAndTheta.radius/4);
        float t = mod2PI(rAndTheta.theta + M_PI/2 + a);
        float u = mod2PI(M_PI - 2*a);
        float v = mod2PI(t + u - radians);
        if(t >= 0 && u >= 0 && v >= 0)
        {
            segment.push_back(ReedsSheppsAction(Left, Forward, t));
            segment.push_back(ReedsSheppsAction(Right, Backward, u));
            segment.push_back(ReedsSheppsAction(Left, Backward, v));
        }
    }
    return segment;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::five_CC_C(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    PolarCoordinates rAndTheta = cart2pol(x - sin(radians), y - 1 + cos(radians));
    if(rAndTheta.radius <= 4)
    {
        float u = acos(1 - rAndTheta.radius*rAndTheta.radius/8);
        float a = asin(2*sin(u)/rAndTheta.radius);
        float t = mod2PI(rAndTheta.theta + M_PI/2 - a);
        float v = mod2PI(t - u - radians);
        if(t >= 0 && u >= 0 && v >= 0)
        {
            segment.push_back(ReedsSheppsAction(Left, Forward, t));
            segment.push_back(ReedsSheppsAction(Right, Forward, u));
            segment.push_back(ReedsSheppsAction(Left, Backward, v));
        }
    }
    return segment;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::six_CCu_CuC(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    PolarCoordinates rAndTheta = cart2pol(x + sin(radians), y - 1 - cos(radians));
    if(rAndTheta.radius <= 4)
    {
        float a, t, u, v = 0;
        if(rAndTheta.radius <= 2)
        {
            a = acos((rAndTheta.radius+2)/4);
            t = mod2PI(rAndTheta.theta + M_PI/2 + a);
            u = mod2PI(a);
            v = mod2PI(radians - t + 2*u);
        }
        else
        {
            a = acos((rAndTheta.radius-2)/4);
            t = mod2PI(rAndTheta.theta + M_PI/2 - a);
            u = mod2PI(M_PI - a);
            v = mod2PI(radians - t + 2*u);
        }
        if(t >= 0 && u >= 0 && v >= 0)
        {
            segment.push_back(ReedsSheppsAction(Left, Forward, t));
            segment.push_back(ReedsSheppsAction(Right, Forward, u));
            segment.push_back(ReedsSheppsAction(Left, Backward, u));
            segment.push_back(ReedsSheppsAction(Right, Backward, v));
        }
    }
    return segment;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::seven_C_CuCu_C(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    PolarCoordinates rAndTheta = cart2pol(x + sin(radians), y - 1 - cos(radians));
    float u1 = (20 - rAndTheta.radius*rAndTheta.radius) / 16;
    if(rAndTheta.radius <= 6 && u1 >= 0 && u1 <= 1)
    {
        float u = acos(u1);
        float a = asin(2*sin(u)/rAndTheta.radius);
        float t = mod2PI(rAndTheta.theta + M_PI/2 + a);
        float v = mod2PI(t - radians);
        if(t >= 0 && u >= 0 && v >= 0)
        {
            segment.push_back(ReedsSheppsAction(Left, Forward, t));
            segment.push_back(ReedsSheppsAction(Right, Backward, u));
            segment.push_back(ReedsSheppsAction(Left, Backward, u));
            segment.push_back(ReedsSheppsAction(Right, Forward, v));
        }
    }
    return segment;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::eight_C_Cpi2SC_SameTurn(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    PolarCoordinates rAndTheta = cart2pol(x - sin(radians), y - 1 + cos(radians));
    if(rAndTheta.radius >= 2)
    {
        float u = sqrt(rAndTheta.radius*rAndTheta.radius - 4) - 2;
        float a = atan2(2, u+2);
        float t = mod2PI(rAndTheta.theta + M_PI/2 + a);
        float v = mod2PI(t - radians + M_PI/2);
        if(t >= 0 && u >= 0 && v >= 0)
        {
            segment.push_back(ReedsSheppsAction(Left, Forward, t));
            segment.push_back(ReedsSheppsAction(Right, Backward, M_PI/2));
            segment.push_back(ReedsSheppsAction(Straight, Backward, u));
            segment.push_back(ReedsSheppsAction(Left, Backward, v));
        }
    } 
    return segment;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::nine_C_Cpi2SC_DiffTurn(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    PolarCoordinates rAndTheta = cart2pol(x + sin(radians), y - 1 - cos(radians));
    if(rAndTheta.radius >= 2)
    {
        float t = mod2PI(rAndTheta.theta + M_PI/2);
        float u = rAndTheta.radius - 2;
        float v = mod2PI(radians - t - M_PI/2);
        if(t >= 0 && u >= 0 && v >= 0)
        {
            segment.push_back(ReedsSheppsAction(Left, Forward, t));
            segment.push_back(ReedsSheppsAction(Right, Backward, M_PI/2));
            segment.push_back(ReedsSheppsAction(Straight, Backward, u));
            segment.push_back(ReedsSheppsAction(Right, Backward, v));
        }
    } 
    return segment;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::ten_CSCp2_C_SameTurn(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    PolarCoordinates rAndTheta = cart2pol(x - sin(radians), y - 1 + cos(radians));
    if(rAndTheta.radius >= 2)
    {
        float u = sqrt(rAndTheta.radius*rAndTheta.radius - 4) - 2;
        float a = atan2(u+2, 2);
        float t = mod2PI(rAndTheta.theta + M_PI/2 - a);
        float v = mod2PI(t - radians - M_PI/2);
        if(t >= 0 && u >= 0 && v >= 0)
        {
            segment.push_back(ReedsSheppsAction(Left, Forward, t));
            segment.push_back(ReedsSheppsAction(Straight, Forward, u));
            segment.push_back(ReedsSheppsAction(Right, Forward, M_PI/2));
            segment.push_back(ReedsSheppsAction(Left, Backward, v));
        }
    } 
    return segment;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::eleven_CSCp2_C_DiffTurn(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    PolarCoordinates rAndTheta = cart2pol(x + sin(radians), y - 1 - cos(radians));
    if(rAndTheta.radius >= 2)
    {
        float t = mod2PI(rAndTheta.theta);
        float u = rAndTheta.radius - 2;
        float v = mod2PI(radians - t - M_PI/2);
        if(t >= 0 && u >= 0 && v >= 0)
        {
            segment.push_back(ReedsSheppsAction(Left, Forward, t));
            segment.push_back(ReedsSheppsAction(Straight, Forward, u));
            segment.push_back(ReedsSheppsAction(Left, Forward, M_PI/2));
            segment.push_back(ReedsSheppsAction(Right, Backward, v));
        }
    }  
    return segment;
}
vector<ReedsSheppsAction> ReedsSheppsCurves::twelve_C_Cpi2SCpi2_C(float x, float y, float radians)
{
    vector<ReedsSheppsAction> segment;
    
    PolarCoordinates rAndTheta = cart2pol(x + sin(radians), y - 1 - cos(radians));
    if(rAndTheta.radius >= 4)
    {
        float u = sqrt(rAndTheta.radius*rAndTheta.radius - 4) -4;
        float a = atan2(2, u+4);
        float t = mod2PI(rAndTheta.theta + M_PI/2 + a);
        float v = mod2PI(t - radians);
        if(t >= 0 && u >= 0 && v >= 0)
        {
            segment.push_back(ReedsSheppsAction(Left, Forward, t));
            segment.push_back(ReedsSheppsAction(Right, Backward, M_PI/2));
            segment.push_back(ReedsSheppsAction(Straight, Backward, u));
            segment.push_back(ReedsSheppsAction(Left, Backward, M_PI/2));
            segment.push_back(ReedsSheppsAction(Right, Forward, v));
        }
    }  
    return segment;
}