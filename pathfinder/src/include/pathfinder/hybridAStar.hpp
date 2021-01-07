#ifndef HAS_H
#define HAS_H
#include "../vehicle.hpp"
#include "../grid.hpp"
#include "reedsShepps.hpp"
#include <queue>

struct HybridAStarNode
{
    VehicleState state;
    vector<ReedsSheppsAction> rsPath;
    HybridAStarNode* parent;
    float g;
    float h;
    HybridAStarNode();
    float FCost();
    //operator overloads
};

class HybridAStar
{
    ReedsSheepsCurves curves = ReedsSheepsCurves();
public:
    HybridAStarNode run(VehicleState start, VehicleState end, Grid *grid, Vehicle vehicle);
private:
    HybridAStarNode rsPath(VehicleState current, VehicleState goal);
    float calculateRSCost(vector<ReedsSheppsAction> path, float unit, float revCost, float gearCost);
    vector<HybridAStarNode> generateResult(HybridAStarNode destination);
    float calculateDistance(VehicleState current, VehicleState destination);
    vector<HybridAStarNode> getNextNode(VehicleState current, Gear gear, VehicleState goal);
    Coordinates4D stateToCell(VehicleState state);
    
};


#endif