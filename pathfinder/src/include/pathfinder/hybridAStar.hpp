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
    float f;
};

struct HAScomparator
{
    bool operator()(const HybridAStarNode* a, const HybridAStarNode* b)
    {
        return a->f > b->f;
    }
};

class HybridAStar
{
    ReedsSheppsCurves curves = ReedsSheppsCurves();
    Grid *grid;
public:
    HybridAStar(Grid* grid);
    vector<VehicleState> run(VehicleState start, VehicleState end, Vehicle vehicle);
private:
    HybridAStarNode* rsPath(VehicleState current, VehicleState goal);

    float calculateRSCost(Vehicle vehicle, vector<ReedsSheppsAction> path, float unit, float revCost, float gearCost);
    float calculateCost(VehicleState current, VehicleState next, float delta_time);
    vector<vector<float>> calculateHeuristic(VehicleState goal);

    vector<VehicleState> generateResult(HybridAStarNode destination);
    vector<HybridAStarNode*> getNextNode(VehicleState current, Gear gear, VehicleState goal, Vehicle vehicle);

    DiscreteCoordinates4D stateToCell(VehicleState state);

    bool areEquivalentStates(VehicleState comp, VehicleState other);
    
};


#endif