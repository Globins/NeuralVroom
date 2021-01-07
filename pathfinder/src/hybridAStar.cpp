// #include "include/pathfinder/hybridAStar.hpp"

// HybridAStarNode HybridAStar::rsPath(VehicleState current, VehicleState goal)
// {
//     bool safe = true;
//     Coordinates3D start = {current.posX, current.posY, current.ori};
//     Coordinates3D end = {goal.posX, goal.posY, goal.ori};
//     vector<ReedsSheppsAction> path = curves.findOptimalPath(start, end);

//     // if(safe)
//     // {
//     //     return path;
//     // }
//     // vector<ReedsSheppsAction> empty;
//     // return empty;
// }
// float HybridAStar::calculateRSCost(vector<ReedsSheppsAction> path, float unit, float revCost, float gearCost)
// {
    
// }
// vector<HybridAStarNode> HybridAStar::generateResult(HybridAStarNode destination)
// {
    
// }
// float HybridAStar::calculateDistance(VehicleState current, VehicleState destination)
// {
    
// }
// vector<HybridAStarNode> HybridAStar::getNextNode(VehicleState current, Gear gear, VehicleState goal)
// {
//     vector<HybridAStarNode> nodes;
//     for(int i = 0; i < NUM_STEERS)
//     {
//         VehicleState new_state = VehicleState();
//         if(pointInGrid(new_state.posX, new_state.posY, grid->width, grid->height) && grid->isSafe(state, 1.5))
//         {
//             nodes.push_back(next_state);
//         }
//     }
//     //cost of RS
//     //if safe, add RS
//     return nodes;
// }
// Coordinates4D HybridAStar::stateToCell(VehicleState state)
// {
//     return Coordinates4D{(int)state.posX,(int)state.posY,mod2PI(state.ori),state.gear};
// }


// HybridAStarNode HybridAStar::run(VehicleState start, VehicleState end, Grid *grid, Vehicle vehicle)
// {
//     priority_queue<HybridAStarNode*> openList;
//     vector<HybridAStarNode*> closedList;
//     // while(openList.size())
//     // {
//     //     HybridAStarNode* current = openList.top();
//     //     closedList.push_back(current);
//     //     if(current->x == end.posX && current->y == end.posY)
//     //     {
//     //         return generateResult(*current);
//     //     }
//     //     for(int i = 0; i < 2; i++)
//     //     {

//     //     }
//     //     openList.pop();
//     // }
// }