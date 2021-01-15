#include "include/pathfinder/hybridAStar.hpp"

HybridAStar::HybridAStar(Grid* grid)
{
    this->grid = grid;  
}
HybridAStarNode HybridAStar::rsPath(VehicleState current, VehicleState goal)
{
    bool safe = true;
    HybridAStarNode rsNode;
    HybridAStarNode empty;
    rsNode.state = goal;
    
    Coordinates3D start = {current.posX, current.posY, current.ori};
    Coordinates3D end = {goal.posX, goal.posY, goal.ori};
    rsNode.rsPath = curves.findOptimalPath(start, end);
    vector<VehicleState> statePath = curves.discretizePath(current, rsNode.rsPath, 1, 1);
    for(VehicleState state : statePath)
    {
        if(!grid->isSafe(state, 1.5))
        {
            safe = false;
            break;
        }
    }
    if(safe)
    {
        return rsNode;
    }
    return empty;
}
float HybridAStar::calculateRSCost(Vehicle vehicle, vector<ReedsSheppsAction> path, float unit, float revCost, float gearCost)
{
    float length = 0;
    for(ReedsSheppsAction action : path)
    {
        if(action.steer != Straight)
        {
            length += vehicle.turnRadius * action.length;
        }
        else
        {
            length += action.length;
        }
    }
    if(revCost && !gearCost)
    {
        return length * unit;
    }
    else if(length == numeric_limits<float>::max() || !path.size())
    {
        return numeric_limits<float>::max();
    }

    float cost = 0;
    Gear prevGear = path.front().gear;
    for(ReedsSheppsAction action : path)
    {
        float actionCost = length * unit;
        if(action.gear == Backward)
        {
            actionCost *= revCost;
        }
        if(action.gear != prevGear)
        {
            actionCost += gearCost;
        }
        prevGear = action.gear;
        cost += actionCost;
    }
    return cost;
}
float HybridAStar::calculateCost(VehicleState current, VehicleState next, float delta_time)
{
    float distance = delta_time * 1;
    float revCost = 0;
    if(next.gear)
    {
        revCost = distance; // * revCost
    }
    if(current.gear != next.gear)
    {
        revCost = 1; // = switchCost
    }
    DiscreteCoordinates4D toCellLocation = stateToCell(next);
    return revCost + distance + distance*grid->cost[toCellLocation.x][toCellLocation.y]; //*voronoifieldfactor
}
float HybridAStar::calcuateHeuristic(VehicleState current, VehicleState goal)
{
    return 0;
}
vector<VehicleState> HybridAStar::generateResult(HybridAStarNode destination)
{
    vector<VehicleState> path;
    path.push_back(destination.state);
    HybridAStarNode* pathbefore = destination.parent;
    while(pathbefore != nullptr)
    {
        HybridAStarNode temp = *pathbefore;
        path.insert(path.begin(), temp.state);
        pathbefore = temp.parent;
    }
    return path;
}
vector<HybridAStarNode> HybridAStar::getNextNode(VehicleState current, Gear gear, VehicleState goal, Vehicle vehicle)
{
    vector<HybridAStarNode> nodes;
    for(Steer steer : {Straight, Left, Right})
    {
        HybridAStarNode nextNode;
        nextNode.state = vehicle.getNextState(current, steer, gear, 1);
        if(pointInGrid(nextNode.state.posX, nextNode.state.posY, grid->width, grid->height) && grid->isSafe(nextNode.state, 1.5))
        {
            nodes.push_back(nextNode);
        }
    }
    //cost of RS
    // int cost = 9;
    // if(cost < 10)
    // {
    //     HybridAStarNode rsNode = rsPath(current, goal);
    //     if(areEquivalentStates(rsNode.state, goal))
    //     {
    //         nodes.push_back(rsNode);
    //     }
    // }
    return nodes;
}
DiscreteCoordinates4D HybridAStar::stateToCell(VehicleState state)
{
    return DiscreteCoordinates4D{(int)state.posX,(int)state.posY,mod2PI(state.ori),state.gear};
}


vector<VehicleState> HybridAStar::run(VehicleState start, VehicleState end, Vehicle vehicle)
{
    vector<vector<float>> heuristicMap;
    vector<vector<float>> costMap;
    costMap.resize(grid->width,vector<float>(grid->height, numeric_limits<float>::max()));
    HybridAStarNode startNode = HybridAStarNode{start};
    priority_queue<HybridAStarNode, vector<HybridAStarNode>, HAScomparator> openList;
    vector<HybridAStarNode> closedList;
    openList.push(startNode);
    while(openList.size())
    {
        HybridAStarNode current = openList.top();
        openList.pop();
        closedList.push_back(current);
        if(areEquivalentStates(current.state, end))
        {
            return generateResult(current);
        }
        for(Gear gear : {Forward, Backward})
        {
            vector<HybridAStarNode> children = getNextNode(current.state, gear, end, vehicle);
            for(HybridAStarNode child : children)
            {
                child.parent = &current;
                if(child.rsPath.size())
                {
                    child.g += current.g + calculateRSCost(vehicle, child.rsPath, 1, 1, 1);
                }
                else
                {
                    child.g += current.g + calculateCost(current.state, child.state, 1);
                }
                
                child.h += calcuateHeuristic(child.state, end);
                child.f = child.h + child.g;
                
                DiscreteCoordinates4D location = stateToCell(child.state);
                if(child.f < costMap[location.x][location.y])
                {
                    costMap[location.x][location.y] = child.f;
                    openList.push(child);
                }
            }
        }
    }
    cout << "NO PATH FOUND" << endl;
    return generateResult(startNode);
}

bool HybridAStar::areEquivalentStates(VehicleState comp, VehicleState other)
{
    return comp.posX == other.posX && comp.posY ==  other.posY && comp.ori == other.ori && comp.gear == other.gear;
}