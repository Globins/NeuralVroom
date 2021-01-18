#include "include/pathfinder/hybridAStar.hpp"

HybridAStar::HybridAStar(Grid* grid)
{
    this->grid = grid;  
}
HybridAStarNode* HybridAStar::rsPath(VehicleState current, VehicleState goal)
{
    bool safe = true;
    HybridAStarNode* rsNode = new HybridAStarNode();
    rsNode->state = goal;
    
    Coordinates3D start = {current.posX, current.posY, current.ori};
    Coordinates3D end = {goal.posX, goal.posY, goal.ori};
    rsNode->rsPath = curves.findOptimalPath(start, end);
    vector<VehicleState> statePath = curves.discretizePath(current, rsNode->rsPath, 1, 1);
    for(VehicleState state : statePath)
    {
        if(!grid->isSafe(state, 1.5))
        {
            safe = false;
            break;
        }
    }
    cout << rsNode->rsPath.size() << endl;
    if(safe)
    {
        return rsNode;
    }
    return nullptr;
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

vector<VehicleState> HybridAStar::generateResult(HybridAStarNode destination)
{
    vector<VehicleState> path;
    path.push_back(destination.state);
    HybridAStarNode* pathbefore = destination.parent;
    if(!destination.rsPath.empty())
    {
        cout << "Honk" << endl;
        vector<VehicleState> statePath = curves.discretizePath(pathbefore->state, destination.rsPath, 1, 1);
        for(int i = statePath.size()-1; i > 0; i--)
        {
            path.push_back(statePath[i]);
        }
    }
    while(pathbefore != nullptr)
    {
        HybridAStarNode temp = *pathbefore;
        path.insert(path.begin(), temp.state);
        pathbefore = temp.parent;
    }
    return path;
}
vector<HybridAStarNode*> HybridAStar::getNextNode(VehicleState current, Gear gear, VehicleState goal, Vehicle vehicle)
{
    vector<HybridAStarNode*> nodes;
    for(Steer steer : {Straight, Left, Right})
    {
        HybridAStarNode* nextNode = new HybridAStarNode();
        nextNode->state = vehicle.getNextState(current, steer, gear, 1);
        if(pointInGrid(nextNode->state.posX, nextNode->state.posY, grid->width, grid->height) && grid->isSafe(nextNode->state, 1.5))
        {
            nodes.push_back(nextNode);
        }
    }
    // cost of RS
    int cost = 9;
    if(cost < 10)
    {
        HybridAStarNode* rsNode = rsPath(current, goal);
        if(rsNode != nullptr)
        {
            nodes.push_back(rsNode);
        }
    }
    return nodes;
}
DiscreteCoordinates4D HybridAStar::stateToCell(VehicleState state)
{
    return DiscreteCoordinates4D{(int)state.posX,(int)state.posY,mod2PI(state.ori),state.gear};
}


vector<VehicleState> HybridAStar::run(VehicleState start, VehicleState end, Vehicle vehicle)
{
    vector<vector<float>> heuristicMap = grid->nonHolonomicRelaxedCostMap(end);
    vector<vector<float>> costMap;
    vector<VehicleState> path;
    costMap.resize(grid->width,vector<float>(grid->height, numeric_limits<float>::max()));

    HybridAStarNode* startNode = new HybridAStarNode{start};
    priority_queue<HybridAStarNode*, vector<HybridAStarNode*>, HAScomparator> openList;
    vector<HybridAStarNode*> closedList;

    openList.push(startNode);
    while(!openList.empty())
    {
        // for (auto row = costMap.begin(); row != costMap.end(); row++) {
        //     vector<float> pushRow;
        //     for (auto col = row->begin(); col != row->end(); col++) 
        //     {
        //         if(*col == numeric_limits<float>::max())
        //         {
        //             cout << -1 << " ";
        //         }
        //         else
        //         {
        //             cout << *col << " ";
        //         }
        //     }
        //     cout << endl;
        // }
        // cout << endl;
        HybridAStarNode* current = openList.top();
        openList.pop();
        closedList.push_back(current);
        if(areEquivalentStates(current->state, end))
        {
            path = generateResult(*current);
            break;
        }

        for(Gear gear : {Forward, Backward})
        {
            vector<HybridAStarNode*> children = getNextNode(current->state, gear, end, vehicle);
            for(HybridAStarNode* child : children)
            {
                child->parent = closedList.back();
                if(!child->rsPath.empty())
                {
                    child->g += current->g + calculateRSCost(vehicle, child->rsPath, 1, 1, 1);
                    cout << "HONK" << endl;
                }
                else
                {
                    child->g += current->g + calculateCost(current->state, child->state, 1);
                }
                
                child->h += heuristicMap[(int)child->state.posX][(int)child->state.posY];
                child->f = child->h + child->g;
                DiscreteCoordinates4D location = stateToCell(child->state);
                if(child->f < costMap[location.x][location.y])
                {
                    costMap[location.x][location.y] = child->f;
                    openList.push(child);
                }
            }
        }
    }
    while(openList.size())
    {
        HybridAStarNode* current = openList.top();
        openList.pop();
        delete current;
    }
    for(auto item : closedList)
    {
        delete item;
    }
    if(path.empty())
    {
        cout << "NO PATH FOUND" << endl;
    }
    return path;
}

bool HybridAStar::areEquivalentStates(VehicleState comp, VehicleState other)
{
    return comp.posX == other.posX && comp.posY ==  other.posY && comp.ori == other.ori && comp.gear == other.gear;
}