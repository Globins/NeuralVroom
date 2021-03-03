#include "include/pathrouter/geneticAlgorithm.hpp"

Genotype::Genotype(vector<float> params)
{
    this->params = params;
    fitness = 0;
}
void Genotype::setRandomParams(float minVal, float maxVal)
{
    float range = maxVal - minVal;
    for(int i = 0; i < params.size(); i++)
    {
        params[i] = (((double) rand() / (RAND_MAX)) *range + minVal);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
Agent::Agent(Genotype genotype, vector<unsigned> topology, VehicleState state, vector<vector<float>> costMap)
{
    vehicleState = state;
    nn = NeuralNet(topology);
    nn.GenotypeParamsToWeights(genotype.params);
    this->costMap = costMap;
}
void Agent::update(vector<vector<int>> m)
{
    cout << "hi" << endl;
    vector<double> sensorInfo = vehicleStatus.getDistanceFromObstacles(m, vehicleState);
    for(double info : sensorInfo)
    {
        cout << info << " ";
    }
    cout << endl;
    nn.feedForward(sensorInfo);
    vector<double> results;
    nn.getResults(results);
    vehicleState = nn.processResults(&vehicleStatus, vehicleState, results);
}
//----------------------------------------------------------------------------------------------------------------------------------
GeneticAlgorithm::GeneticAlgorithm(int genotypeParamCount, int populationSize, const vector<unsigned> &topology, mapGenerator &m)
{
    this->populationSize = populationSize;
    this->topology = topology;
    this->mapGenPtr = &m;
    currentPopulation.resize(populationSize, Genotype(vector<float>(genotypeParamCount, 0)));
    GenerationCount = 1;
    sortPopulation = true;
}
void GeneticAlgorithm::start(const int trainAmount)
{
    this->trainAmount = trainAmount;
    initPopulation();
    evaluation();
}

void GeneticAlgorithm::initPopulation()
{
    for(Genotype &geo : currentPopulation)
    {
        geo.setRandomParams(defaultInitParamMin, defaultInitParamMax);
    }
}

void GeneticAlgorithm::evaluation()
{
    agents.clear();
    vector<vector<float>> start = mapGenPtr->getStartPoints();
    vector<vector<float>> end = mapGenPtr->getEndPoints();
    Grid *grid = mapGenPtr->getGrid();
    
    for(int i = 0; i < currentPopulation.size(); i++)
    {
        VehicleState state = VehicleState{start[i][0], start[i][1], start[i][2], Forward, Straight};
        vector<vector<float>> costMap = grid->nonHolonomicRelaxedCostMap(VehicleState{end[i][0], end[i][1], end[i][2], Forward, Straight});
        agents.push_back(Agent(currentPopulation[i], topology, state, costMap));
    }
    int vehiclesCrashed = 0;
    //while(vehiclesCrashed < currentPopulation.size())
    //{
        for(int i = 0; i < agents.size(); i++)
        {
            if(agents[i].hasCrashed)
            {
                cout << "AGENT " << i << ": CRASHED" << endl;
                continue;
            }
            else if(!grid->isSafe(agents[i].vehicleState, 1.5))
            {
                agents[i].hasCrashed = true;
                cout << "AGENT " << i << ": CRASHED" << endl;
                continue;
            }
            agents[i].update(mapGenPtr->getMap());
            float startDist = agents[i].costMap[start[i][0]][start[i][1]];
            float currentDist = agents[i].costMap[agents[i].vehicleState.posX][agents[i].vehicleState.posY];
            cout << startDist << " " << currentDist << endl;
            currentPopulation[i].eval = (startDist - currentDist) / startDist;
            cout << "AGENT " << i << ": " << agents[i].vehicleState.posX << ", " << agents[i].vehicleState.posY << ", " << currentPopulation[i].eval << endl;
        }
        cout << endl;
    //}
    evaluationFinished();
}
void GeneticAlgorithm::evaluationFinished()
{
    fitnessCalculation();
    if(sortPopulation)
    {

    }
    // if(GenerationCount < trainAmount)
    // {
    //     vector<Genotype> intermediatePop = selection();
    //     vector<Genotype> newPopulation = recombination(intermediatePop, populationSize);
    //     mutation(newPopulation);
    //     currentPopulation = newPopulation;
    //     GenerationCount++;
    //     evaluation();
    // }
}
void GeneticAlgorithm::fitnessCalculation()
{
    int populationSize = 0;
    float overallEval = 0;
    for(Genotype &geno : currentPopulation)
    {
        overallEval += geno.eval;
        populationSize++;
    }
    float averageEval = overallEval / populationSize;
    for(Genotype &geno : currentPopulation)
    {
        geno.fitness = overallEval / populationSize;
    }
}
vector<Genotype> GeneticAlgorithm::selection()
{
    vector<Genotype> intermediatePopulation;
    for(Genotype &geno : currentPopulation)
    {
        if(geno.fitness < 1)
        {
            break;
        }
        else
        {
            for(int i = 0; i < (int)geno.fitness; i++)
            {
                intermediatePopulation.push_back(Genotype(geno.params));
            }
        }
    }
    for(Genotype &geno : currentPopulation)
    {
        float remainder = geno.fitness - (int)geno.fitness;
        if(((double) rand() / (RAND_MAX)) < remainder)
        {
            intermediatePopulation.push_back(Genotype(geno.params));
        }
    }

    return intermediatePopulation;
}
vector<Genotype> GeneticAlgorithm::recombination(vector<Genotype> intermediatePop, int newPopSize)
{
    if(intermediatePop.size() < 2)
    {
        throw new invalid_argument("Intermediate population too small");
    }
    vector<Genotype> newPopulation;
    newPopulation.push_back(intermediatePop[0]);
    newPopulation.push_back(intermediatePop[1]);
    while(newPopulation.size() < newPopSize)
    {
        int rand1 = rand() % newPopSize;
        int rand2 = 0;
        do
        {
            rand2 = rand() % newPopSize;
        } while (rand1 == rand2);
        
        vector<Genotype> offspringOneAndTwo = completeCrossover(intermediatePop[rand1], intermediatePop[rand2], defaultCrossSwapProb);
        newPopulation.push_back(offspringOneAndTwo[0]);
        if(newPopulation.size() < newPopSize)
        {
            newPopulation.push_back(offspringOneAndTwo[1]);
        }
    }
    return newPopulation;
}
void GeneticAlgorithm::mutation(vector<Genotype> &newPop)
{
    for(int i = 2; i < newPop.size(); i++)
    {
        if(((double) rand() / (RAND_MAX)) < defaultMutationPerc)
        {
            mutateGenotype(newPop[i], defaultMutationProb, defaultMutationAmount);
        }
    }
}
vector<Genotype> GeneticAlgorithm::completeCrossover(const Genotype &parent1, const Genotype &parent2, float swapChance)
{
    int paramCount = parent1.params.size();
    vector<float> off1Params(paramCount);
    vector<float> off2Params(paramCount);
    for(int i = 0; i < paramCount; i++)
    {
        if(((double) rand() / (RAND_MAX)) < swapChance)
        {
            off1Params[i] = parent2.params[i];
            off2Params[i] = parent1.params[i];
        }
        else
        {
            off1Params[i] = parent1.params[i];
            off2Params[i] = parent2.params[i];
        }
    }
    return vector<Genotype>{Genotype(off1Params), Genotype(off2Params)};
}
void GeneticAlgorithm::mutateGenotype(Genotype &genotype, float mutationProb, float mutationAmount)
{
    for(float param : genotype.params)
    {
        if(((double) rand() / (RAND_MAX)) < mutationProb*100)
        {
            param += float(((double) rand() / (RAND_MAX)) * mutationAmount* 2 - mutationAmount);
        }
    }
}
vector<Genotype> GeneticAlgorithm::getPopulation()
{
    return currentPopulation;
}
void GeneticAlgorithm::printPopulation()
{
    for(Genotype &geo : currentPopulation)
    {
        for(float param : geo.params)
        {
            cout << param << " ";
        }
        cout << endl;
    }
}