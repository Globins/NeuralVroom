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
Agent::Agent(Genotype genotype, vector<float> topology, VehicleState state)
{
    vehicleState = state;
    nn = NeuralNet(topology);
    vehicleStatus = Vehicle(10, 16.5);
}
void Agent::update()
{
    vector<double> sensorInfo = vehicleStatus.getDistanceFromObstacles();
    nn.feedForward(sensorInfo);
    vector<double> results;
    nn.getResults(results);
    processNNResults(results); //set new location for neural network
    nonHolonomicRelaxedCostMap() //current position, start position, get PercDone, add eval to geno
}
void Agent::processNNresults(vector<double> nnResults)
{
    Steer steer = Straight;
    if(nnResults[0] < .33)
    {
        steer = Left;
    }
    else if(nnResults[0 < .66])
    {
        steer = Right;
    }
    Gear gear = ( nnResults[1] > .5f) ? Forward : Backward;
    state = vehicleStatus.getNextState(state, steer, gear, nnResults[2]);
}
//----------------------------------------------------------------------------------------------------------------------------------
GeneticAlgorithm::GeneticAlgorithm(int genotypeParamCount, int populationSize)
{
    this->populationSize = populationSize;
    vector<Genotype> currentPopulation;
    currentPopulation.resize(populationSize,Genotype(vector<float>(genotypeParamCount, 0)));
    GenerationCount = 1;
    sortPopulation = true;
    running = false;
}
void GeneticAlgorithm::start()
{
    running = true;
    initPopulation(currentPopulation);
    evaluation(currentPopulation);
}

void GeneticAlgorithm::initPopulation(vector<Genotype> currentPop)
{
    for(Genotype geo : currentPop)
    {
        geo.setRandomParams(defaultInitParamMin, defaultInitParamMax);
    }
}
void GeneticAlgorithm::evaluation(vector<Genotype> currentPop)
{
    agents.clear();
    //refresh grid/start points
    vector<float> topology = {24, 12, 12, 6,  3};
    for(Genotype geno : currentPopulation)
    {
        agents.push_back(Agent(geno, topology));
    }
    int vehiclesCrashed = 0;
    while(vehiclesCrashed < currentPopulation.size())
    {
        for(Agent agent : agents)
        {
            if(agent.hasCrashed)
            {
                continue;
            }
            agent.update();
        }
    }
    evaluationFinished();
}
void GeneticAlgorithm::evaluationFinished()
{
    fitnessCalculation(currentPopulation);
    if(sortPopulation)
    {

    }
    if(false)
    {
        //quit
    }
    vector<Genotype> intermediatePop = selection(currentPopulation);
    vector<Genotype> newPopulation = recombination(intermediatePop, populationSize);
    mutation(newPopulation);
    currentPopulation = newPopulation;
    GenerationCount++;
    evaluation(currentPopulation);
}
void GeneticAlgorithm::fitnessCalculation(vector<Genotype> currentPop)
{
    int populationSize = 0;
    float overallEval = 0;
    for(Genotype geno : currentPop)
    {
        overallEval += geno.eval;
        populationSize++;
    }
    float averageEval = overallEval / populationSize;
    for(Genotype geno : currentPop)
    {
        geno.fitness = overallEval / populationSize;
    }
}
vector<Genotype> GeneticAlgorithm::selection(vector<Genotype> currentPop)
{
    vector<Genotype> intermediatePopulation;
    for(Genotype geno : currentPop)
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
    for(Genotype geno : currentPop)
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
void GeneticAlgorithm::mutation(vector<Genotype> newPop)
{
    for(int i = 2; i < newPop.size(); i++)
    {
        if(((double) rand() / (RAND_MAX)) < defaultMutationPerc)
        {
            mutateGenotype(newPop[i], defaultMutationProb, defaultMutationAmount);
        }
    }
}
vector<Genotype> GeneticAlgorithm::completeCrossover(Genotype parent1, Genotype parent2, float swapChance)
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
void GeneticAlgorithm::mutateGenotype(Genotype genotype, float mutationProb, float mutationAmount)
{
    for(float param : genotype.params)
    {
        if(((double) rand() / (RAND_MAX)) < mutationProb*100)
        {
            param += float(((double) rand() / (RAND_MAX)) * mutationAmount* 2 - mutationAmount);
        }
    }
}