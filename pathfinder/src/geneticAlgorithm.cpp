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
Agent::Agent(Genotype genotype, vector<unsigned> topology, VehicleState state)
{
    vehicleState = state;
    nn = NeuralNet(topology);
    nn.GenotypeParamsToWeights(genotype.params);
}
void Agent::update()
{
    // vector<double> sensorInfo = vehicleStatus.getDistanceFromObstacles();
    // nn.feedForward(sensorInfo);
    vector<double> results;
    nn.getResults(results);
    vehicleState = nn.processResults(&vehicleStatus, vehicleState, results);
    //nonHolonomicRelaxedCostMap() //current position, start position, get PercDone, add eval to geno
     //check if crashed
}
//----------------------------------------------------------------------------------------------------------------------------------
GeneticAlgorithm::GeneticAlgorithm(int genotypeParamCount, int populationSize, const vector<unsigned> &topology)
{
    this->populationSize = populationSize;
    this->topology = topology;
    currentPopulation.resize(populationSize, Genotype(vector<float>(genotypeParamCount, 0)));
    GenerationCount = 1;
    sortPopulation = true;
}
void GeneticAlgorithm::start(const int trainAmount)
{
    this->trainAmount = trainAmount;
    initPopulation();
    //mapGenerator m = mapGenerator(50, 50, vector<int>{3,3},7,numVehicles);
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
    // //refresh grid/start points
    //NEED WALDO
    for(Genotype geno : currentPopulation)
    {
        //agents.push_back(Agent(geno, topology));
    }
    int vehiclesCrashed = 0;
    while(vehiclesCrashed < currentPopulation.size())
    {
        for(Agent &agent : agents)
        {
            //check their locations, flag hasCrashed if any true
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
    fitnessCalculation();
    if(sortPopulation)
    {

    }
    if(GenerationCount < trainAmount)
    {
        vector<Genotype> intermediatePop = selection();
        vector<Genotype> newPopulation = recombination(intermediatePop, populationSize);
        mutation(newPopulation);
        currentPopulation = newPopulation;
        GenerationCount++;
        evaluation();
    }
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