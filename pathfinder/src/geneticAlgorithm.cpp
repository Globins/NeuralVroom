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
    intermediatePopulation.push_back(currentPop[0]);
    intermediatePopulation.push_back(currentPop[1]);
    intermediatePopulation.push_back(currentPop[2]);

    return intermediatePopulation;
}
vector<Genotype> GeneticAlgorithm::recombination(vector<Genotype> intermediatePop, int newPopSize)
{
    if(intermediatePop.size() < 2)
    {
        throw new invalid_argument("Intermediate population too small");
    }
    vector<Genotype> newPopulation;
    while(newPopulation.size() < newPopSize)
    {
        vector<Genotype> offspringOneAndTwo = completeCrossover(intermediatePop[0], intermediatePop[1], defaultCrossSwapProb);
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
    for(Genotype geno : newPop)
    {
        if(((double) rand() / (RAND_MAX)) < defaultMutationPerc)
        {
            mutateGenotype(geno, defaultMutationProb, defaultMutationAmount);
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