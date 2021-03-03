// #include "include/pathrouter/geneticAlgorithm.hpp"

// Genotype::Genotype(vector<float> params)
// {
//     this->params = params;
//     fitness = 0;
// }

// //----------------------------------------------------------------------------------------------------------------------------------

// GeneticAglorithm::GeneticAlgorithm(int genotypeParamCount, int populationSize)
// {
//     this->populationSize = populationSize;
//     vector<Genotype> currentPopulation;
//     currentPopulation.resize(populationSize,Genotype(vector<float>(genotypeParamCount, 0)));
//     GenerationCount = 1;
//     sortPopulation = true;
//     running = false;
// }
// void GeneticAlgorithm::start()
// {
//     running = true;
//     initPopulation(currentPopulation);
//     evaluation(currentPopulation);
// }

// void GeneticAlgorithm::initPopulation(vector<Genotype> currentPop)
// {
//     for(Genotype geo : currentPop)
//     {
//         geo.setRandomParams(defaultInitParamMin, defaultInitParamMax);
//     }
// }
// void GeneticAlgorithm::evaluation(vector<Genotype> currentPop)
// {

// }
// void GeneticAlgorithm::evaluationFinished()
// {

// }
// void GeneticAlgorithm::fitnessCalculation(vector<Genotype> currentPop)
// {

// }
// void GeneticAlgorithm::selection(vector<Genotype> currentPop)
// {

// }
// void GeneticAlgorithm::recombination(vector<Genotype> currentPop, int newPopSize)
// {

// }
// void GeneticAlgorithm::mutation(vector<Genotype> newPop)
// {

// }
// vector<Genotype> GeneticAlgorithm::completeCrossover(Genotype parent1, Genotype parent2, float swapChance)
// {

// }
// void GeneticAlgorithm::utateGenotype(Genotype genotype, float mutationProb, float mutationAmount)
// {

// }