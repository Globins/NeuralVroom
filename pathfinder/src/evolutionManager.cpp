#include "include/pathrouter/evolutionManager.hpp"

NeuralNet EvolutionManager::train(const vector<unsigned> &topology, const int trainAmount, const bool saveData)
{
    this->saveData = saveData;
    NeuralNet nn = NeuralNet(topology);
    GeneticAlgorithm ga = GeneticAlgorithm(nn.weightCount, 3, topology);
    genotypesSaved = 0;
    ga.start(trainAmount);
    if(saveData)
    {
        writeResultsToFile(ga.getPopulation());
    }
    nn.GenotypeParamsToWeights(ga.getPopulation()[0].params);
    return nn;
}

void EvolutionManager::writeResultsToFile(vector<Genotype> currentPop)
{

}