#include "include/pathrouter/evolutionManager.hpp"

//vector<double> inputs = getInputs(m.getMap(), vehicleIDMap[id], startPoints[id], endPoints[id]);
NeuralNet EvolutionManager::train(const vector<unsigned> &topology, Grid* grid, bool saveData)
{
    this->topology = topology;
}
void EvolutionManager::startEvolution()
{
    NeuralNet nn = NeuralNet(topology);
    //GeneticAglorithm ga = GeneticAlgorithm(nn.weightCount, 3);
    //ga.start();
    genotypesSaved = 0;

    // if(saveData)
    // {
    //     //Write weights to file
    // }

}