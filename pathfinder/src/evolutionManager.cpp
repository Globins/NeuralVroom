#include "include/pathrouter/evolutionManager.hpp"

//vector<double> inputs = getInputs(m.getMap(), vehicleIDMap[id], startPoints[id], endPoints[id]);
NeuralNet EvolutionManager::train(const vector<unsigned> &topology, int trainAmount, bool saveData)
{
    this->topology = topology;
    this->trainAmount = trainAmount;
    this->saveData = saveData;
    NeuralNet nn = NeuralNet(topology);
    GeneticAlgorithm ga = GeneticAlgorithm(nn.weightCount, 3);
    genotypesSaved = 0;
    if(saveData)
    {
        //Write weights to file
    }
    ga.start();

}
void EvolutionManager::startEvolution()
{
    // vector<NeuralNet>;
    // vector<Vehicle>;
    // while(true)
    // {

    // }

}
void EvolutionManager::restartEvolution()
{
    startEvolution();
}
// we actually modify the genotypes in the population and create new neural networks every time
// eval for params are defaulted by the overall dijkstra distance from the end position
// after all cars make it to position or crash, the thing is calculated again 

//question is how to we make sure that the right choice is the closest dijkstra distance

// well, we'll only be applying this to get around obstacles
//therefore, 