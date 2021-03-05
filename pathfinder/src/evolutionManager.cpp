#include "include/pathrouter/evolutionManager.hpp"

NeuralNet EvolutionManager::train(const vector<unsigned> &topology, const int trainAmount, const bool saveData, mapGenerator &m)// add in map
{
    this->saveData = saveData;
    NeuralNet nn = NeuralNet(topology);
    GeneticAlgorithm ga = GeneticAlgorithm(nn.weightCount, m.getStartPoints().size(), topology, m);
    ga.start(trainAmount);
    if(saveData)
    {
        writeResultsToFile(ga.getPopulation());
    }
    cout <<"DONE" << endl;
    for(Genotype geno : ga.getPopulation())
    {
        cout << geno.eval << ", " << geno.fitness << endl;
    }
    nn.GenotypeParamsToWeights(ga.getPopulation()[0].params);
    return nn;
}

void EvolutionManager::writeResultsToFile(vector<Genotype> currentPop)
{
    ofstream weights;
    weights.open("weights.txt");
    for(Genotype geno : currentPop)
    {
        for(float param : geno.params)
        {
            weights << param << " ";
        }
        weights << "\n";
    }
    weights.close();
}