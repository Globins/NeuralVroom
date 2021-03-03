#include "geneticAlgorithm.hpp"
class EvolutionManager
{
public:
    NeuralNet train(const vector<unsigned> &topology, const int trainAmount, const bool saveData, mapGenerator &m);
    void startEvolution();
private:
    void writeResultsToFile(vector<Genotype> currentPop);
    void restart(GeneticAlgorithm ga);

    int genotypesSaved;
    bool saveData;

};