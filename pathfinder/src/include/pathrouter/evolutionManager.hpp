#include "neuralnetwork.hpp"
#include "geneticAlgorithm.hpp"
#include "../grid.hpp"
class EvolutionManager
{
public:
    NeuralNet train(const vector<unsigned> &topology, Grid* Grid, bool saveData);
    void startEvolution();
private:
    void writeResultsToFile();
    void checkIfAgentReachedDest();
    void restartEvolution();

    vector<unsigned> topology;
    int genotypesSaved;

};