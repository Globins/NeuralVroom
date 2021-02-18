#include "neuralnetwork.hpp"
#include "geneticAlgorithm.hpp"
class EvolutionManager
{
public:
    NeuralNet train(const vector<unsigned> &topology);
    void startEvolution();
private:
    void writeResultsToFile();
    void checkIfAgentReachedDest();
    void restartEvolution();
};