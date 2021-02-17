#include "neuralnetwork.hpp"
class EvolutionManager
{
public:
    void startEvolution();
private:
    void writeResultsToFile();
    void checkIfAgentReachedDest();
    void restartEvolution();
};