#include "neuralnetwork.hpp"
#include "geneticAlgorithm.hpp"
#include "../grid.hpp"
#include "../mapGenerator.hpp"
#include "../vehicle.hpp"
class EvolutionManager
{
public:
    NeuralNet train(const vector<unsigned> &topology, int trainAmount, bool saveData);
    void startEvolution();
private:
    void writeResultsToFile();
    void checkIfAgentReachedDest();
    void restartEvolution();

    vector<unsigned> topology;
    int genotypesSaved;
    int trainAmount;
    bool saveData;

};
//EVAL == % DIJKSTRA DISTANCE FROM ENDGOAL