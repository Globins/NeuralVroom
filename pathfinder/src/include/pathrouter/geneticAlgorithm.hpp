#include "../utils.hpp"
#include "../grid.hpp"
#include "../mapGenerator.hpp"
#include "../vehicle.hpp"
#include "../pathrouter/neuralnetwork.hpp"
class Genotype
{
public:
    Genotype(vector<float> params);
    void setRandomParams(float minVal, float maxVal);
    Genotype randomGenotype(int paramCount, float minVal, float maxVl);
    void SavetoFile(string path);
    void LoadFromFile(string path);
    float eval;
    float fitness;
    vector<float> params;
    
};

struct GENOcomparator
{
    bool operator()(const Genotype* a, const Genotype* b)
    {
        return a->fitness > b->fitness;
    }
};

struct Agent
{
public:
    Agent(Genotype genotype, vector<unsigned> topology, VehicleState state);
    void update(vector<vector<int>> m);

    Vehicle vehicleStatus = Vehicle(10, 16.5);
    VehicleState vehicleState;
    NeuralNet nn = NeuralNet({1});
    bool hasCrashed = false;
};
//--------------------------------------------------------------------------------------------------------------------------------------------
class GeneticAlgorithm
{
    public:
        GeneticAlgorithm(int genotypeParamCount, int populationSize, const vector<unsigned> &topology, mapGenerator &m);
        void start(const int trainAmount);
        void evaluation();
        vector<Genotype> getPopulation();
        void printPopulation();
        
    private:
        void initPopulation();
        void evaluationFinished();
        void fitnessCalculation();
        vector<Genotype> selection();
        vector<Genotype> recombination(vector<Genotype> intermediatePop, int newPopSize);
        void mutation(vector<Genotype> &newPop);
        vector<Genotype> completeCrossover(const Genotype &parent1, const Genotype &parent2, float swapChance);
        void mutateGenotype(Genotype &genotype, float mutationProb, float mutationAmount);

        float defaultInitParamMin = -1;
        float defaultInitParamMax = 1;
        float defaultCrossSwapProb = .6;
        float defaultMutationProb = .3;
        float defaultMutationAmount = 2;
        float defaultMutationPerc = 1;
        
        mapGenerator* mapGenPtr;
        vector<Agent> agents;
        vector<unsigned> topology;
        vector<Genotype> currentPopulation;
        int populationSize;
        int GenerationCount;
        int trainAmount;
        bool sortPopulation;
};

