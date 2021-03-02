#include "../utils.hpp"
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
    Agent(Genotype genotype, vector<float> topology, VehicleState state);
    void update();
    void processNNresults(vector<double> nnResults);
    Vehicle vehicleStatus;
    VehicleState vehicleState;
    NeuralNet nn;
    bool hasCrashed = false;
};
//--------------------------------------------------------------------------------------------------------------------------------------------
class GeneticAlgorithm
{
    public:
        GeneticAlgorithm(int genotypeParamCount, int populationSize);
        void start();
        void initPopulation(vector<Genotype> currentPop);
        void evaluation(vector<Genotype> currentPop);
        void evaluationFinished();
        void fitnessCalculation(vector<Genotype> currentPop);
        vector<Genotype> selection(vector<Genotype> currentPop);
        vector<Genotype> recombination(vector<Genotype> intermediatePop, int newPopSize);
        void mutation(vector<Genotype> newPop);
        vector<Genotype> completeCrossover(Genotype parent1, Genotype parent2, float swapChance);
        void mutateGenotype(Genotype genotype, float mutationProb, float mutationAmount);
        
    private:
        float defaultInitParamMin = -1;
        float defaultInitParamMax = 1;
        float defaultCrossSwapProb = .6;
        float defaultMutationProb = .3;
        float defaultMutationAmount = 2;
        float defaultMutationPerc = 1;
        
        vector<Agent> agents;

        vector<Genotype> currentPopulation;
        int populationSize;
        int GenerationCount;
        bool sortPopulation;
        bool running;


};

