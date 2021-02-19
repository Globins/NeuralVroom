#include "../utils.hpp"

class Genotype
{
public:
    Genotype(vector<float> params);
    void setRandomParams(float minVal, float maxVal);
    vector<float> getDeepCopyParams();
    Genotype randomGenotype(int paramCount, float minVal, float maxVl);
    void SavetoFile(string path);
    void LoadFromFile(string path);
    float eval;
    float fitness;
private:
    vector<float> params;
};

struct GENOcomparator
{
    bool operator()(const Genotype* a, const Genotype* b)
    {
        return a->fitness > b->fitness;
    }
};


class GeneticAlgorithm
{
    public:
        GeneticAglorithm(int genotypeParamCount, int populationSize);
        void start();
        void initPopulation(vector<Genotype> currentPop);
        void evaluation(vector<Genotype> currentPop);
        void evaluationFinished();
        void fitnessCalculation(vector<Genotype> currentPop);
        void selection(vector<Genotype> currentPop);
        void recombination(vector<Genotype> currentPop, int newPopSize);
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

        vector<Genotype> currentPopulation;
        int populationSize;
        int GenerationCount;
        bool sortPopulation;
        bool running;


};