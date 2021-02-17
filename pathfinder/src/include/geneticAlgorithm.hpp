

class GeneticAglorithm
{
    public:
        GeneticAglorithm(int genotypeParamCount, int populationSize);
        void start();
        void EvaluationFinished();
        
    private:
        float defaultInitParamMin = -1;
        float defaultInitParamMax = 1;
        float defaultCrossSwapProb = .6;
        float defaultMutationProb = .3;
        float defaultMutationAmount = 2;
        float defaultMutationPerc = 1;

        int populationSize;
        int GenerationCount;
        bool sortPopulation;
        bool running;


};