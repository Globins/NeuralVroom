#include "../vehicle.hpp"
#include "../grid.hpp"

class Neuron;
struct Connection
{
    double weight;
    double deltaWeight;
};

typedef vector<Neuron> Layer;

class Neuron
{
public:
    Neuron(unsigned numOutputs, unsigned myIndex);
    void setOutputVal(double val) { m_outputVal = val; };
    double getOutputVal(void) const { return m_outputVal; };
    void feedForward(const Layer &prevLayer);
private:
    unsigned m_myIndex;
    static double transferFunction(double x);
    static double transferFunctionDerivative(double x);
    static double randomWeight(void) { return rand() / double(RAND_MAX);}
    double m_outputVal;
    vector<Connection> m_outputWeights;
    
};



class NeuralNet
{
public:
    NeuralNet(const vector<unsigned> &topology);
    void feedForward(const vector<double> &inputVals);
    void backProp(const vector<double> &argetVals);
    void getResults(vector<double> &resultVals) const;
    vector<VehicleState> run(Grid* grid, Vehicle* vehicle, VehicleState startPos, VehicleState endPos);
    bool areEquivalentStates(VehicleState comp, VehicleState other);

private:
    vector<Layer> m_layers; 
};