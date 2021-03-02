#include "include/pathrouter/neuralnetwork.hpp"

Neuron::Neuron(unsigned numOutputs, unsigned myIndex)
{
    for(unsigned c = 0; c < numOutputs; c++)
    {
        m_outputWeights.push_back(Connection());
        m_outputWeights.back().weight = randomWeight();
    }
    m_myIndex = myIndex;
}
void Neuron::feedForward(const Layer &prevLayer)
{
    double sum = 0.0;
    for(unsigned n = 0; n < prevLayer.size(); n++)
    {
        sum += prevLayer[n].getOutputVal() * prevLayer[n].m_outputWeights[m_myIndex].weight;
    }
    m_outputVal = Neuron::transferFunction(sum);
}
double Neuron::transferFunction(double x)
{
    return tanh(x);
}
double Neuron::transferFunctionDerivative(double x)
{
    return 1 - x*x;
}


//----------------------------------------------------------------------------------------------------------------------------------
NeuralNet::NeuralNet(const vector<unsigned> &topology)
{
    unsigned numLayers = topology.size();
    for(unsigned layerNum = 0; layerNum < numLayers; layerNum++)
    {
        m_layers.push_back(Layer());
        unsigned numOutputs = layerNum == topology.size()-1 ? 0 : topology[layerNum+1];
        for(unsigned neuronNum = 0; neuronNum <= topology[layerNum]; neuronNum++)
        {
            weightCount++;
            m_layers.back().push_back(Neuron(numOutputs, neuronNum));
        }
    }
}
void NeuralNet::feedForward(const vector<double> &inputVals)
{
    for(unsigned i = 0; i < inputVals.size(); i++)
    {
        m_layers[0][i].setOutputVal(inputVals[i]);
    }
    for(unsigned layerNum = 1; layerNum < m_layers.size(); layerNum++)
    {
        Layer &prevLayer = m_layers[layerNum-1];
        for(unsigned n = 0; n < m_layers[layerNum].size() - 1; n++)
        {
            m_layers[layerNum][n].feedForward(prevLayer);
        }
    }
}
void NeuralNet::getResults(vector<double> &resultVals) const{
    resultVals.clear();
    for(unsigned n = 0; n < m_layers.back().size() -1; n++){
        resultVals.push_back(m_layers.back()[n].getOutputVal());
    }


}
VehicleState NeuralNet::processResults(Vehicle* vehicle, VehicleState state, const vector<double> &resultVals)
{
    Steer steer = Straight;
    if(resultVals[0] < .33)
    {
        steer = Left;
    }
    else if(resultVals[0 < .66])
    {
        steer = Right;
    }
    Gear gear = ( resultVals[1] > .5f) ? Forward : Backward;
    return vehicle->getNextState(state, steer, gear, resultVals[2]);
}
vector<VehicleState> NeuralNet::run(Grid* grid, Vehicle* vehicle, VehicleState startPos, VehicleState endPos)
{
    vector<VehicleState> path;
    VehicleState currentPos = startPos; 
    path.push_back(currentPos);
    vector<vector<int>> m = grid->returnRawMap();
    while(grid->isSafe(currentPos, 1.5) || areEquivalentStates(currentPos, endPos))
    {
        vector<double> inputs = vehicle->getDistanceFromObstacles(m, currentPos);
        feedForward(inputs);
        vector<double> results;
        getResults(results);
        path.push_back(processResults(vehicle, currentPos, results));
    }
    return path;
}
bool NeuralNet::areEquivalentStates(VehicleState comp, VehicleState other)
{
    return comp.posX == other.posX && comp.posY ==  other.posY && comp.ori == other.ori && comp.gear == other.gear;
}