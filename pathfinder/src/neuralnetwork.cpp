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
NeuralNet::NeuralNet(const vector<unsigned> &topology)
{
    unsigned numLayers = topology.size();
    for(unsigned layerNum = 0; layerNum < numLayers; layerNum++)
    {
        m_layers.push_back(Layer());
        unsigned numOutputs = layerNum == topology.size()-1 ? 0 : topology[layerNum+1];
        for(unsigned neuronNum = 0; neuronNum <= topology[layerNum]; neuronNum++)
        {
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