//// Copyright 2016 Carrie Rebhuhns
//#include "NeuralNet.h"
//#include <vector>
//#include <string>
//#include <ctime>
//#include <random>

//using easymath::rand;
//using easymath::sum;
//using easymath::operator*;
//using easymath::operator-;
//using easymath::dot;
//using easymath::flatten;
//using easymath::sigmoid;
//using easymath::sigmoidDerivative;
//using easymath::randSetFanIn;
//using easymath::T;
//using std::vector;
//using std::string;

//Action NeuralNet::operator()(State X) const{
//    X.push_back(1.0);
    
//    auto s = X*layers_[0].w_bar_;
//    auto z = sigmoid(s);

//    for (size_t i = 1; i < N; i++) {
//        auto zbias = z;
//        zbias.push_back(1.0);
//        s = zbias*layers_[i].w_bar_;
//        z = sigmoid(s);
//    }
//    if (sigmoid_output_)
//        return z;
//    else
//        return s;
//}

//void NeuralNet::updateWt(matrix1d X, const BPState& S) {
//    X.push_back(1.0);
//    layers_[0].w_bar_ = layers_[0].w_bar_ - k_eta_*T(S.d[0] * X);
//    for (size_t i = 0, j = 1; j < N; i++, j++) {
//        matrix1d zbias = S.z[i];
//        zbias.push_back(1.0);
//        matrix2d mod = T(S.d[j] * zbias);
//        layers_[j].w_bar_ = layers_[j].w_bar_ - k_eta_*(mod);
//    }
//}

//void NeuralNet::backProp(matrix1d Y, BPState *S) {
//    S->d[N - 1] = S->z[N - 1] - Y;

//    size_t startnum = N - 2;
//    size_t endnum = 0;

//    int i = 0;
//    auto w_nobias = layers_[i].w_;
//    matrix2d m = w_nobias[i] * S->d[i + 1];

//    S->d[i] = dot(S->f[i], flatten(m));
//}

//matrix1d NeuralNet::fwrdProp(matrix1d X, BPState* S) {
//    X.push_back(1.0);
//    if (S == NULL) {
//        S = new BPState();
//    }

//    S->s[0] = X*layers_[0].w_bar_;
//    S->z[0] = sigmoid(S->s[0]);
//    S->f[0] = sigmoidDerivative(S->s[0]);

//    for (size_t i = 1; i < N; i++) {
//        auto zbias = S->z[i - 1];
//        zbias.push_back(1.0);
//        S->s[i] = zbias*layers_[i].w_bar_;
//        S->z[i] = sigmoid(S->s[i]);
//        S->f[i] = sigmoidDerivative(S->s[i]);
//    }
//    return S->z[N - 1];
//}


//double NeuralNet::backProp(const matrix1d &X, const matrix1d &Y) {
//    // 'X' is the input vector, 'Y' is the 'target vector'
//    // returns the SSE for the output vector

//    fwrdProp(X, nn_state_);
//    backProp(Y, nn_state_);
//    updateWt(X, *nn_state_);
//    return 0.0;
//}

//NeuralNet::Layer::Layer(size_t above, size_t below) :
//    k_num_nodes_above_(above), k_num_nodes_below_(below) {
//    // Populate Wbar with small random weights, including bias
//    w_bar_ = easymath::zeros(above + 1, below);

//    for (int i = 0; i < w_bar_.size(); i++) {
//        for (int j = 0; j < w_bar_[i].size(); j++) {
//            w_bar_[i][j] = easymath::rand(-1.0, 1.0);
//        }
//    }

//    w_ = w_bar_;
//    w_.pop_back();
//}

//void NeuralNet::mutate() {
//    for (Layer &l : layers_) {
//        for (matrix1d &wt_outer : l.w_bar_) {
//            double fan_in = static_cast<double>(l.k_num_nodes_above_);
//            // #pragma parallel omp for
//            for (double &w : wt_outer) {
//                w += mutateAmount();
//            }
//        }
//    }
//}

//double NeuralNet::mutateAmount() {
//    // Adds random amount mutRate_% of the time,
//    if (rand(0, 1) > k_mut_rate_) {
//        return 0.0;
//    } else {
//        return (pow(-1,rand()%2))*normal_(generator);
//    }
//}

//NeuralNet::NeuralNet(size_t num_inputs, size_t num_hidden, size_t num_outputs, double gamma, double eta, bool sigmoid_output) :
//    k_gamma_(gamma), k_eta_(eta), evaluation_(0), k_mut_rate_(0.5), k_mut_std_(10.0), N(2), sigmoid_output_(sigmoid_output) {
    
//    nn_state_ = new BPState();

//    // 1 hidden layer neural net (corresponds to N of 2)
//    layers_.push_back(Layer(num_inputs, num_hidden));
//    layers_.push_back(Layer(num_hidden, num_outputs));
//    // generator.seed(0);

//    normal_ = std::normal_distribution<double>(0.0, k_mut_std_);
//}

//void NeuralNet::load(string filein) {
//    matrix2d wts = cio::read2<double>(filein);
//    load(wts[0], wts[1]);
//}

//void NeuralNet::save(string fileout) const {
//    matrix2d out(2);
//    out[0] = getTopology();
//    for (Layer l : layers_) {
//        matrix1d flat_wbar = flatten(l.w_bar_);
//        out[1].insert(out[1].end(), flat_wbar.begin(), flat_wbar.end());
//    }
//    cio::print2(out, fileout, false);
//}

//NeuralNet::NeuralNet(std::string fname) : NeuralNet() {
//    load(fname);
//}

//void NeuralNet::load(matrix1d node_info, matrix1d wt_info) {
//    size_t num_inputs = static_cast<int>(node_info[0]);
//    size_t num_hidden = static_cast<int>(node_info[1]);
//    size_t num_outputs = static_cast<int>(node_info[2]);

//    layers_ = std::vector<Layer>();
//    layers_.push_back(Layer(num_inputs, num_hidden));
//    layers_.push_back(Layer(num_hidden, num_outputs));

//    int index = 0;  // index for accessing NN elements
//    for (Layer &l : layers_) {  // number of layers
//        for (matrix1d &wt_outer : l.w_bar_) {
//            for (double &wt_inner : wt_outer) {
//                wt_inner = wt_info[index++];
//            }
//        }
//        l.w_ = l.w_bar_;
//        l.w_.pop_back();
//    }
//}


//matrix1d NeuralNet::getTopology() const {
//    matrix1d topology(1);
//    topology[0] = layers_.front().k_num_nodes_above_;
//    for (Layer l : layers_)
//        topology.push_back(static_cast<double>(l.k_num_nodes_below_));
//    return topology;
//}


/**********************************************************************************************************
  C++ Source FILE: Backpropagation Neural Network Implementation
  File: bpnet.cpp
  Version: 0.1
  Copyright(C) NeuroAI (http://www.learnartificialneuralnetworks.com)
  Documentation:http://www.learnartificialneuralnetworks.com/neural-network-software/backpropagation-source-code/
  NeuroAI Licence:
  Permision granted to use this source code only for non commercial and educational purposes.
  You can distribute this file but you can't take ownership of it or modify it.
  You can include this file as a link on any website but you must link directly to NeuroAI website
  (http://www.learnartificialneuralnetworks.com)
  Written by Daniel Rios <daniel.rios@learnartificialneuralnetworks.com> , June 2013

 /*********************************************************************************************************/
#include "NeuralNet.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/*****************************neuron routines*******************************/

neuron::neuron():weights(0),deltavalues(0),output(0),gain(0),wgain(0) //constructor
{

}
//Destructor
neuron::~neuron()
{
    if(weights)
        delete [] weights;
    if(deltavalues)
        delete [] deltavalues;
}

//Initializates neuron weights
void neuron::create(int inputcount)
{
    assert(inputcount);
    float sign=-1;//to change sign
    float random;//to get random number
    weights=new float[inputcount];
    deltavalues=new float[inputcount];
    //important initializate all weights as random unsigned values
    //and delta values as 0
    for(int i=0;i<inputcount;i++)
    {
        //get a random number between -0.5 and 0.5
        random=(float(rand()) / float(RAND_MAX))/2.f; //min 0.5
        random*=sign;
        sign*=-1;
        weights[i]=random;
        deltavalues[i]=0;
    }
    gain=1;

    random=(float(rand()) / float(RAND_MAX))/2.f; //min 0.5
    random*=sign;
    sign*=-1;
    wgain=random;




}


/***********************************Layer member functions********************************/
layer::layer():neurons(0),neuroncount(0),layerinput(0),inputcount(0)
{
    use_sigmoid = true;
}

layer::~layer()
{
    if(neurons)
    {
        for(int i=0;i<neuroncount;i++)
        {
            delete neurons[i];
        }
        delete [] neurons;
    }
    if(layerinput)
    {
        delete [] layerinput;
    }
}

void layer::create(int inputsize, int _neuroncount)
{
    assert(inputsize && _neuroncount);//check for errors
    int i;
    neurons=new neuron*[_neuroncount];
    for(i=0;i<_neuroncount;i++)
    {
        neurons[i]=new neuron;
        neurons[i]->create(inputsize);
    }

    layerinput=new float[inputsize];
    neuroncount=_neuroncount;
    inputcount=inputsize;
}
//Calculates the neural network result of the layer using the sigmoid function
void layer::calculate()
{
    int i,j;
    float sum;
    //Apply the formula for each neuron
    for(i=0;i<neuroncount;i++)
    {
        sum=0;//store the sum of all values here
        for(j=0;j<inputcount;j++)
        {
        //Performing function
            sum+=neurons[i]->weights[j] * layerinput[j]; //apply input * weight
        }
        sum+=neurons[i]->wgain * neurons[i]->gain; //apply the gain or theta multiplied by the gain weight.
        //sigmoidal activation function


        neurons[i]->output= 1.f/(1.f + exp(-sum));//calculate the sigmoid function
       // neurons[i]->output=-1 + 2*(1.f + exp(-sum));
    }
}



/***************************bpnet object functions**************/
bpnet::bpnet():m_hiddenlayers(0),m_hiddenlayercount(0)
{
}

bpnet::~bpnet()
{
    if(m_hiddenlayers)
    {
        for(int i=0;i<m_hiddenlayercount;i++)
        {
            delete m_hiddenlayers[i];
        }

        delete [] m_hiddenlayers;
    }

}

void bpnet::create(int inputcount, int inputneurons, int outputcount, int *hiddenlayers, int hiddenlayercount)
{
   //make sure required values are not zero
            assert(inputcount && inputneurons && outputcount);
            int i;
            m_inputlayer.create(inputcount,inputneurons);
            if(hiddenlayers && hiddenlayercount)
            {
                m_hiddenlayers=new layer*[hiddenlayercount];
                m_hiddenlayercount=hiddenlayercount;
                for(i=0;i<hiddenlayercount;i++)
                {
                    m_hiddenlayers[i]=new layer;
                    if(i==0)
                    {
                    //first hidden layer receives the output of the inputlayer so we set as input the neuroncount
                    //of the inputlayer
                        m_hiddenlayers[i]->create(inputneurons,hiddenlayers[i]);
                    }
                    else
                    {
                        m_hiddenlayers[i]->create(hiddenlayers[i-1],hiddenlayers[i]);
                    }
                }
                m_outputlayer.create(hiddenlayers[hiddenlayercount - 1],outputcount);
                m_outputlayer.use_sigmoid = false;
            }
            else
            {
                m_outputlayer.create(inputneurons,outputcount);
                m_outputlayer.use_sigmoid = false;
            }
}


void bpnet::propagate(const float *input)
{
    //The propagation function should start from the input layer
    //first copy the input vector to the input layer Always make sure the size
    //"array input" has the same size of inputcount
    memcpy(m_inputlayer.layerinput,input,m_inputlayer.inputcount * sizeof(float));
    //now calculate the inputlayer
    m_inputlayer.calculate();

    update(-1);//propagate the inputlayer out values to the next layer
    if(m_hiddenlayers)
    {
        //Calculating hidden layers if any
        for(int i=0;i<m_hiddenlayercount;i++)
        {
            m_hiddenlayers[i]->calculate();
            update(i);
        }
    }

    //calculating the final statge: the output layer
    m_outputlayer.calculate();
}
//Main training function. Run this function in a loop as many times needed per pattern
float bpnet::train(const float *desiredoutput, const float *input, float alpha, float momentum)
{
    //function train, teaches the network to recognize a pattern given a desired output
    float errorg=0; //general quadratic error
    float errorc; //local error;
    float sum=0,csum=0;
    float delta,udelta;
    float output;
    //first we begin by propagating the input
    propagate(input);
    int i,j,k;
    //the backpropagation algorithm starts from the output layer propagating the error  from the output
    //layer to the input layer
    for(i=0;i<m_outputlayer.neuroncount;i++)
    {
        //calculate the error value for the output layer
        output=m_outputlayer.neurons[i]->output*10000; //copy this value to facilitate calculations
        //from the algorithm we can take the error value as
        errorc=(desiredoutput[i] - output) * output * (1 - output);
        //and the general error as the sum of delta values. Where delta is the squared difference
        //of the desired value with the output value
        //quadratic error
        errorg+=(desiredoutput[i] - output) * (desiredoutput[i] - output) ;

        //now we proceed to update the weights of the neuron
        for(j=0;j<m_outputlayer.inputcount;j++)
        {
            //get the current delta value
            delta=m_outputlayer.neurons[i]->deltavalues[j];
            //update the delta value
            udelta=alpha * errorc * m_outputlayer.layerinput[j] + delta * momentum;
            //update the weight values
            m_outputlayer.neurons[i]->weights[j]+=udelta;
            m_outputlayer.neurons[i]->deltavalues[j]=udelta;

            //we need this to propagate to the next layer
            sum+=m_outputlayer.neurons[i]->weights[j] * errorc;
        }

        //calculate the weight gain
        m_outputlayer.neurons[i]->wgain+= alpha * errorc * m_outputlayer.neurons[i]->gain;

    }

    for(i=(m_hiddenlayercount - 1);i>=0;i--)
    {
        for(j=0;j<m_hiddenlayers[i]->neuroncount;j++)
        {
            output=m_hiddenlayers[i]->neurons[j]->output;
            //calculate the error for this layer
            errorc= output * (1-output) * sum;
            //update neuron weights
            for(k=0;k<m_hiddenlayers[i]->inputcount;k++)
            {
                delta=m_hiddenlayers[i]->neurons[j]->deltavalues[k];
                udelta= alpha * errorc * m_hiddenlayers[i]->layerinput[k] + delta * momentum;
                m_hiddenlayers[i]->neurons[j]->weights[k]+=udelta;
                m_hiddenlayers[i]->neurons[j]->deltavalues[k]=udelta;
                csum+=m_hiddenlayers[i]->neurons[j]->weights[k] * errorc;//needed for next layer

            }

            m_hiddenlayers[i]->neurons[j]->wgain+=alpha * errorc * m_hiddenlayers[i]->neurons[j]->gain;

        }
        sum=csum;
        csum=0;
    }

    //and finally process the input layer
    for(i=0;i<m_inputlayer.neuroncount;i++)
    {
        output=m_inputlayer.neurons[i]->output;
        errorc=output * (1 - output) * sum;

        for(j=0;j<m_inputlayer.inputcount;j++)
        {
            delta=m_inputlayer.neurons[i]->deltavalues[j];
            udelta=alpha * errorc * m_inputlayer.layerinput[j] + delta * momentum;
            //update weights
            m_inputlayer.neurons[i]->weights[j]+=udelta;
            m_inputlayer.neurons[i]->deltavalues[j]=udelta;
        }
        //and update the gain weight
        m_inputlayer.neurons[i]->wgain+=alpha * errorc * m_inputlayer.neurons[i]->gain;
    }

    //return the general error divided by 2
    return errorg / 2;

}

void bpnet::update(int layerindex)
{
    int i;
    if(layerindex==-1)
    {
        //dealing with the inputlayer here and propagating to the next layer
        for(i=0;i<m_inputlayer.neuroncount;i++)
        {
            if(m_hiddenlayers)//propagate to the first hidden layer
            {
                m_hiddenlayers[0]->layerinput[i]=m_inputlayer.neurons[i]->output;
            }
            else //propagate directly to the output layer
            {
                m_outputlayer.layerinput[i]=m_inputlayer.neurons[i]->output;
            }
        }

    }
    else
    {
        for(i=0;i<m_hiddenlayers[layerindex]->neuroncount;i++)
        {
            //not the last hidden layer
            if(layerindex < m_hiddenlayercount -1)
            {
                m_hiddenlayers[layerindex + 1]->layerinput[i]=m_hiddenlayers[layerindex]->neurons[i]->output;
            }
            else
            {
                m_outputlayer.layerinput[i]=m_hiddenlayers[layerindex]->neurons[i]->output;
            }
        }
    }
}

