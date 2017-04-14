//// Copyright 2016 Carrie Rebhuhn
//#ifndef SRC_LEARNING_INCLUDE_NEURALNET_H_
//#define SRC_LEARNING_INCLUDE_NEURALNET_H_

//#include <vector>
//#include <string>
//#include <iostream>
//#include <random>
//#include "MatrixTypes.h"
//#include "easymath.h"
//#include "FileIn.h"
//#include "IPolicy.h"

//typedef matrix1d State;
//typedef matrix1d Action;
//typedef double Reward;

//class NeuralNet : public IPolicy<State, Action, Reward> {
//public:
//    bool sigmoid_output_;
//    typedef Action Action;
//    typedef State State;
//    typedef Reward Reward;
//    double backProp(const matrix1d &X, const matrix1d &Y);
   
//    Action operator()(State s) const;
//    Reward getEvaluation() const { return evaluation_; }
//    void save(std::string fileout) const;

//    NeuralNet(size_t num_input, size_t num_hidden, size_t num_output,
//        double gamma = 0.9, double eta = 0.1, bool sigmoid_output = false);
//    void update(Reward R) { evaluation_ = R; }
//    void mutate();  // different if child class
//    void load(std::string file_in);
//    void load(matrix1d node_info, matrix1d wt_info);
//    explicit NeuralNet(std::string fname);

//    virtual ~NeuralNet() {}

//protected:
//    struct Layer {
//        const size_t k_num_nodes_above_, k_num_nodes_below_;
//        matrix2d w_bar_, w_;
//Layer(size_t above, size_t below);
//    };
    
//    std::vector<Layer> layers_;
//    std::normal_distribution<double> normal_;


//    const size_t N;
//    const double k_eta_;

//    // Changing elements (backprop)
//    struct BPState {
//        BPState() {
//            s = matrix2d(2);
//            z = matrix2d(2);
//            f = matrix2d(2);
//            d = matrix2d(2); //Will added this
//        }
//        matrix2d s, z, f, d;
//    };
    
//    NeuralNet() : k_gamma_(0.9), k_eta_(0.1), N(2), nn_state_(new BPState()) {}

//private:
//    // Life cycle


//    BPState* nn_state_;

//    matrix1d fwrdProp(matrix1d X, BPState* S) ;
//    void backProp(matrix1d Y, BPState* S);
//    void updateWt(matrix1d X, const BPState& S);

//    // Mutators
//    std::default_random_engine generator;


//    // Accessors
//    double evaluation_;
//    double k_gamma_;
//    double k_mut_std_;      //! mutation standard deviation
//    double k_mut_rate_;     //! probability that each connection is changed

//    matrix1d getTopology() const;
//    double mutateAmount();
//};

//#endif  // SRC_LEARNING_INCLUDE_NEURALNET_H_


/**********************************************************************************************************
  HEADER FILE: Backpropagation Neural Network Implementation
  File: bpnet.h
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


#ifndef BPNET_H
#define BPNET_H

/*********************************Structure representing a neuron******************************/
struct neuron
{
    float *weights; // neuron input weights or synaptic connections
    float *deltavalues; //neuron delta values
    float output; //output value
    float gain;//Gain value
    float wgain;//Weight gain value

    neuron();//Constructor
    ~neuron();//Destructor
    void create(int inputcount);//Allocates memory and initializates values
};
/**************************************Structure representing a layer******************************/
struct layer
{
    neuron **neurons;//The array of neurons
    int neuroncount;//Contains the total number of neurons
    float *layerinput;//The layer input
    int inputcount;//The total count of elements in layerinput

    bool use_sigmoid;
    layer();//Object constructor. Initializates all values as 0

    ~layer();//Destructor. Frees the memory used by the layer

    void create(int inputsize, int _neuroncount);//Creates the layer and allocates memory
    void calculate();//Calculates all neurons performing the network formula
};
/********************************Structure Representing the network********************************/
class bpnet
{
private:
    layer m_inputlayer;//input layer of the network
    layer m_outputlayer;//output layer..contains the result of applying the network
    layer **m_hiddenlayers;//Additional hidden layers
    int m_hiddenlayercount;//the count of additional hidden layers

public:
//function tu create in memory the network structure
    bpnet();//Construction..initialzates all values to 0
    ~bpnet();//Destructor..releases memory
    //Creates the network structure on memory
    void create(int inputcount,int inputneurons,int outputcount,int *hiddenlayers,int hiddenlayercount);

    void propagate(const float *input);//Calculates the network values given an input pattern
    //Updates the weight values of the network given a desired output and applying the backpropagation
    //Algorithm
    float train(const float *desiredoutput,const float *input,float alpha, float momentum);

    //Updates the next layer input values
    void update(int layerindex);

    //Returns the output layer..this is useful to get the output values of the network
    inline layer &getOutput()
    {
        return m_outputlayer;
    }

};

#endif // BPNET_H
