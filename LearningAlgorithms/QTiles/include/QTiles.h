#pragma once
#ifndef QTILES_H
#define QTILES_H

#include <algorithm>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <tiles2.h>
#include <cmath>
#include <iostream>
#include <QElement.h>
#include <ILearningAlgorithm.h>
#include <utils.h>
#include <NeuralNet.h>

#include <doublefann.h>
#include <parallel_fann.hpp>
#include <fann_cpp.h>
#include <fann_train.h>


using namespace std;

class QTilesArguments : public ILearningArguments
{
public:
    int num_tiles;
    vector<double> resolution;
};

class NN_Experience
{
public:
    vector<double> state;
    vector<double> state_scaled;
    vector<double> next_state;
    vector<double> next_state_scaled;
    double reward;
    int action;
    vector<double> low_dim_values;
    vector<double> low_next_state_action_values;
    vector<double> test;
};

class QTiles : public ILearningAlgorithm
{
public:

    void clear_trace();
    QTiles(QTilesArguments* args);
    pair<QElement::Action, double> get_action(QElement::State s);
    vector<double> get_action_values(vector<QElement *> nearby_states);
    void update(QElement::State old_s, QElement::Action old_a, QElement::State new_s, double reward);
    int get_table_size();
    unordered_map<int, QElement *> *get_table();

    vector<QElement*> q_table;
    unordered_map<int, QElement*> q_table_m;
    map<int, vector<vector<double> >* > temp;


    pair<QElement::Action, double> random_action(vector<double> action_values);
    void update(QUpdate update);
    void e_update(QUpdate update);
    vector<QElement*> old_states;
    vector<double> get_action_values(QElement::State s);
    void e_update(QElement::State state, QElement::Action action, QElement::State new_state,double reward);
    virtual ~QTiles();
    void end_epoch();
    void init();
    void output(string file);
    void read(string file);
    bool no_new;
    double alpha, gamma;


    void set_table(unordered_map<int, QElement *> table);
    double get_average();
    void set_average(double average);
    vector<QElement*> get_elements(int *input_tiles);
    double default_weight;
private:
    QTilesArguments* m_args;
    map<QElement*, vector<double> > eligibility;
    vector<QElement*> calculate_nearby_states(QElement::State s);
    QElement *add_tiles(int tile);
    int num_tiles;
    int memory_size;
    int* tiles_array;
    int cur_it;
    int count;
    int tilings;
    bool do_eligability;
    int test_it;
    double cur_eps;
    bool use_tiles;
    set<QElement*> ele_to_update;
    unordered_map<QElement::Action, double> V_local;
    pair<QElement::Action, double> eligibility_action_value;

    bpnet* net;
    FANN::neural_net* fann_net;
    vector<vector<fann_type> > inputs;
    vector<vector<fann_type> > outputs;
    vector<NN_Experience> experience;
    bool minibatch;

    //NeuralNet* nn;
    QElement* nn_qele;
};

#endif // QTILES_H
