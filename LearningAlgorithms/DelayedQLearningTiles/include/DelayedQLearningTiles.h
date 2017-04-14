#pragma once
#ifndef DELAYEDQTILES_H
#define DELAYEDQTILES_H

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

using namespace std;

class QElementDelayed : public QElement
{
public:
    vector<double> u;
    vector<bool> learn;
    vector<int> l;
    vector<int> t;
};

class DelayedQLearningTilesArguments : public ILearningArguments
{
public:
    int num_tiles;
    vector<double> resolution;
};

class DelayedQLearningTiles : public ILearningAlgorithm
{
public:
    int t;
    int cur_timestep;
    int m;
    void clear_trace();
    DelayedQLearningTiles(DelayedQLearningTilesArguments* args);
    pair<QElement::Action, double> get_action(QElement::State s);
    vector<double> get_action_values(vector<QElement *> nearby_states);
    vector<double> get_update_values(vector<QElement *> nearby_states);

    void update(QElement::State old_s, QElement::Action old_a, QElement::State new_s, double reward);
    int get_table_size();

    unordered_map<int, QElement*> q_table_m;


    pair<QElement::Action, double> random_action(vector<double> action_values);
    void update(QUpdate update);
    void e_update(QUpdate update);
    vector<QElement*> old_states;
    vector<double> get_action_values(QElement::State s);
    void e_update(QElement::State state, QElement::Action action, QElement::State new_state,double reward);
    virtual ~DelayedQLearningTiles();
    void end_epoch();
    void init();
    void output(string file);
    void read(string file);
    bool no_new;
    double alpha, gamma;
    double epsilon_error;

    double get_average();
    void set_average(double average);
    double default_weight;
private:
    DelayedQLearningTilesArguments* m_args;
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
    set<QElement*> ele_to_update;
    unordered_map<QElement::Action, double> V_local;
    pair<QElement::Action, double> eligibility_action_value;
};

#endif // QTILES_H
