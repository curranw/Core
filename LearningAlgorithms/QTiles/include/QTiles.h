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

using namespace std;

class QTilesArguments : public ILearningArguments
{
public:
    int num_tiles;
    vector<double> resolution;
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
    void read(string file);
    bool no_new;
private:
    QTilesArguments* m_args;
    map<QElement*, vector<double> > eligability;
    vector<QElement*> calculate_nearby_states(QElement::State s);
    QElement *add_tiles(int tile);
    int tiles;
    double alpha, gamma;
    int memory_size;
    int* tiles_array;
    int cur_it;
    int count;
    int tilings;
    bool do_eligability;
    int test_it;

    set<QElement*> ele_to_update;
    unordered_map<QElement::Action, double> V_local;
};

#endif // QTILES_H
