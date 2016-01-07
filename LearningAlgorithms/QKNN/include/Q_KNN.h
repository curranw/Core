/*
 * Q_KNN.h
 *
 *  Created on: Jun 19, 2015
 *      Author: Will Curran
 */

#pragma once
#ifndef Q_KNN_H_
#define Q_KNN_H_

#include <ANN/ANN.h>
#include <set>
#include <vector>
#include <algorithm>
#include <numeric>
#include <map>
#include <QElement.h>

using namespace std;
struct ANN_STRUCT
{
	int n_pts; // actual number of data points
	ANNpointArray data_pts; // data points
	ANNpoint query_pt; // query point
	ANNidxArray nn_idx; // near neighbor indices
	ANNdistArray dists; // near neighbor distances
	ANNkd_tree* kd_tree; // search structure
	unsigned int num_neighbors; // number of nearest neighbors
	int dim; // dimension
	double eps; // error bound
};


class Q_KNN {
public:
	Q_KNN(double ai = 0.9, double gi = 0.9);
	void init_knn(unsigned int num_neighbors, int dim, double eps);
    QElement::Action get_action(QElement::State s);
    map<int, double> get_action_values(QElement::State s);
    pair<QElement::Action, double> random_action(map<int, double> action_values);
    void update(QElement::State old_s, QElement::Action old_a, QElement::State new_s, double reward);
    void update(QUpdate update);
    void e_update(QUpdate update);
    void e_update(QElement::State old_s, QElement::Action old_a, QElement::State new_s, double reward);

    vector<QElement*> calculate_nearby_states(QElement::State s);
    int get_table_size();
    void set_possible_actions(vector<QElement::Action> new_possible_actions);
    void reset_table(vector<QElement*> new_table);
    void get_nearest_state(QElement::State* s, QElement::State **nearest_state);
    void set_range(vector<double> ranges_min_input, vector<double> ranges_max_input);
    void set_resolution(vector<double> resolution);
    void init_q_table();
    void clear_trace();
    void update_state(QElement::State s, vector<double> V);
    virtual ~Q_KNN();

    pair<QElement::State, map<int, double> > old_values;
    vector<QElement*> old_states;
    vector<double> old_probabilities;
    vector<QElement*> q_table;
    bool debug;


private:
    set<QElement*> ele_to_update;
    map<QElement*, vector<double> > eligability;

    //Learning Weights
	vector<double> calculate_weights(ANNdistArray nearby_dists);
	vector<double> calculate_probabilities(vector<double>* weights);
	map<int, double> calculate_values(vector<QElement*> nearby_states, vector<double>* probabilities);

	void knn_search();

	//Learning params
    double alpha, gamma;
    vector<double> ranges_min, ranges_max;
    vector<double> resolution;
	int cur_it;

	ANN_STRUCT ann_struct;
    double old_value;
    vector<double> all_actions;
	int cur_neighbors;
    vector<QElement::Action> possible_actions;
};

#endif /* Q_KNN_H_ */
