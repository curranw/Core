#pragma once
#include <ILearningAlgorithm.h>
#include <ValueIteration.h>
#include <linterp.h>
#include <algorithm>
#include <set>
class FittedRMaxArgs : public ILearningArguments
{
public:
    vector<double> resolution;
};

class FittedRMax : public ILearningAlgorithm
{
public:

    FittedRMax(FittedRMaxArgs *args);
    void init();
    pair<QElement::Action, double> get_action(QElement::State s);
    void set_possible_actions(vector<int> possible_actions);
    void set_ranges(vector<double> min_ranges, vector<double> max_ranges);
    void set_resolution(vector<double> resolution);
    void update(QUpdate update);

    double calculate_dist(QElement::State s1, QElement::Action a1, QElement::State s2, QElement::Action a2);
    map<int, double> calculate_mlinterp(QElement::State s1, vector<QElement::State> *states = 0);
    QElement::State action_effect_function(QElement::State s1, QElement::State s2, QElement::State s3);
private:
    ValueIteration* VI;

    arma::Mat<double> R_estimate;
    arma::Mat<double> R_temp;

    arma::Mat<double> D_estimate;

    arma::Mat<double> N_estimate;

    arma::Mat<double> P_estimate;
    arma::Mat<double> P_temp;


    map<QElement::State, int>* state_to_ind;

    set<int> updated_states;
    double R_max;

};
