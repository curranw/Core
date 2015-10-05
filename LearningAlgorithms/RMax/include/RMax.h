#pragma once

#include <ILearningAlgorithm.h>
#include <ValueIteration.h>

class RMax : public ILearningAlgorithm
{
public:

    RMax(ILearningArguments *args);
    pair<QElement::Action, double> get_action(QElement::State s);
    void set_possible_actions(vector<int> possible_actions);
    void set_ranges(vector<double> min_ranges, vector<double> max_ranges);
    void set_resolution(vector<double> resolution);
    void update(QUpdate update);

private:
    ValueIteration* VI;

    arma::Mat<double> R_estimate;
    arma::Mat<double> D_estimate;
    arma::Mat<double> N_estimate;
    arma::Mat<double> P_estimate;
    map<QElement::State, int>* state_to_ind;

    double R_max;

};
