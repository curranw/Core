#pragma once

#include <ILearningAlgorithm.h>
#include <armadillo>
#include <cmath>


class ValueIteration : public ILearningAlgorithm
{
public:

    ValueIteration(ILearningArguments *args);
    pair<QElement::Action, double> get_action(QElement::State s);
    void update_model(arma::Mat<double> R_new, arma::Mat<double> P_new);
    void update(QUpdate update);

    void calculate_states();
    void calculate_solution();

    //vector<double> m_min_ranges, m_max_ranges, m_resolution;
   // vector<int> m_possible_actions;


private:
    arma::Mat<double> R;
    arma::Mat<double> V;
    arma::Mat<double> P;
    arma::Mat<double> Q_best;
    map<QElement::State, int> state_to_ind;


};
