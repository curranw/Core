#pragma once

#include <ILearningAlgorithm.h>
#include <armadillo>
#include <cmath>

class ValueIterationArgs : public ILearningArguments
{
public:
    vector<double> resolution;
};


class ValueIteration : public ILearningAlgorithm
{
public:

    ValueIteration(ValueIterationArgs *args);
    pair<QElement::Action, double> get_action(QElement::State s);
    void update_model(arma::Mat<double>* R_new, arma::Mat<double>* P_new);
    void update(QUpdate update);

    void calculate_states();
    void calculate_solution(bool debug = false);


    map<QElement::State, int>* get_state_to_ind();
    double get_Q(int ind, int action);
    int get_ind(QElement::State s);

    arma::Mat<double> Q_best;
    arma::Mat<double> V;
    //vector<double> m_min_ranges, m_max_ranges, m_resolution;
   // vector<int> m_possible_actions;
vector<QElement::State*> states;

private:

    int sub2ind( int row, int col, int cols);
    int getStateDimensionIndex(double val, double range_min, double range_max, double range_size);

    arma::Mat<double>* R;
    arma::Mat<double>* P;

    map<QElement::State, int> state_to_ind;

    ValueIterationArgs* m_args;


};
