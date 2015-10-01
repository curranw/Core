#pragma once

#include <IDomain.h>
#include <armadillo>

class RobotChasing : public IDomain
{
public:
    RobotChasing();
    void init();
    vector<double> get_state();
    void step(int action);
    vector<double> get_min_ranges();
    vector<double> get_max_ranges();
    bool end_of_episode();
    double get_reward();
    void compute_possible_actions();

    arma::Mat<double> calculate_probabilities();
    arma::Mat<double> calculate_rewards();
private:
    int m_robot_x;
    int m_robot_y;
    int m_agent_x;
    int m_agent_y;
    int m_max_x;
    int m_max_y;

    map<vector<double>, int> state_to_ind;
    void calculate_states();
    vector<vector<double> > get_neighbors(vector<double> state, int action);
};
