#pragma once

#include <math.h>
#include <vector>
#include <iostream>
#include <random>
#include <armadillo>
#include <cmath>

#include <IDomain.h>

using namespace std;
class NPole: public IDomain{
public:
    NPole();
    void init();
    void step(int action);
    bool end_of_episode();
    vector<double> get_state();
    vector<double> get_min_ranges();
    vector<double> get_max_ranges();
    double get_reward();
    void compute_possible_actions();

    void calculate_accel(vector<double> action);
    void calculate_pos(bool output);

    double sind(double degrees);
    double cosd(double degrees);

    double get_performance(double steps_left);
    void calculate_performance();

    int num_links;
    vector<double> mass;
    vector<double> length;

    arma::Mat<double> theta;
    arma::Mat<double> theta_dot;

    vector<double> link_pos_x;
    vector<double> link_pos_y;

    double dt;
    double theta_max;
    double theta_min;
    double vel_limit;
    double len_limit;
    int num_iterations;
    double cum_r;

    vector<vector<double> > all_x;
    vector<vector<double> > all_y;
    virtual ~NPole();
};
