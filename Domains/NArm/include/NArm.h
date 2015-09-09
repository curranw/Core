#pragma once

#include <cmath>
#include <vector>
#include <iostream>
#include <IDomain.h>

using namespace std;
class NArm : public IDomain{
public:
    NArm();
    void init();
    void step(int action);
    bool end_of_episode();
    vector<double> get_state();
    vector<double> get_min_ranges();
    vector<double> get_max_ranges();
    double get_reward();
    void compute_possible_actions();

    virtual ~NArm();

private:

    void calculate_pos();
    int m_num_links;
    double target_x, target_y;

    vector<double> m_links_angles;
    vector<double> m_links_lengths;
    vector<double> m_link_pos_x, m_link_pos_y;

};
