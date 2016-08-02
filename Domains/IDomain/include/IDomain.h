#pragma once

#include <vector>
#include <map>
using namespace std;

class IDomain
{
public:
    IDomain();
    virtual void init() = 0;
    virtual vector<double> get_state() = 0;
    virtual void step(int action) = 0;
    virtual vector<double> get_min_ranges() = 0;
    virtual vector<double> get_max_ranges() = 0;
    virtual bool end_of_episode() = 0;
    virtual double get_reward() = 0;
    virtual void compute_possible_actions() = 0;
    virtual void get_possible_actions(vector<int> &possible_actions, map<int, vector<double> > &action_mapping);
    virtual void set_possible_actions(vector<int> possible_actions, map<int, vector<double> > action_mapping);
    virtual double get_performance();
    bool m_accumulate_data;

protected:
    vector<int> m_possible_actions;
    map<int, vector<double> > m_action_mapping;
    vector<vector<double> > data;
};
