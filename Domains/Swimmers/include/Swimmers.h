#pragma once

#include <IDomain.h>
#include <CSwimmer.h>
#include <CPolicy.h>
#include <CXSwimmer.h>

class Swimmers : public IDomain, public CPolicy
{
public:
    Swimmers(int segments);
    void init();
    vector<double> get_state();
    void step(int action);
    vector<double> get_min_ranges();
    vector<double> get_max_ranges();
    bool end_of_episode();
    double get_reward();
    void compute_possible_actions();
    bool viz;

private:
    CSwimmer* swimmer;
    CXSwimmer* swimmerwin;
    vector<double> action_vec;
    void GetControl(const double *px, double *pu) const;

};
