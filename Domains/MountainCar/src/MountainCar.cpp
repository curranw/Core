/*
 * MountainCar.cpp
 *
 *  Created on: Jun 22, 2015
 *      Author: Will Curran
 */

#include "MountainCar.h"

MountainCar::MountainCar() : IDomain(){
    throttleFactor = 1.0;
    max_position = 0.6;
    min_position = -1.2;
    max_height = 1;
    min_height = -1;
    max_velocity = 0.07;
    min_velocity = -0.07;
    target_position = 0.6;
}

void MountainCar::init()
{
    double f = (double)rand() / RAND_MAX;
    position = min_position + f * (max_position - min_position);

    f = (double)rand() / RAND_MAX;
    velocity = min_velocity + f * (max_velocity - min_velocity);
}

vector<double> MountainCar::get_state()
{
    vector<double> state;
    state.push_back(position);
    //state.push_back(height);
    state.push_back(velocity);
    return state;
}
void MountainCar::step(int action)
{
    vector<double> taken_action = m_action_mapping[action];

    float throttle = taken_action[0] * (throttleFactor);
    velocity = velocity + throttle * 0.001 + cos(3.0 * position) * (-0.0025);
    if(velocity > max_velocity) velocity = max_velocity;
    if(velocity < min_velocity) velocity = min_velocity;

    position += velocity;
    if (position > max_position) position = max_position;
    if (position < min_position) position = min_position;

    height = sin(3.0*position);
}

bool MountainCar::end_of_episode()
{
    return position >= target_position;
}

vector<double> MountainCar::get_max_ranges()
{
    vector<double> max_ranges;
    max_ranges.push_back(max_position);
    //max_ranges.push_back(max_height);
    max_ranges.push_back(max_velocity);
    return max_ranges;
}

double MountainCar::get_reward()
{
    bool win = end_of_episode();

    //Reward
    double reward;
    if(!win) reward = -1;
    else reward = 100;

    return reward;
}

void MountainCar::compute_possible_actions()
{
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;
    for(int i = 0; i < 3; i++)
    {
        possible_actions.push_back(i);
        vector<double> a;
        if(i == 0) a.push_back(0.0);
        if(i == 1) a.push_back(-1.0);
        if(i == 2) a.push_back(1.0);
        action_mapping.insert(make_pair(i, a));
    }
    set_possible_actions(possible_actions, action_mapping);
}

vector<double> MountainCar::get_min_ranges()
{
    vector<double> min_ranges;
    min_ranges.push_back(min_position);
    //min_ranges.push_back(min_height);
    min_ranges.push_back(min_velocity);
    return min_ranges;
}

MountainCar::~MountainCar() {
    // TODO Auto-generated destructor stub
}
