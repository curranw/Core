/*
 * MountainCar3D.cpp
 *
 *  Created on: Jun 22, 2015
 *      Author: Will Curran
 */

#include "MountainCar3D.h"

MountainCar3D::MountainCar3D() : IDomain(){
    throttleFactor = 1.0;
    max_position = 0.6;
    min_position = -1.2;
    max_velocity = 0.07;
    min_velocity = -0.07;
    target_position = 0.5;
    max_height = 2;
    min_height = -2;
}

void MountainCar3D::init()
{
    double f = (double)rand() / RAND_MAX;
    position_x = min_position + f * (max_position - min_position);

    f = (double)rand() / RAND_MAX;
    position_y = min_position + f * (max_position - min_position);

    f = (double)rand() / RAND_MAX;
    velocity_x = min_velocity + f * (max_velocity - min_velocity);

    f = (double)rand() / RAND_MAX;
    velocity_y = min_velocity + f * (max_velocity - min_velocity);

    //position_x = -0.5;
    //position_y = -0.5;
    //velocity_x = 0.0;
    //velocity_y = 0.0;
    height = sin(3.0*position_x) + sin(3.0 * position_y);
}

vector<double> MountainCar3D::get_state()
{
    vector<double> state;
    state.push_back(position_x);
    state.push_back(position_y);
    state.push_back(velocity_x);
    state.push_back(velocity_y);
    return state;
}
void MountainCar3D::step(int action)
{
    vector<double> taken_action = m_action_mapping[action];

    velocity_x = velocity_x + taken_action[0] * 0.001 + cos(3.0 * position_x) * (-0.0025);
    velocity_y = velocity_y + taken_action[1] * 0.001 + cos(3.0 * position_y) * (-0.0025);
    if(velocity_x > max_velocity) velocity_x = max_velocity;
    if(velocity_x < min_velocity) velocity_x = min_velocity;
    if(velocity_y > max_velocity) velocity_y = max_velocity;
    if(velocity_y < min_velocity) velocity_y = min_velocity;

    position_x += velocity_x;
    position_y += velocity_y;
    if (position_x > max_position) position_x = max_position;
    if (position_x < min_position) position_x = min_position;
    if (position_y > max_position) position_y = max_position;
    if (position_y < min_position) position_y = min_position;
}

bool MountainCar3D::end_of_episode()
{
    return (position_x >= target_position && position_y >= target_position);
}

double MountainCar3D::get_reward()
{
    bool win = end_of_episode();

    //Reward
    double reward;
    if(!win) reward = -1;
    else reward = 0;

    return reward;
}

vector<double> MountainCar3D::get_max_ranges()
{
    vector<double> max_ranges;
    max_ranges.push_back(max_position);
    max_ranges.push_back(max_position);
    max_ranges.push_back(max_velocity);
    max_ranges.push_back(max_velocity);
    return max_ranges;
}

vector<double> MountainCar3D::get_min_ranges()
{
    vector<double> min_ranges;
    min_ranges.push_back(min_position);
    min_ranges.push_back(min_position);
    min_ranges.push_back(min_velocity);
    min_ranges.push_back(min_velocity);
    return min_ranges;
}

void MountainCar3D::compute_possible_actions()
{
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;
//    int it = 0;
//    for(int i = -1; i <= 1; i++)
//    {
//        for(int j = -1; j <= 1; j++)
//        {
//            if(i != 0 && j != 0) continue;
//            possible_actions.push_back(it);
//            vector<double> a;
//            a.push_back(i);
//            a.push_back(j);
//            action_mapping.insert(make_pair(it++, a));
//        }
//    }

    possible_actions.push_back(0);
    possible_actions.push_back(1);
    possible_actions.push_back(2);
    possible_actions.push_back(3);
    possible_actions.push_back(4);
    vector<double> a;
    a.push_back(0);
    a.push_back(0);
    action_mapping.insert(make_pair(0,a));

    a.clear();
    a.push_back(1);
    a.push_back(0);
    action_mapping.insert(make_pair(1,a));

    a.clear();
    a.push_back(0);
    a.push_back(1);
    action_mapping.insert(make_pair(2,a));

    a.clear();
    a.push_back(-1);
    a.push_back(0);
    action_mapping.insert(make_pair(3,a));

    a.clear();
    a.push_back(0);
    a.push_back(-1);
    action_mapping.insert(make_pair(4,a));

    set_possible_actions(possible_actions, action_mapping);
}

MountainCar3D::~MountainCar3D() {
    // TODO Auto-generated destructor stub
}
