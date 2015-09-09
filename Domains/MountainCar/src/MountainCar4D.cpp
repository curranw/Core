/*
 * MountainCar4D.cpp
 *
 *  Created on: Jun 22, 2015
 *      Author: Will Curran
 */

#include "MountainCar4D.h"

MountainCar4D::MountainCar4D() : IDomain(){
    throttleFactor = 1.0;
    max_position = 0.6;
    min_position = -1.2;
    max_velocity = 0.07;
    min_velocity = -0.07;
    target_position = 0.5;
    max_height = 2;
    min_height = -2;
}

void MountainCar4D::init()
{
    double f = (double)rand() / RAND_MAX;
    position_x = min_position + f * (max_position - min_position);

    f = (double)rand() / RAND_MAX;
    position_y = min_position + f * (max_position - min_position);

    f = (double)rand() / RAND_MAX;
    position_z = min_position + f * (max_position - min_position);

    f = (double)rand() / RAND_MAX;
    velocity_x = min_velocity + f * (max_velocity - min_velocity);

    f = (double)rand() / RAND_MAX;
    velocity_y = min_velocity + f * (max_velocity - min_velocity);

    f = (double)rand() / RAND_MAX;
    velocity_z = min_velocity + f * (max_velocity - min_velocity);
    //position_x = -0.5;
    //position_y = -0.5;
    //velocity_x = 0.0;
    //velocity_y = 0.0;
    height = sin(3.0*position_x) + sin(3.0 * position_y);
}

vector<double> MountainCar4D::get_state()
{
    vector<double> state;
    state.push_back(position_x);
    state.push_back(position_y);
    state.push_back(position_z);
    state.push_back(velocity_x);
    state.push_back(velocity_y);
    state.push_back(velocity_z);
    return state;
}
void MountainCar4D::step(int action)
{
    vector<double> taken_action = m_action_mapping[action];

    velocity_x = velocity_x + taken_action[0] * 0.001 + cos(3.0 * position_x) * (-0.0025);
    velocity_y = velocity_y + taken_action[1] * 0.001 + cos(3.0 * position_y) * (-0.0025);
    velocity_z = velocity_z + taken_action[2] * 0.001 + cos(3.0 * position_z) * (-0.0025);

    if(velocity_x > max_velocity) velocity_x = max_velocity;
    if(velocity_x < min_velocity) velocity_x = min_velocity;
    if(velocity_y > max_velocity) velocity_y = max_velocity;
    if(velocity_y < min_velocity) velocity_y = min_velocity;
    if(velocity_z > max_velocity) velocity_z = max_velocity;
    if(velocity_z < min_velocity) velocity_z = min_velocity;

    position_x += velocity_x;
    position_y += velocity_y;
    position_z += velocity_z;
    if (position_x > max_position) position_x = max_position;
    if (position_x < min_position) position_x = min_position;
    if (position_y > max_position) position_y = max_position;
    if (position_y < min_position) position_y = min_position;
    if (position_z > max_position) position_z = max_position;
    if (position_z < min_position) position_z = min_position;
}

bool MountainCar4D::end_of_episode()
{
    return (position_x >= target_position && position_y >= target_position && position_z >= target_position);
}

double MountainCar4D::get_reward()
{
    bool win = end_of_episode();

    //Reward
    double reward;
    if(!win) reward = -1;
    else reward = 100;

    return reward;
}

vector<double> MountainCar4D::get_max_ranges()
{
    vector<double> max_ranges;
    max_ranges.push_back(max_position);
    max_ranges.push_back(max_position);
    max_ranges.push_back(max_position);
    max_ranges.push_back(max_velocity);
    max_ranges.push_back(max_velocity);
    max_ranges.push_back(max_velocity);
    return max_ranges;
}

vector<double> MountainCar4D::get_min_ranges()
{
    vector<double> min_ranges;
    min_ranges.push_back(min_position);
    min_ranges.push_back(min_position);
    min_ranges.push_back(min_position);
    min_ranges.push_back(min_velocity);
    min_ranges.push_back(min_velocity);
    min_ranges.push_back(min_velocity);
    return min_ranges;
}

void MountainCar4D::compute_possible_actions()
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
    possible_actions.push_back(5);
    possible_actions.push_back(6);
    vector<double> a;
    a.push_back(0);
    a.push_back(0);
    a.push_back(0);
    action_mapping.insert(make_pair(0,a));

    a.clear();
    a.push_back(1);
    a.push_back(0);
    a.push_back(0);
    action_mapping.insert(make_pair(1,a));

    a.clear();
    a.push_back(0);
    a.push_back(1);
    a.push_back(0);
    action_mapping.insert(make_pair(2,a));

    a.clear();
    a.push_back(0);
    a.push_back(0);
    a.push_back(1);
    action_mapping.insert(make_pair(3,a));

    a.clear();
    a.push_back(-1);
    a.push_back(0);
    a.push_back(0);
    action_mapping.insert(make_pair(4,a));

    a.clear();
    a.push_back(0);
    a.push_back(-1);
    a.push_back(0);
    action_mapping.insert(make_pair(5,a));

    a.clear();
    a.push_back(0);
    a.push_back(0);
    a.push_back(-1);
    action_mapping.insert(make_pair(6,a));

    set_possible_actions(possible_actions, action_mapping);
}

MountainCar4D::~MountainCar4D() {
    // TODO Auto-generated destructor stub
}
