/*
 * TurtleMaze.cpp
 *
 *  Created on: Jun 22, 2015
 *      Author: Will Curran
 */

#include "TurtleMaze.h"

TurtleMaze::TurtleMaze() : IDomain(){

}

void TurtleMaze::init()
{

}

vector<double> TurtleMaze::get_state()
{
    vector<double> state;

    return state;
}
void TurtleMaze::step(int action)
{
    vector<double> taken_action = m_action_mapping[action];

}

bool TurtleMaze::end_of_episode()
{
    return position >= target_position;
}

vector<double> TurtleMaze::get_max_ranges()
{
    vector<double> max_ranges;

    return max_ranges;
}

double TurtleMaze::get_reward()
{
    bool win = end_of_episode();

    //Reward
    double reward;
    if(!win) reward = -1;
    else reward = 0;

    return reward;
}

void TurtleMaze::compute_possible_actions()
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

vector<double> TurtleMaze::get_min_ranges()
{
    vector<double> min_ranges;

    return min_ranges;
}

TurtleMaze::~TurtleMaze() {
    // TODO Auto-generated destructor stub
}
