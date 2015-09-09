/*
 * MountainCar.h
 *
 *  Created on: Jun 22, 2015
 *      Author: Will Curran
 */

#pragma once

#include <math.h>
#include <vector>
#include <iostream>
#include <random>

#include <IDomain.h>
using namespace std;
class MountainCar4D : public IDomain{
public:
    MountainCar4D();
    void init();
    void step(int action);
	bool end_of_episode();
	vector<double> get_state();
    double get_reward();
    vector<double> get_max_ranges();
    vector<double> get_min_ranges();
    void compute_possible_actions();

    double position_x, position_y, position_z, velocity_x, velocity_y, velocity_z, height;
	double target_position, target_velocity;
	double throttleFactor;
    double max_velocity, min_velocity, max_position, min_position, max_height, min_height;

    virtual ~MountainCar4D();
};
