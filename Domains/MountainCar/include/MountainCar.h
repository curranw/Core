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
class MountainCar : public IDomain{
public:
	MountainCar();
    void init();
    void step(int action);
	bool end_of_episode();
	vector<double> get_state();
    vector<double> get_min_ranges();
    vector<double> get_max_ranges();
    double get_reward();
    void compute_possible_actions();

    double position, velocity, height;
	double target_position, target_velocity;
	double throttleFactor;
    double max_velocity, min_velocity, max_position, min_position, max_height, min_height;

	virtual ~MountainCar();
};
