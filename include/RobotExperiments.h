#pragma once
#include <BallBalanceExperiment.h>
#include <BallBalanceExperimentPCA.h>
#include <BallBalance.h>

class RobotExperiments
{
public:
    void setup_experiments();
    void normal_learning_ball_balance(string save_file);
};
