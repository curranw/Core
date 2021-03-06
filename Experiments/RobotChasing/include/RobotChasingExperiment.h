#pragma once

#include <IExperiment.h>
#include <RobotChasing.h>
#include <ValueIteration.h>
#include <RMax.h>
#include <FittedRMax.h>
#include <armadillo>

using namespace std;

class RobotChasingExperimentArgs : public IExperimentArgs
{
public:
    bool demonstrations;
};

class RobotChasingExperiment : public IExperiment
{
public:
    RobotChasingExperiment(RobotChasing* domain, ValueIteration* learning_algorithm, RobotChasingExperimentArgs* exp_args);
    RobotChasingExperiment(RobotChasing* domain, RMax* learning_algorithm, RobotChasingExperimentArgs* exp_args);
    RobotChasingExperiment(RobotChasing* domain, FittedRMax* learning_algorithm, RobotChasingExperimentArgs* exp_args);
    void end_epoch();
    void output_results();
    void init();
    virtual ~RobotChasingExperiment();

private:
    RobotChasingExperimentArgs* m_exp_args;
    ILearningAlgorithm* m_learning_algorithm;
    RobotChasing* m_domain;
    vector<vector<double> > good_data;
    vector<vector<double> > bad_data;

    double performance;
    vector<double> tot_performance;

};
