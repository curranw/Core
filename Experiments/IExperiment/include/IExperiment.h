#pragma once

#include <IDomain.h>
#include <ILearningAlgorithm.h>
#include <utils.h>

using namespace std;

class IExperimentArgs
{
public:
    int num_steps;
    int num_epochs;
    string save_file;
};

class IExperiment
{
public:
    IExperiment(IDomain* domain, ILearningAlgorithm* learning_algorithm, IExperimentArgs* experiment_args);
    virtual void run_experiment();
    virtual ~IExperiment();

public:
    IDomain* m_domain;
    ILearningAlgorithm* m_learning_algorithm;
    IExperimentArgs* m_exp_args;

    QUpdate update_params;

    vector<double> m_rewards;
    double tot_reward;

    vector<vector<double> > m_accumulated_data;
public:
    virtual void init();
    virtual void epoch();
    virtual void step();
    virtual void get_reward();
    virtual void output_results();
    virtual void begin_epoch();
    virtual void end_epoch();

    virtual vector<vector<double> > get_accumulated_data();

    int current_iteration;
    int cur_step;

};
