#pragma once

#include <IExperiment.h>
#include <Swimmers.h>
#include <ValueIteration.h>
#include <RMax.h>
#include <FittedRMax.h>
#include <armadillo>
#include <QTiles.h>
#include <QTilesReuse.h>
using namespace std;

class SwimmersExperimentArgs : public IExperimentArgs
{
public:
    bool demonstrations;
};

class SwimmersExperiment : public IExperiment
{
public:
    SwimmersExperiment(Swimmers* domain, ValueIteration* learning_algorithm, SwimmersExperimentArgs* exp_args);
    SwimmersExperiment(Swimmers* domain, RMax* learning_algorithm, SwimmersExperimentArgs* exp_args);
    SwimmersExperiment(Swimmers* domain, FittedRMax* learning_algorithm, SwimmersExperimentArgs* exp_args);
    SwimmersExperiment(Swimmers* domain, QTiles* learning_algorithm, SwimmersExperimentArgs* exp_args);
    void end_epoch();
    void output_results();
    void step();
    void init();
    virtual ~SwimmersExperiment();

    double potential_prev;

private:
    SwimmersExperimentArgs* m_exp_args;
    ILearningAlgorithm* m_learning_algorithm;
    Swimmers* m_domain;
    vector<vector<double> > good_data;
    vector<vector<double> > bad_data;
    vector<double> good_rewards;
    vector<double> bad_rewards;

    double performance;
    vector<double> tot_performance;
    int iteration;
};
