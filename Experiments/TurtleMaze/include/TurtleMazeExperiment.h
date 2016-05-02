#pragma once

#include <IExperiment.h>
#include <TurtleMaze.h>
#include <QTiles.h>
#include <QTilesReuse.h>
using namespace std;

class TurtleMazeExperimentArgs : public IExperimentArgs
{
public:
    bool demonstrations;
};

class TurtleMazeExperiment : public IExperiment
{
public:
    TurtleMazeExperiment(TurtleMaze* domain, QTiles* learning_algorithm, TurtleMazeExperimentArgs* exp_args);
    virtual ~TurtleMazeExperiment();

    double potential_prev;

private:
    TurtleMazeExperimentArgs* m_exp_args;
    ILearningAlgorithm* m_learning_algorithm;
    TurtleMaze* m_domain;
    vector<vector<double> > good_data;
    vector<vector<double> > bad_data;
    vector<double> good_rewards;
    vector<double> bad_rewards;

    double performance;
    vector<double> tot_performance;
    int iteration;
};
