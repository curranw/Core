#pragma once

#include <IExperiment.h>
#include <TurtleMaze.h>
#include <QTiles.h>
#include <QTilesReuse.h>
#include <sstream>
#include "tf/tf.h"
using namespace std;

class TurtleMazeExperimentArgs : public IExperimentArgs
{
public:
    bool demonstrations;
    TurtleMazeExperimentArgs()
    {
        demonstrations = false;
    }
};

class TurtleMazeExperiment : public IExperiment
{
public:
    TurtleMazeExperiment(TurtleMaze* domain, QTiles* learning_algorithm, TurtleMazeExperimentArgs* exp_args);
    virtual ~TurtleMazeExperiment();

    double potential_prev;

    void init();
    void step();
    void end_epoch();
    void output_results();
private:

    void load_updates(string update_file);
    TurtleMazeExperimentArgs* m_exp_args;
    ILearningAlgorithm* m_learning_algorithm;
    QTiles* casted_learning_algorithm;
    TurtleMaze* m_domain;
    vector<vector<double> > good_data;
    vector<vector<double> > bad_data;
    vector<double> good_rewards;
    vector<double> bad_rewards;

    map<vector<double>, pair<double, int> > demo_updates;
    vector<double> performance;
    int num_demo_used;
    int iteration;
};
