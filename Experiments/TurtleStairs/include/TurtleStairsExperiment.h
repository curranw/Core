#pragma once

#include <IExperiment.h>
#include <TurtleStairs.h>
#include <QTiles.h>
#include <QTilesReuse.h>
#include <sstream>
#include "tf/tf.h"
using namespace std;

class TurtleStairsExperimentArgs : public IExperimentArgs
{
public:
    bool demonstrations;
    TurtleStairsExperimentArgs()
    {
        demonstrations = false;
    }
};

class TurtleStairsExperiment : public IExperiment
{
public:
    TurtleStairsExperiment(TurtleStairs* domain, QTiles* learning_algorithm, TurtleStairsExperimentArgs* exp_args);
    virtual ~TurtleStairsExperiment();

    void init();
    void step();
    void end_epoch();
    void output_results();
private:

    void load_updates(string update_file);
    TurtleStairsExperimentArgs* m_exp_args;
    ILearningAlgorithm* m_learning_algorithm;
    QTiles* casted_learning_algorithm;
    TurtleStairs* m_domain;
    vector<vector<double> > good_data;
    vector<vector<double> > bad_data;
    vector<double> good_rewards;
    vector<double> bad_rewards;

    map<vector<double>, pair<double, int> > demo_updates;
    vector<double> performance;
    int num_demo_used;
    int iteration;
};
