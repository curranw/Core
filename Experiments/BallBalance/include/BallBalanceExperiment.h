#pragma once

#include <IExperiment.h>
#include <BallBalance.h>
#include <QTiles.h>
#include <QTilesReuse.h>
#include <sstream>
#include "tf/tf.h"
using namespace std;

class BallBalanceExperimentArgs : public IExperimentArgs
{
public:
    bool demonstrations;
    BallBalanceExperimentArgs()
    {
        demonstrations = false;
    }
};

class BallBalanceExperiment : public IExperiment
{
public:
    BallBalanceExperiment(BallBalance* domain, QTiles* learning_algorithm, BallBalanceExperimentArgs* exp_args);
    virtual ~BallBalanceExperiment();

    double potential_prev;

    void init();
    void step();
    void end_epoch();
    void output_results();
private:

    void load_updates(string update_file);
    BallBalanceExperimentArgs* m_exp_args;
    QTiles* m_learning_algorithm;
    QTiles* casted_learning_algorithm;
    BallBalance* m_domain;
    vector<vector<double> > good_data;
    vector<vector<double> > bad_data;
    vector<double> good_rewards;
    vector<double> bad_rewards;
    vector<int> table_sizes;
    map<vector<double>, pair<double, int> > demo_updates;
    vector<double> performance;
    int num_demo_used;
    int iteration;
};
