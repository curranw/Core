#pragma once
#include <IExperiment.h>
#include <BallBalance.h>
#include <QTilesReuse.h>
#include <QTiles.h>

using namespace std;

class BallBalanceExperimentPCAArgs : public IExperimentArgs
{
public:
    bool demonstrations;
    bool iterative;
    int start_dimension;
};

class BallBalanceExperimentPCA : public IExperiment
{
public:
    BallBalanceExperimentPCA(BallBalance* domain, QTilesReuse* learning_algorithm, BallBalanceExperimentPCAArgs* exp_args);
    virtual ~BallBalanceExperimentPCA();

    void init();
    void end_epoch();
    void step();
    void output_results();

private:
    BallBalanceExperimentPCAArgs* m_exp_args;
    QTilesReuse* m_learning_algorithm;
    int current_dimension;
    vector<int> action_count;
    vector<int> table_sizes;
};
