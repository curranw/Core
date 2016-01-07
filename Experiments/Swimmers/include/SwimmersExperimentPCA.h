#pragma once
#include <IExperiment.h>
#include <Swimmers.h>
#include <QTilesReuse.h>
#include <QTiles.h>

using namespace std;

class SwimmersExperimentPCAArgs : public IExperimentArgs
{
public:
    bool demonstrations;
    bool iterative;
    int start_dimension;
};

class SwimmersExperimentPCA : public IExperiment
{
public:
    SwimmersExperimentPCA(Swimmers* domain, QTilesReuse* learning_algorithm, SwimmersExperimentPCAArgs* exp_args);
    virtual ~SwimmersExperimentPCA();
    void step();
    void init();
    void end_epoch();
    void output_results();
    Swimmers* m_domain;

private:
    SwimmersExperimentPCAArgs* m_exp_args;
    QTilesReuse* m_learning_algorithm;
    int current_dimension;
    double performance;
    int iteration;
    vector<int> table_sizes;
};
