#pragma once
#include <IExperiment.h>
#include <ArmTouch.h>
#include <QTilesReuse.h>
#include <QTiles.h>

using namespace std;

class ArmTouchExperimentPCAArgs : public IExperimentArgs
{
public:
    bool demonstrations;
    bool iterative;
    int start_dimension;
};

class ArmTouchExperimentPCA : public IExperiment
{
public:
    ArmTouchExperimentPCA(ArmTouch* domain, QTilesReuse* learning_algorithm, ArmTouchExperimentPCAArgs* exp_args);
    virtual ~ArmTouchExperimentPCA();

    void init();
    void end_epoch();
    void step();
    void output_results();

private:
    ArmTouchExperimentPCAArgs* m_exp_args;
    QTilesReuse* m_learning_algorithm;
    int current_dimension;
    vector<int> action_count;
    vector<int> table_sizes;
};
