#pragma once
#include <IExperiment.h>
#include <NArm.h>
#include <QTilesReuse.h>
#include <QTiles.h>

using namespace std;

class NArmExperimentPCAArgs : public IExperimentArgs
{
public:
    bool demonstrations;
    bool iterative;
    int start_dimension;
};

class NArmExperimentPCA : public IExperiment
{
public:
    NArmExperimentPCA(NArm* domain, QTilesReuse* learning_algorithm, NArmExperimentPCAArgs* exp_args);
    virtual ~NArmExperimentPCA();

    void init();
    void end_epoch();

private:
    NArmExperimentPCAArgs* m_exp_args;
    QTilesReuse* m_learning_algorithm;
    int current_dimension;
};
