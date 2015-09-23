#pragma once
#include <IExperiment.h>
#include <NPole.h>
#include <QTilesReuse.h>
#include <QTiles.h>

using namespace std;

class NPoleExperimentPCAArgs : public IExperimentArgs
{
public:
    bool demonstrations;
    bool iterative;
    int start_dimension;
};

class NPoleExperimentPCA : public IExperiment
{
public:
    NPoleExperimentPCA(NPole* domain, QTilesReuse* learning_algorithm, NPoleExperimentPCAArgs* exp_args);
    virtual ~NPoleExperimentPCA();
    void step();
    void init();
    void end_epoch();
    NPole* m_domain;

private:
    NPoleExperimentPCAArgs* m_exp_args;
    QTilesReuse* m_learning_algorithm;
    int current_dimension;
    double performance;
};
