#pragma once

#include <IExperiment.h>
#include <NPole.h>
#include <QTiles.h>

using namespace std;

class NPoleExperimentArgs : public IExperimentArgs
{
public:
    bool demonstrations;
};

class NPoleExperiment : public IExperiment
{
public:
    NPoleExperiment(NPole* domain, QTiles* learning_algorithm, NPoleExperimentArgs* exp_args);
    void step();
    void end_epoch();
    void output_results();
    virtual ~NPoleExperiment();

private:
    NPoleExperimentArgs* m_exp_args;
    QTiles* m_learning_algorithm;
    NPole* m_domain;
    vector<vector<double> > good_data;
    vector<vector<double> > bad_data;

    double performance;
    vector<double> tot_performance;

};
