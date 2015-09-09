#pragma once

#include <IExperiment.h>
#include <NArm.h>
#include <QTiles.h>

using namespace std;

class NArmExperimentArgs : public IExperimentArgs
{
public:
    bool demonstrations;
};

class NArmExperiment : public IExperiment
{
public:
    NArmExperiment(NArm* domain, QTiles* learning_algorithm, NArmExperimentArgs* exp_args);
    void end_epoch();
    void output_results();
    virtual ~NArmExperiment();

private:
    NArmExperimentArgs* m_exp_args;
    QTiles* m_learning_algorithm;
    vector<vector<double> > good_data;
    vector<vector<double> > bad_data;

};
