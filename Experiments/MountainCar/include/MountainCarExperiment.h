#pragma once
#include <IExperiment.h>
#include <MountainCar.h>
#include <MountainCar3D.h>
#include <MountainCar4D.h>
#include <MountainCar3DScaled.h>
#include <QTilesReuse.h>
#include <QTiles.h>

using namespace std;

class MountainCarExperimentArgs : public IExperimentArgs
{
public:
    bool demonstrations;
};

class MountainCarExperiment : public IExperiment
{
public:
    MountainCarExperiment(MountainCar3D* domain, QTiles* learning_algorithm, MountainCarExperimentArgs* exp_args);
    MountainCarExperiment(MountainCar3DScaled* domain, QTiles* learning_algorithm, MountainCarExperimentArgs* exp_args);
    MountainCarExperiment(MountainCar4D* domain, QTiles* learning_algorithm, MountainCarExperimentArgs* exp_args);
    MountainCarExperiment(MountainCar *domain, QTiles *learning_algorithm, MountainCarExperimentArgs* exp_args);
    void end_epoch();
    void output_results();
    virtual ~MountainCarExperiment();

private:
    MountainCarExperimentArgs* m_exp_args;
    QTiles* m_learning_algorithm;
    vector<vector<double> > good_data;
    vector<vector<double> > bad_data;

};
