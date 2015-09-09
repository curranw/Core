#pragma once
#include <IExperiment.h>
#include <MountainCar.h>
#include <MountainCar3D.h>
#include <MountainCar3DScaled.h>
#include <MountainCar4D.h>
#include <QTilesReuse.h>
#include <QTiles.h>

using namespace std;

class MountainCarExperimentPCAArgs : public IExperimentArgs
{
public:
    bool demonstrations;
    bool iterative;
    int start_dimension;
};

class MountainCarExperimentPCA : public IExperiment
{
public:
    MountainCarExperimentPCA(MountainCar3D* domain, QTilesReuse* learning_algorithm, MountainCarExperimentPCAArgs* exp_args);
    MountainCarExperimentPCA(MountainCar3DScaled* domain, QTilesReuse* learning_algorithm, MountainCarExperimentPCAArgs* exp_args);
    MountainCarExperimentPCA(MountainCar4D* domain, QTilesReuse* learning_algorithm, MountainCarExperimentPCAArgs* exp_args);
    virtual ~MountainCarExperimentPCA();

    void init();
    void end_epoch();

private:
    MountainCarExperimentPCAArgs* m_exp_args;
    QTilesReuse* m_learning_algorithm;
    int current_dimension;
};
