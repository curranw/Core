#pragma once

#include <NArmExperiment.h>
#include <NArmExperimentPCA.h>
#include <QTiles.h>
#include <QTilesReuse.h>
#include <NArm.h>
#include <ValueIteration.h>
#include <RMax.h>
#include <RobotChasing.h>
#include <RobotChasingExperiment.h>
#include <SwimmersExperiment.h>
#include <SwimmersExperimentPCA.h>
#include <Swimmers.h>

using namespace std;
class IROS2015
{
public:
    void setup_experiments();

    void normal_learning(string save_file);
    void normal_learning_swimmers(string save_file);
    void pca_learning_swimmers(string pca_file, int amount, string save_file, int dim, bool iterative);
    void normal_learning_rmax(string save_file);
    void normal_learning_fitted_rmax(string save_file);
    void pca_learning(string pca_file, int amount, string save_file, int dim, bool iterative);
};
