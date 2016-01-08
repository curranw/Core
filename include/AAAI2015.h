#pragma once

#include <MountainCarExperiment.h>
#include <MountainCarExperimentPCA.h>
#include <NPoleExperiment.h>
#include <NPoleExperimentPCA.h>
#include <BlackjackExperiment.h>
#include <BlackjackExperimentPCA.h>
#include <QTiles.h>
#include <QTilesReuse.h>
#include <FittedRMax.h>
#include <MountainCar3D.h>
#include <MountainCar3DScaled.h>
#include <MountainCar4D.h>
#include <NPole.h>
#include <Blackjack.h>
using namespace std;
class AAAI2015
{
public:
    void setup_experiments();

    void normal_learning(string save_file);
    void normal_learning4d(string save_file);
    void normal_learning_scaled(string save_file);
    void normal_learning_fitted_rmax(string save_file);
    void normal_learning4d_fitted_rmax(string save_file);

    void pca_learning(string pca_file, int amount, string save_file, int dim, bool iterative);
    void pca_learning_4d(string pca_file, int amount, string save_file);
    void pca_learning_scaled(string pca_file, int amount, string save_file);


    void normal_learning_pole(string save_file);
    void pca_learning_pole(string pca_file, int amount, string save_file, int dim, bool iterative);

    void normal_learning_blackjack(string save_file);
    void pca_learning_blackjack(string pca_file, int amount, string save_file, int dim, bool iterative);
};
