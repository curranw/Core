#pragma once

#include <NArmExperiment.h>
#include <NArmExperimentPCA.h>
#include <QTiles.h>
#include <QTilesReuse.h>
#include <NArm.h>

using namespace std;
class IROS2015
{
public:
    void setup_experiments();

    void normal_learning(string save_file);

    void pca_learning(string pca_file, int amount, string save_file, int dim, bool iterative);
};
