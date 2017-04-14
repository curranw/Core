#pragma once
#include <ArmTouchExperiment.h>
#include <ArmTouchExperimentPCA.h>
#include <ArmTouch.h>

class IROS2017
{
public:
    void setup_experiments();
    void normal_learning_arm_touch(string save_file);
    void normal_learning_dual_arm_touch(string save_file);
    void pca_learning(string pca_file, int amount, string save_file, int dim, bool iterative);
    void pca_learning_dual(string pca_file, int amount, string save_file, int dim, bool iterative);
};
