#include <iostream>
#include <fstream>
#include <AAAI2015.h>
#include <HRI2016.h>
#include <IROS2017.h>
#include <IROS2015.h>
#include <RobotExperiments.h>
#include <omp.h>
#include <cblas.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
using namespace std;

void run_experiment()
{
//    HRI2016* exp_hri = new HRI2016();
//    exp_hri->setup_experiments();
//    IROS2015* exp_iros = new IROS2015();
//    exp_iros->setup_experiments();

//    AAAI2015* exp_aaai = new AAAI2015();
//    exp_aaai->setup_experiments();

//    IROS2017* exp_iros = new IROS2017();
//    exp_iros->setup_experiments();

    RobotExperiments* exp_re = new RobotExperiments();
    exp_re->setup_experiments();
}

extern "C" void openblas_set_num_threads(int num_threads);

int main(int argc, char **argv)
{

    openblas_set_num_threads(1);
    run_experiment();
    exit(1);
}

