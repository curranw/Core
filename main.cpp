#include <iostream>
#include <fstream>
#include <AAAI2015.h>
#include <IROS2015.h>

#include <NPole.h>
using namespace std;

void run_experiment()
{
//    IROS2015* exp_iros = new IROS2015();
//    exp_iros->setup_experiments();

    AAAI2015* exp_aaai = new AAAI2015();
    exp_aaai->setup_experiments();
}

int main()
{
//    NPole* p = new NPole();
//    p->init();
//    p->compute_possible_actions();
//    p->step(0);
//    p->step(0);
//    int action = 0;
//    for(unsigned int i = 0; i < 1000; i++)
//    {
//        if(i % 10 == 0 && i != 0) action++;
//        if(action > 10) action = 0;
//        p->step(0);
//    }
//    exit(1);
    run_experiment();
    exit(1);
}

