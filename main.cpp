#include <iostream>
#include <fstream>
#include <AAAI2015.h>
#include <IROS2015.h>
#include <HRI2016.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

//#include <tapkee/tapkee.hpp>
//#include <tapkee/callbacks/dummy_callbacks.hpp>

using namespace std;

void run_experiment()
{
    HRI2016* exp_hri = new HRI2016();
//    IROS2015* exp_iros = new IROS2015();
//    exp_iros->setup_experiments();

//    AAAI2015* exp_aaai = new AAAI2015();
//    exp_aaai->setup_experiments();
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    cout << msg->data.c_str() << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/chatter", 1000, chatterCallback);
    ros::spin();
    //srand(0);
//    NPole* p = new NPole();
//    p->init();
//    p->compute_possible_actions();
//    int action = 0;
//    p->step(0);
//    p->step(0);
//    p->step(0);
//    for(unsigned int i = 0; i < 10000; i++)
//    {
//        if(i % 10 == 0 && i != 0) action++;
//        if(action > 10) action = 0;
//        p->step(1);
//    }
//    exit(1);
//    const int N = 100;
//    vector<IndexType> indices(N);
//    for (int i=0; i<N; i++) indices[i] = i;

//    MyDistanceCallback d;

//    TapkeeOutput output = tapkee::initialize()
//       .withParameters((method=MultidimensionalScaling,target_dimension=1))
//       .withDistance(d)
//       .embedUsing(indices);

//    cout << output.embedding.transpose() << endl;
//    return 0;
    run_experiment();
    exit(1);
}

