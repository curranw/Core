#pragma once

#include <vector>
#include <sklearn_interface/BuildDimensionalityReduction.h>
#include <sklearn_interface/DimensionalityReduction.h>
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>

using namespace std;

class PCAInterfaceROS
{
public:
    PCAInterfaceROS();
    virtual ~PCAInterfaceROS();

    void set_projection_dimension(int projection_dimension);
    vector<double> transform_down(vector<double> *data);

    void test();
    bool use_pca;
    bool use_kpca;

    ros::NodeHandle* node_handler;
    ros::ServiceClient bdr_client;
    ros::ServiceClient dr_client;

    int m_projection_dimension;


private:

};
