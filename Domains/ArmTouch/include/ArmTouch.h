/*
 * MountainCar.h
 *
 *  Created on: Jun 22, 2015
 *      Author: Will Curran
 */

#pragma once

#include <math.h>
#include <vector>
#include <iostream>
#include <random>
#include <IDomain.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "std_srvs/Empty.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/GetLinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/GetJointProperties.h"
#include "gazebo_msgs/JointRequest.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <joint_limits_interface/joint_limits_interface.h>
#include "geometry_msgs/Wrench.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

using namespace std;
class ArmTouch : public IDomain{
public:
    ArmTouch();
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::JointState> MySyncPolicy;
    void init();
    void step(int action);
	bool end_of_episode();
	vector<double> get_state();
    vector<double> get_min_ranges();
    vector<double> get_max_ranges();
    double get_reward();
    void compute_possible_actions();
    double get_performance();

    ros::Subscriber joint_sub;
    //message_filters::Subscriber<sensor_msgs::JointState>* robot_1_subscriber;
    ros::Subscriber robot_1_subscriber;
    void single_joint_subscriber(const sensor_msgs::JointState::ConstPtr& robot_1);
    void joint_subscriber(const sensor_msgs::JointState::ConstPtr& robot_1, const sensor_msgs::JointState::ConstPtr& robot_2);
    message_filters::Synchronizer<MySyncPolicy>* sync;
    //ROS Publishers
    ros::Publisher robot_1_joint_1_pub, robot_1_joint_2_pub, robot_1_joint_3_pub, robot_1_joint_4_pub, robot_1_joint_5_pub, robot_1_joint_6_pub, robot_1_joint_7_pub;
    ros::Publisher robot_2_joint_1_pub, robot_2_joint_2_pub, robot_2_joint_3_pub, robot_2_joint_4_pub, robot_2_joint_5_pub, robot_2_joint_6_pub, robot_2_joint_7_pub;

    ros::Publisher robot_1_wrench_pub;
    ros::Publisher robot_2_wrench_pub;
    //ROS Services
    ros::ServiceClient client;

    ros::ServiceClient clear_joint_client, get_link_client;

    int init_pose_pub_times;

    virtual ~ArmTouch();

private:
    bool new_pose, restarting;
    double dist;
    double last_dist;
    double robot1_joint1, robot1_joint2, robot1_joint3, robot1_joint4, robot1_joint5, robot1_joint6, robot1_end_x, robot1_end_y, robot1_end_z;
    double robot2_joint1, robot2_joint2, robot2_joint3, robot2_joint4, robot2_joint5, robot2_joint6, robot2_end_x, robot2_end_y, robot2_end_z;
    double robot1_vjoint1, robot1_vjoint2, robot1_vjoint3, robot1_vjoint4, robot1_vjoint5, robot1_vjoint6;
    double robot2_vjoint1, robot2_vjoint2, robot2_vjoint3, robot2_vjoint4, robot2_vjoint5, robot2_vjoint6;


    bool both_arms;
    double left_x, left_y, left_z, right_x, right_y, right_z;
    double wrap(double rad);
    double wrap(double rad, double min, double max);

    geometry_msgs::Wrench l_w_action;
    geometry_msgs::Wrench r_w_action;
};

