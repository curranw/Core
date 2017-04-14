/*
 * MountainCar.h
 *
 *  Created on: Jun 22, 2015
 *      Author: Will Curran
 */

#pragma once

//Trajectory Splicing
#include <math.h>
#include <vector>
#include <iostream>
#include <random>
#include <IDomain.h>
#include "ros/ros.h"

#include "sensor_msgs/JointState.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "actionlib_msgs/GoalStatusArray.h"

#include "tf/transform_listener.h"

#include "visualization_msgs/Marker.h"

using namespace std;
class BallBalance : public IDomain{
public:
    BallBalance();
    void init();
    void step(int action);
	bool end_of_episode();
	vector<double> get_state();
    vector<double> get_min_ranges();
    vector<double> get_max_ranges();
    double get_reward();
    void compute_possible_actions();
    double get_performance();

    ros::Subscriber robot_1_subscriber;
    void single_joint_subscriber(const sensor_msgs::JointState::ConstPtr& robot_1);

    ros::Subscriber goal_status_subscriber;
    void goal_status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr &goal_status);

    //ROS Publishers
    ros::Publisher robot_1_joint_1_pub;
    ros::Publisher fake_joint_state_pub;
    ros::Publisher vis_pub;
    tf::TransformListener* plate_listener;

    bool viz;
    virtual ~BallBalance();

private:
    bool fake_joints;

    int num_joints;
    vector<string> joint_names;
    string cur_goal_id;
    bool finished_action;


    trajectory_msgs::JointTrajectoryPoint moveit_action;
    vector<double> arm_a, arm_v, arm_p;
    vector<double> arm_a_last, arm_v_last, arm_p_last;

    double y_ball_a, y_ball_v, y_ball_p;
    double x_ball_a, x_ball_v, x_ball_p;

    double t;
    void update_ball_kinematics();
    void compute_arm_moveit_action(vector<double>* action);
    void compute_moveit_accelerations(vector<double>* action);
    void compute_moveit_velocities();
    void compute_moveit_positions();

    tf::StampedTransform plate_angle;
    void compute_ball();
    void make_marker();
};

