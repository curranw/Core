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
#include "nav_msgs/Odometry.h"

using namespace std;
class TurtleMaze : public IDomain{
public:
    TurtleMaze();
    void init();
    void step(int action);
	bool end_of_episode();
	vector<double> get_state();
    vector<double> get_min_ranges();
    vector<double> get_max_ranges();
    double get_reward();
    void compute_possible_actions();
    double get_performance();
    typedef pair<double, double> corner;
    typedef vector<corner> square;

    //ROS Subscribers
    ros::Subscriber pose_sub;
    void pose_subscriber(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    ros::Subscriber odom_sub;
    void odom_subscriber(const nav_msgs::Odometry::ConstPtr& msg);
    ros::Subscriber laser_sub;
    void laser_subscriber(const sensor_msgs::LaserScan::ConstPtr& msg);

    //ROS Publishers
    ros::Publisher twist_pub;
    ros::Publisher init_pose_pub, string_pub;

    //ROS Services
    ros::ServiceClient client;
    ros::ServiceClient set_model_client;
    int init_pose_pub_times;
    virtual ~TurtleMaze();

private:
    void make_waypoints();
    void check_waypoints();
    pair<double, double> make_corner(double x, double y);
    square make_square(corner bottom_left, corner bottom_right, corner top_left, corner top_right);
    set<int> hit_waypoints;
    double position_x, position_y, last_position_x, last_position_y, last_yaw, roll, pitch, yaw;
    double target_position_x, target_position_y;
	double throttleFactor;
    double max_velocity, min_velocity, max_position, min_position, max_height, min_height;

    bool new_pose;
    bool safe;

    vector<square> waypoints;

};

