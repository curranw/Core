/*
 * TurtleMaze.cpp
 *
 *  Created on: Jun 22, 2015
 *      Author: Will Curran
 */

#include "TurtleStairs.h"
#include <unistd.h>
TurtleStairs::TurtleStairs() : IDomain()
{
    vector<pair<string, string> > temp;
    ros::init(temp, "TurtleStairsController");
    ros::NodeHandle node_handler;
    odom_sub = node_handler.subscribe("/ground_truth", 100, &TurtleStairs::odom_subscriber, this);
    laser_sub = node_handler.subscribe("/scan", 200, &TurtleStairs::laser_subscriber, this);
    twist_pub = node_handler.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    ros::service::waitForService("/gazebo/set_model_state");
    set_model_client = node_handler.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::Rate poll_rate(10);
    ros::spinOnce();
    poll_rate.sleep();
    safe = true;

    //-20...-16
    //3.5
    target_position_x = -14;
    target_position_y = 3.5;

}

void TurtleStairs::init()
{
    gazebo_msgs::ModelState message;
    gazebo_msgs::SetModelState request;
    tf::Matrix3x3 m;
    double rand_yaw = (rand()/double(RAND_MAX) * (3.14159 * 2) ) - 3.14159;
    rand_yaw = 3.14159;
    cout << "Random yaw! " << rand_yaw << endl;
    m.setRPY(0,0, rand_yaw);
    tf::Quaternion q;
    m.getRotation(q);

    cout << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << endl;

    message.pose.orientation.x = q.x();
    message.pose.orientation.y = q.y();
    message.pose.orientation.z = q.z();
    message.pose.orientation.w = q.w();
    message.pose.position.x = 5.0;
    message.pose.position.y = 0.0;
    message.model_name = "mobile_base";
    message.reference_frame = "world";
    request.request.model_state = message;
    set_model_client.call(request);

    sleep(1.0);
    ros::spinOnce();
}

vector<double> TurtleStairs::get_state()
{
    ros::spinOnce();
    vector<double> state;
    state.push_back(position_x);
    state.push_back(position_y);
    state.push_back(yaw);
    ros::spinOnce();
    return state;
}
void TurtleStairs::step(int action)
{
    new_pose = false;
    ros::Rate poll_rate(10);
    while(!new_pose)
    {

        ros::spinOnce();
        vector<double> taken_action = m_action_mapping[action];
        geometry_msgs::Twist twist_action;
        twist_action.linear.x = taken_action[0];
        twist_action.linear.y = taken_action[1];
        twist_action.angular.z = taken_action[2];
        if(safe) twist_pub.publish(twist_action);
        else if(twist_action.angular.z == 0)
        {
            ros::spinOnce();
            break;
        }
        else
        {
            twist_action.linear.x = 0.0;
            twist_pub.publish(twist_action);
            ros::spinOnce();
        }
        ros::spinOnce();
        poll_rate.sleep();
    }
    new_pose = false;
    return;
}

bool TurtleStairs::end_of_episode()
{
    if(position_x > -16 && position_x < -13 && position_y > 3.5)
    {
        return true;
    }
    return false;
}

double TurtleStairs::get_reward()
{
    bool win = end_of_episode();
    if(win)
    {
        return 1000;
    }
    return -1;
    if(!safe) return -1;
    double reward_x = abs(position_x - target_position_x) * abs(position_x - target_position_x);
    double reward_y = abs(position_y - target_position_y) * abs(position_y - target_position_y);
    double dist_to_goal = sqrt(reward_x + reward_y);

    double last_reward_x = abs(last_position_x - target_position_x) * abs(last_position_x - target_position_x);
    double last_reward_y = abs(last_position_y - target_position_y) * abs(last_position_y - target_position_y);
    double last_dist_to_goal = sqrt(last_reward_x + last_reward_y);

    double reward = (last_dist_to_goal - dist_to_goal) * 10;
    //if(reward < 0) reward = 0;
    return reward;
}

void TurtleStairs::compute_possible_actions()
{
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;
    int num_dimensions = 3;
    vector<double> a;
    possible_actions.push_back(0);
    a.push_back(0.5);
    a.push_back(0);
    a.push_back(0);
    action_mapping.insert(make_pair(0, a));
    a.clear();

    possible_actions.push_back(1);
    a.push_back(0.5);
    a.push_back(0);
    a.push_back(1.0);
    action_mapping.insert(make_pair(1, a));
    a.clear();

    possible_actions.push_back(2);
    a.push_back(0.5);
    a.push_back(0);
    a.push_back(-1.0);
    action_mapping.insert(make_pair(2, a));
    a.clear();

    set_possible_actions(possible_actions, action_mapping);
}

double TurtleStairs::get_performance()
{
    return 0;
}

vector<double> TurtleStairs::get_min_ranges()
{
    vector<double> min_ranges;
    min_ranges.push_back(-17);
    min_ranges.push_back(-3);
    min_ranges.push_back(-3.14159265359);
    return min_ranges;

}

vector<double> TurtleStairs::get_max_ranges()
{
    vector<double> max_ranges;
    max_ranges.push_back(11);
    max_ranges.push_back(4);
    max_ranges.push_back(3.14159265359);
    return max_ranges;
}

void TurtleStairs::pose_subscriber(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    //cout << "Pose Update" << endl;
    new_pose = true;
    last_position_x = position_x;
    last_position_y = position_y;
    last_yaw = yaw;
    position_x = msg->pose.pose.position.x;
    position_y = msg->pose.pose.position.y;

    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);


}

void TurtleStairs::odom_subscriber(const nav_msgs::Odometry::ConstPtr &msg)
{
    new_pose = true;
    last_position_x = position_x;
    last_position_y = position_y;
    last_yaw = yaw;
    position_x = msg->pose.pose.position.x;
    position_y = msg->pose.pose.position.y;

    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

void TurtleStairs::laser_subscriber(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    safe = true;
    for(unsigned int i = 0; i < msg->ranges.size(); i++)
    {
        if(msg->ranges[i] < 0.5)
        {
            safe = false;
        }
    }
}

TurtleStairs::~TurtleStairs() {
    // TODO Auto-generated destructor stub
}
