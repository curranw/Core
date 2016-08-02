/*
 * TurtleMaze.cpp
 *
 *  Created on: Jun 22, 2015
 *      Author: Will Curran
 */

#include "TurtleMaze.h"
#include <unistd.h>
TurtleMaze::TurtleMaze() : IDomain()
{
    vector<pair<string, string> > temp;
    ros::init(temp, "TurtleMazeController");
    ros::NodeHandle node_handler;
    pose_sub = node_handler.subscribe("/amcl_pose", 100, &TurtleMaze::pose_subscriber, this);
    odom_sub = node_handler.subscribe("/ground_truth", 100, &TurtleMaze::odom_subscriber, this);
    laser_sub = node_handler.subscribe("/scan", 200, &TurtleMaze::laser_subscriber, this);
    twist_pub = node_handler.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    init_pose_pub = node_handler.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    ros::service::waitForService("/gazebo/reset_world");
    client = node_handler.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    set_model_client = node_handler.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
//    init_pose_pub_times = 0;
    target_position_x = -4.0;
    target_position_y = -6.5;
    ros::Rate poll_rate(10);
    ros::spinOnce();
    poll_rate.sleep();
    safe = true;
    make_waypoints();
}

void TurtleMaze::init()
{
    hit_waypoints.clear();

    std_srvs::Empty empty;
    //client.call(empty);

    //sleep(1.0);
    ros::spinOnce();
    geometry_msgs::PoseWithCovarianceStamped init_pose_msg;

    ros::NodeHandle node_handler;

    gazebo_msgs::SetModelState request;

    gazebo_msgs::ModelState message;
    tf::Matrix3x3 m;
    double rand_yaw = (rand()/double(RAND_MAX) * (3.14159 * 2) ) - 3.14159;
    cout << "Random yaw! " << rand_yaw << endl;
    m.setRPY(0,0, rand_yaw);
    tf::Quaternion q;
    m.getRotation(q);

    cout << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << endl;

    message.pose.orientation.x = q.x();
    message.pose.orientation.y = q.y();
    message.pose.orientation.z = q.z();
    message.pose.orientation.w = q.w();
    message.pose.position.x = 0.0;
    message.pose.position.y = 0.0;
    message.model_name = "mobile_base";
    message.reference_frame = "world";
    request.request.model_state = message;
    set_model_client.call(request);


//    init_pose_msg.header.seq = init_pose_pub_times;
//    init_pose_msg.header.frame_id = "map";
//    init_pose_msg.header.stamp = ros::Time::now();

//    init_pose_msg.pose.pose.position.x = 0.0;
//    init_pose_msg.pose.pose.position.y = 0.0;
//    init_pose_msg.pose.pose.position.z = 0.0;

//    init_pose_msg.pose.pose.orientation.x = q.x();
//    init_pose_msg.pose.pose.orientation.y = q.y();
//    init_pose_msg.pose.pose.orientation.z = q.z();
//    init_pose_msg.pose.pose.orientation.w = q.w();

//    init_pose_pub.publish(init_pose_msg);

//    init_pose_pub_times++;
    sleep(1.0);
    ros::spinOnce();
}

vector<double> TurtleMaze::get_state()
{
    ros::spinOnce();
    vector<double> state;
    state.push_back(position_x);
    state.push_back(position_y);
    state.push_back(yaw);
    ros::spinOnce();
    return state;
}
void TurtleMaze::step(int action)
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

bool TurtleMaze::end_of_episode()
{
    if(hit_waypoints.find(3) != hit_waypoints.end()) return true;
    return false;
}

double TurtleMaze::get_reward()
{
    check_waypoints();
    bool win = end_of_episode();
    if(win)
    {
        return 100;
    }
    if(!safe) return -10;
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

void TurtleMaze::compute_possible_actions()
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

double TurtleMaze::get_performance()
{
    return hit_waypoints.size();
}

vector<double> TurtleMaze::get_min_ranges()
{
    vector<double> min_ranges;
    min_ranges.push_back(-7);
    min_ranges.push_back(-8);
    min_ranges.push_back(-3.14159265359);
    return min_ranges;
}

vector<double> TurtleMaze::get_max_ranges()
{
    vector<double> max_ranges;
    max_ranges.push_back(8.2);
    max_ranges.push_back(7);
    max_ranges.push_back(3.14159265359);
    return max_ranges;
}

void TurtleMaze::pose_subscriber(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
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

void TurtleMaze::odom_subscriber(const nav_msgs::Odometry::ConstPtr &msg)
{
//    static int num_calls = 0;
//    num_calls++;
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
//    if(num_calls > 30)
//    {
//        cout << "Pose Update" << endl;
//        num_calls = 0;
//        new_pose = true;
//    }
}

void TurtleMaze::laser_subscriber(const sensor_msgs::LaserScan::ConstPtr &msg)
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

TurtleMaze::~TurtleMaze() {
    // TODO Auto-generated destructor stub
}


void TurtleMaze::check_waypoints()
{
    for(unsigned int i = 0; i < waypoints.size(); i++)
    {
        if(hit_waypoints.find(i) != hit_waypoints.end()) continue;
        square cur_waypoint = waypoints[i];
        bool good = true;
        for(int j = 0; j < 4; j++)
        {
            corner cur_corner = cur_waypoint[j];
            if(j == 0 && (position_x < cur_corner.first || position_y < cur_corner.second)) good = false;
            if(j == 1 && (position_x > cur_corner.first || position_y < cur_corner.second)) good = false;
            if(j == 2 && (position_x < cur_corner.first || position_y > cur_corner.second)) good = false;
            if(j == 3 && (position_x > cur_corner.first || position_y > cur_corner.second)) good = false;
            if(!good) break;
        }
        if(good)
        {
            hit_waypoints.insert(i);
        }
    }
}

void TurtleMaze::make_waypoints()
{
    waypoints.push_back(make_square(make_corner(4.7, -3.4), make_corner(8.2, -3.4), make_corner(4.7, -1.0), make_corner(8.2, -1)));
    waypoints.push_back(make_square(make_corner(4.7, 4.0),  make_corner(8.2,  4.0), make_corner(4.7,  7.0), make_corner(8.2, 7.0)));
    waypoints.push_back(make_square(make_corner(-6.5, 4.0), make_corner(-2.5, 4.0), make_corner(-6.5, 7.0), make_corner(-2.5, 7.0)));
    waypoints.push_back(make_square(make_corner(-6.5, -7.0), make_corner(-2.5, -7.0), make_corner(-6.5, -5.0), make_corner(-2.5, -5.0)));
}

pair<double, double> TurtleMaze::make_corner(double x, double y)
{
    pair<double, double> corner;
    corner.first = x;
    corner.second = y;
    return corner;
}

TurtleMaze::square TurtleMaze::make_square(corner bottom_left, corner bottom_right, corner top_left, corner top_right)
{
    square waypoint;
    waypoint.push_back(bottom_left);
    waypoint.push_back(bottom_right);
    waypoint.push_back(top_left);
    waypoint.push_back(top_right);
    return waypoint;
}
