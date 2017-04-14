/*
 * BallBalance.cpp
 *
 *  Created on: Jun 22, 2015
 *      Author: Will Curran
 */

#include "BallBalance.h"
#include <unistd.h>
BallBalance::BallBalance() : IDomain()
{
    fake_joints = true;
    num_joints = 6;
    t = 0.01;
    vector<pair<string, string> > temp;
    ros::init(temp, "BallBalanceController");
    ros::NodeHandle node_handler;

    robot_1_joint_1_pub = node_handler.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/joint_trajectory_action/goal", 1);
    fake_joint_state_pub = node_handler.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);
    vis_pub = node_handler.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    robot_1_subscriber = node_handler.subscribe("/joint_states", 1, &BallBalance::single_joint_subscriber, this);
    goal_status_subscriber = node_handler.subscribe("/joint_trajectory_action/status", 1, &BallBalance::goal_status_cb, this);

    moveit_action.accelerations.assign(num_joints,0);
    moveit_action.velocities.assign(num_joints,0);
    moveit_action.positions.assign(num_joints,0);

    plate_listener = new tf::TransformListener();
    ros::Rate poll_rate(10);
    ros::spinOnce();
    poll_rate.sleep();
    viz = false;
}

void BallBalance::init()
{
    moveit_action.accelerations.assign(num_joints,0);
    moveit_action.velocities.assign(num_joints,0);
    moveit_action.positions.assign(num_joints,0);
    arm_p.assign(6, 0);
    arm_v.assign(6, 0);
    arm_a.assign(6, 0);

    if(joint_names.empty())
    {
        //        ['joint_s', 'joint_l', 'joint_u', 'joint_r', 'joint_b', 'joint_t']
        joint_names.push_back("joint_s");
        joint_names.push_back("joint_l");
        joint_names.push_back("joint_u");
        joint_names.push_back("joint_r");
        joint_names.push_back("joint_b");
        joint_names.push_back("joint_t");
    }


    double f = (double)rand()/ RAND_MAX;
    f = 0 + f * (0.1) - 0.05;
    y_ball_a = 0;
    y_ball_v = 0;
    y_ball_p = f;
    x_ball_a = 0;
    x_ball_v = 0;
    f = (double)rand()/ RAND_MAX;
    f = 0 + f * (0.1) - 0.05;
    x_ball_p = f;

    if(fake_joints)
    {
        ros::Rate poll_rate(10);
        arm_a = moveit_action.accelerations;
        arm_v = moveit_action.velocities;
        arm_p = moveit_action.positions;
        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        js.name = joint_names;
        js.position = arm_p;
        js.velocity = arm_v;
        fake_joint_state_pub.publish(js);
        ros::spinOnce();
        poll_rate.sleep();
    }
    else
    {
        control_msgs::FollowJointTrajectoryActionGoal goal;
        goal.goal_id.stamp = ros::Time::now();

        string action_num_str = std::to_string(0);
        goal.goal_id.id = action_num_str;
        cur_goal_id = action_num_str;

        trajectory_msgs::JointTrajectory traj;
        traj.header.frame_id = "/world_frame";
        traj.header.stamp = ros::Time::now();
        traj.joint_names = joint_names;
        traj.points.push_back(moveit_action);
        goal.goal.trajectory = traj;

        robot_1_joint_1_pub.publish(goal);
    }

    compute_ball();
    ros::spinOnce();

    finished_action = true;
    return;

}

vector<double> BallBalance::get_state()
{
    vector<double> state;
//    state.push_back(arm_p[0]);
    state.push_back(arm_p[1]);
    state.push_back(arm_p[2]);
    state.push_back(arm_p[3]);
    state.push_back(arm_p[4]);
    state.push_back(arm_p[5]);
    state.push_back(x_ball_p);
    state.push_back(y_ball_p);
    state.push_back(x_ball_v);
    state.push_back(y_ball_v);

    cout << x_ball_v << "," << y_ball_v << endl;
    return state;
}
void BallBalance::step(int action)
{
//    viz = true;
//            while(true)
//            {
//                ros::Rate poll_rate(60);
//                compute_ball();
//                ros::spinOnce();
//                poll_rate.sleep();
//            }
    if(fake_joints)
    {
        if(viz)
        {
            ros::Rate poll_rate(60);
            vector<double> cur_action = m_action_mapping[action];
            compute_arm_moveit_action(&cur_action);
            arm_a = moveit_action.accelerations;
            arm_v = moveit_action.velocities;
            arm_p = moveit_action.positions;
            sensor_msgs::JointState js;
            js.header.stamp = ros::Time::now();
            js.name = joint_names;
            js.position = arm_p;
            js.velocity = arm_v;
            fake_joint_state_pub.publish(js);
            compute_ball();
            ros::spinOnce();
            poll_rate.sleep();
        }
        else
        {
            ros::Rate poll_rate(10000000000);
            vector<double> cur_action = m_action_mapping[action];
            compute_arm_moveit_action(&cur_action);
            arm_a = moveit_action.accelerations;
            arm_v = moveit_action.velocities;
            arm_p = moveit_action.positions;
            sensor_msgs::JointState js;
            js.header.stamp = ros::Time::now();
            js.name = joint_names;
            js.position = arm_p;
            js.velocity = arm_v;
            fake_joint_state_pub.publish(js);
            compute_ball();
            ros::spinOnce();
            poll_rate.sleep();
        }
    }
    else
    {
        if(!finished_action) return;
        static int action_num = 0;


        vector<double> cur_action = m_action_mapping[action];
        compute_arm_moveit_action(&cur_action);
        control_msgs::FollowJointTrajectoryActionGoal goal;
        goal.goal_id.stamp = ros::Time::now();

        string action_num_str = std::to_string(action_num++);
        goal.goal_id.id = action_num_str;
        cur_goal_id = action_num_str;

        trajectory_msgs::JointTrajectory traj;
        traj.header.frame_id = "/world_frame";
        traj.header.stamp = ros::Time::now();
        traj.joint_names = joint_names;
        traj.points.push_back(moveit_action);
        goal.goal.trajectory = traj;

        robot_1_joint_1_pub.publish(goal);

        finished_action = false;

        ros::spinOnce();

        while(!finished_action)
        {
            ros::spinOnce();
            continue;
        }
    }
    return;
}

bool BallBalance::end_of_episode()
{
    if(x_ball_p > 0.25/2.0 || y_ball_p > 0.25/2.0 || x_ball_p < -0.25/2.0 || y_ball_p < -0.25/2.0 ) return true;
    return false;

    //    double right_reward_x = abs(robot1_end_x - right_x) * abs(robot1_end_x - right_x);
    //    double right_reward_y = abs(robot1_end_y - right_y) * abs(robot1_end_y - right_y);
    //    double right_reward_z = abs(robot1_end_z - right_z) * abs(robot1_end_z - right_z);
    //    double right_reward = sqrt(right_reward_x + right_reward_y + right_reward_z);

    //    double left_reward_x = abs(robot2_end_x - left_x) * abs(robot2_end_x - left_x);
    //    double left_reward_y = abs(robot2_end_y - left_y) * abs(robot2_end_y - left_y);
    //    double left_reward_z = abs(robot2_end_z - left_z) * abs(robot2_end_z - left_z);
    //    double left_reward = sqrt(left_reward_x + left_reward_y + left_reward_z);

    //    last_dist = dist;
    //    dist = right_reward + left_reward;


    //    cout << "Distance to goal: " << dist << endl;
    return false;
}

double BallBalance::get_reward()
{
    bool lose = end_of_episode();
    if(lose)
    {
        return -1000;
    }
    return 1;
}

void BallBalance::compute_possible_actions()
{
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;
    vector<double> a;

    int vsize = 5;
    double val = 0.5;
    for (int i=0; i<vsize*2; i++){
        possible_actions.push_back(i);
        a = std::vector<double>(vsize,0);
        a[i/2] = val*(pow(-1.0, i));

        action_mapping.insert(make_pair(i, a));
        a.clear();
    }
    possible_actions.push_back(vsize*2);
    a.assign(vsize,0);
    action_mapping.insert(make_pair(vsize*2, a));
    set_possible_actions(possible_actions, action_mapping);
}

double BallBalance::get_performance()
{
    //    return hit_waypoints.size();
}

vector<double> BallBalance::get_min_ranges()
{
    vector<double> min_ranges;
//    min_ranges.push_back(-2.5);
    min_ranges.push_back(-0.5);
    min_ranges.push_back(-4);
    min_ranges.push_back(-2.5);
    min_ranges.push_back(-0.1);
    min_ranges.push_back(-2.5);
    min_ranges.push_back(-0.125);
    min_ranges.push_back(-0.125);
    return min_ranges;
}

vector<double> BallBalance::get_max_ranges()
{
    vector<double> max_ranges;
//    max_ranges.push_back(1);
    max_ranges.push_back(1.5);
    max_ranges.push_back(1);
    max_ranges.push_back(0.1);
    max_ranges.push_back(7);
    max_ranges.push_back(0.1);
    max_ranges.push_back(0.125);
    max_ranges.push_back(0.125);

    return max_ranges;
}

void BallBalance::single_joint_subscriber(const sensor_msgs::JointState::ConstPtr& robot_1)
{
    if(fake_joints)
    {
//        arm_p = robot_1->position;
        return;
        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        js.name = joint_names;
        js.position = arm_p;
        js.velocity = arm_v;
        fake_joint_state_pub.publish(js);
    }
    else
    {
        static ros::Time last_time;
        static ros::Time cur_time;

        arm_a_last = arm_a;
        arm_v_last = arm_v;
        arm_p_last = arm_p;
        //['joint_s', 'joint_l', 'joint_u', 'joint_r', 'joint_b', 'joint_t']

        //if(restarting) return;
        static bool first_call = true;
        static vector<int> locations(6);
        if(first_call)
        {
            arm_p.assign(6, 0);
            arm_v.assign(6, 0);
            arm_a.assign(6, 0);
            first_call = false;
            for(unsigned int i = 0; i < robot_1->name.size(); i++)
            {
                if(robot_1->name[i].compare("joint_s") == 0) locations[0] = i;
                if(robot_1->name[i].compare("joint_l") == 0) locations[1] = i;
                if(robot_1->name[i].compare("joint_u") == 0) locations[2] = i;
                if(robot_1->name[i].compare("joint_r") == 0) locations[3] = i;
                if(robot_1->name[i].compare("joint_b") == 0) locations[4] = i;
                if(robot_1->name[i].compare("joint_t") == 0) locations[5] = i;
            }
        }
        arm_p[0] = robot_1->position.at(locations[0]);
        arm_p[1] = robot_1->position.at(locations[1]);
        arm_p[2] = robot_1->position.at(locations[2]);
        arm_p[3] = robot_1->position.at(locations[3]);
        arm_p[4] = robot_1->position.at(locations[4]);
        arm_p[5] = robot_1->position.at(locations[5]);

        cur_time = robot_1->header.stamp;
        if(last_time.isZero())
        {
            last_time = cur_time;
            return;
        }
        ros::Duration time_diff = cur_time - last_time;
        double diff = time_diff.toSec();

        arm_v[0] = (arm_p[0] - arm_p_last[0])/diff;
        arm_v[1] = (arm_p[1] - arm_p_last[1])/diff;
        arm_v[2] = (arm_p[2] - arm_p_last[2])/diff;
        arm_v[3] = (arm_p[3] - arm_p_last[3])/diff;
        arm_v[4] = (arm_p[4] - arm_p_last[4])/diff;
        arm_v[5] = (arm_p[5] - arm_p_last[5])/diff;

        arm_a[0] = (arm_v[0] - arm_v_last[0])/diff;
        arm_a[1] = (arm_v[1] - arm_v_last[1])/diff;
        arm_a[2] = (arm_v[2] - arm_v_last[2])/diff;
        arm_a[3] = (arm_v[3] - arm_v_last[3])/diff;
        arm_a[4] = (arm_v[4] - arm_v_last[4])/diff;
        arm_a[5] = (arm_v[5] - arm_v_last[5])/diff;

        last_time = cur_time;
    }
}

void BallBalance::goal_status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr& goal_status)
{
    for(unsigned int i = 0; i < goal_status->status_list.size(); i++)
    {
        if(goal_status->status_list.at(i).goal_id.id.compare(cur_goal_id) == 0 && goal_status->status_list.at(i).status == goal_status->status_list.at(i).SUCCEEDED)
        {
            finished_action = true;
        }
    }
}


BallBalance::~BallBalance() {

}

void BallBalance::compute_arm_moveit_action(vector<double>* action)
{
    compute_moveit_accelerations(action);
    compute_moveit_velocities();
    compute_moveit_positions();
}

void BallBalance::compute_moveit_accelerations(vector<double>* action)
{
    moveit_action.accelerations[0] = 0;
    for(unsigned int i = 0; i < num_joints-1; i++)
    {
        moveit_action.accelerations[i+1] = action->at(i);
    }
}

void BallBalance::compute_moveit_velocities()
{
    for(unsigned int i = 0; i < num_joints; i++)
    {
        moveit_action.velocities[i] = arm_v[i] + (moveit_action.accelerations[i] * t);
    }
}

void BallBalance::compute_moveit_positions()
{
    for(unsigned int i = 0; i < num_joints; i++)
    {
        moveit_action.positions[i] = arm_p[i] + arm_v[i] * t + 0.5 * (moveit_action.accelerations[i] * t * t);
    }
}

void BallBalance::compute_ball()
{
//    try{
//        plate_listener->lookupTransform("/world_frame", "/plate_body",
//                                        ros::Time(0), plate_angle);
//    }
//    catch(tf::TransformException ex)
//    {
//        ROS_ERROR("%s",ex.what());
//    }

    double roll, pitch, yaw;
//    tf::Matrix3x3(plate_angle.getRotation()).getRPY(roll, pitch, yaw);

    //pitch: joint_t joint_r
    double roll_2 = arm_p[5];
    roll_2 += arm_p[3];

    double pitch_2 = arm_p[2];
    pitch_2 += arm_p[4];
    pitch_2 += -arm_p[1];

    roll = roll_2;
    pitch = pitch_2;

    double y_prev_v = y_ball_v;
    double  x_prev_v = x_ball_v;

    double y_friction = 9.8 * cos(roll);
    double x_friction = 9.8 * cos(pitch);

    y_ball_a = 9.8 * sin(roll);
    y_ball_v = y_prev_v + y_ball_a * t;
    y_ball_p = y_ball_p + y_prev_v * t + 0.5 * y_ball_a * t * t;

    x_ball_a = 9.8 * sin(pitch);
    x_ball_v = x_prev_v + x_ball_a * t;
    x_ball_p = x_ball_p + x_prev_v * t + 0.5 * x_ball_a * t * t;

    make_marker();
}

void BallBalance::make_marker()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "plate_body";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = -x_ball_p;
    marker.pose.position.y = y_ball_p;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub.publish( marker );
}
