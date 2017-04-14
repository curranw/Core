/*
 * ArmTouch.cpp
 *
 *  Created on: Jun 22, 2015
 *      Author: Will Curran
 */

#include "ArmTouch.h"
#include <unistd.h>
ArmTouch::ArmTouch() : IDomain()
{
    both_arms = true;
    restarting = true;
    vector<pair<string, string> > temp;
    ros::init(temp, "ArmTouchController");
    ros::NodeHandle node_handler;
    ros::service::waitForService("/gazebo/reset_simulation");
    //    client = node_handler.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    //get_link_client = node_handler.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
    client = node_handler.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    get_link_client = node_handler.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

    robot_1_joint_1_pub = node_handler.advertise<std_msgs::Float64>("/r_elbow_flex_effort_controller/command", 1);
    robot_1_joint_2_pub = node_handler.advertise<std_msgs::Float64>("/r_forearm_roll_effort_controller/command", 1);
    robot_1_joint_3_pub = node_handler.advertise<std_msgs::Float64>("/r_shoulder_lift_effort_controller/command", 1);
    robot_1_joint_4_pub = node_handler.advertise<std_msgs::Float64>("/r_shoulder_pan_effort_controller/command", 1);
    robot_1_joint_5_pub = node_handler.advertise<std_msgs::Float64>("/r_upper_arm_roll_effort_controller/command", 1);
    robot_1_joint_6_pub = node_handler.advertise<std_msgs::Float64>("/r_wrist_flex_effort_controller/command", 1);
    robot_1_joint_7_pub = node_handler.advertise<std_msgs::Float64>("/r_wrist_roll_effort_controller/command", 1);

    robot_1_wrench_pub = node_handler.advertise<geometry_msgs::Wrench>("/r_arm_wrench_controller/command", 1);
    if(both_arms)
    {
        robot_2_joint_1_pub = node_handler.advertise<std_msgs::Float64>("/l_elbow_flex_effort_controller/command", 1);
        robot_2_joint_2_pub = node_handler.advertise<std_msgs::Float64>("/l_forearm_roll_effort_controller/command", 1);
        robot_2_joint_3_pub = node_handler.advertise<std_msgs::Float64>("/l_shoulder_lift_effort_controller/command", 1);
        robot_2_joint_4_pub = node_handler.advertise<std_msgs::Float64>("/l_shoulder_pan_effort_controller/command", 1);
        robot_2_joint_5_pub = node_handler.advertise<std_msgs::Float64>("/l_upper_arm_roll_effort_controller/command", 1);
        robot_2_joint_6_pub = node_handler.advertise<std_msgs::Float64>("/l_wrist_flex_effort_controller/command", 1);
        robot_2_joint_7_pub = node_handler.advertise<std_msgs::Float64>("/l_wrist_roll_effort_controller/command", 1);

        robot_2_wrench_pub = node_handler.advertise<geometry_msgs::Wrench>("/l_arm_wrench_controller/command", 1);
    }
    robot_1_subscriber = node_handler.subscribe("/joint_states", 1, &ArmTouch::single_joint_subscriber, this);
    //robot_1_subscriber = new message_filters::Subscriber<sensor_msgs::JointState>(node_handler, "/joint_states", 10);
    //    robot_2_subscriber = new message_filters::Subscriber<sensor_msgs::JointState>(node_handler, "/robot2/joint_states", 10);

    //robot_1_subscriber->registerCallback(boost::bind(&ArmTouch::single_joint_subscriber, this, _1));
    //    sync = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(10), *robot_1_subscriber, *robot_2_subscriber);
    //message_filters::TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(robot_1_subscriber, robot_2_subscriber, 10);
    //    sync->registerCallback(boost::bind(&ArmTouch::joint_subscriber, this, _1, _2));

    clear_joint_client = node_handler.serviceClient<gazebo_msgs::JointRequest>("/gazebo/clear_joint_forces");


    ros::Rate poll_rate(10);
    ros::spinOnce();
    poll_rate.sleep();


//    //Goal Left:  x:0.4817 y:-0.0420 z:0.9212
//    left_x = 0.4817;
//    left_y = -0.0420;
//    left_z = 0.9212;
//    //Goal Right: x:0.5075 y:0.0848  z:0.5777
//    right_x = 0.5075;
//    right_y = 0.0848;
//    right_z = 0.5777;
}

void ArmTouch::init()
{
    sleep(1);
    ros::spinOnce();
    restarting = true;
    //    const char* arr[] = {"fl_caster_rotation_joint", "fl_caster_l_wheel_joint", "fl_caster_r_wheel_joint", "fr_caster_rotation_joint", "fr_caster_l_wheel_joint", "fr_caster_r_wheel_joint", "bl_caster_rotation_joint", "bl_caster_l_wheel_joint", "bl_caster_r_wheel_joint", "br_caster_rotation_joint", "br_caster_l_wheel_joint", "br_caster_r_wheel_joint", "torso_lift_joint", "torso_lift_motor_screw_joint", "head_pan_joint", "head_tilt_joint", "laser_tilt_mount_joint", "r_upper_arm_roll_joint", "r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_forearm_roll_joint", "r_elbow_flex_joint", "r_wrist_flex_joint", "r_wrist_roll_joint", "r_gripper_joint", "r_gripper_l_finger_joint", "r_gripper_r_finger_joint", "r_gripper_r_finger_tip_joint", "r_gripper_l_finger_tip_joint", "r_gripper_motor_screw_joint", "r_gripper_motor_slider_joint", "l_upper_arm_roll_joint", "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_forearm_roll_joint", "l_elbow_flex_joint", "l_wrist_flex_joint", "l_wrist_roll_joint", "l_gripper_joint", "l_gripper_l_finger_joint", "l_gripper_r_finger_joint", "l_gripper_r_finger_tip_joint", "l_gripper_l_finger_tip_joint", "l_gripper_motor_screw_joint", "l_gripper_motor_slider_joint"};
    //    std::vector<std::string> v(std::begin(arr), std::end(arr));

    //    for(unsigned int i = 0; i < v.size(); i++)
    //    {
    //        gazebo_msgs::JointRequest jr;
    //        jr.request.joint_name = v[i];
    //        clear_joint_client.call(jr);
    //    }


    if(both_arms)
    {
        //'r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint',
        //-0.0024134939368511965, -0.5571742804764863, -0.23220095232029703, -1.4390766631126422, -0.5811811260364728, -0.3868730011833703, 1.845092121935732,
        // 'l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint',
        //-0.0017436347261892138, 0.7517435834368005, 0.5399681217731197, 0.6196114395733243, -0.5887259319310516, -0.9851493134001208, -0.9595504304212801,

        std_msgs::Float64 default_action;

        default_action.data = 0;

        robot_1_joint_1_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_2_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_3_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_4_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_5_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_6_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_7_pub.publish(default_action);ros::spinOnce();

        robot_2_joint_1_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_2_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_3_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_4_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_5_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_6_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_7_pub.publish(default_action);ros::spinOnce();

        cout << "Starting" << endl;
        gazebo_msgs::SetModelConfiguration smc;
        smc.request.model_name = "pr2";
        smc.request.urdf_param_name = "robot_description";

        double rand_yaw;
        smc.request.joint_names.push_back("r_shoulder_pan_joint");
        smc.request.joint_names.push_back("r_shoulder_lift_joint");
        smc.request.joint_names.push_back("r_upper_arm_roll_joint");
        smc.request.joint_names.push_back("r_elbow_flex_joint");
        smc.request.joint_names.push_back("r_forearm_roll_joint");
        smc.request.joint_names.push_back("r_wrist_flex_joint");
        smc.request.joint_names.push_back("r_wrist_roll_joint");
        rand_yaw = (rand()/double(RAND_MAX) * (-1.5));
        smc.request.joint_positions.push_back(rand_yaw);

        rand_yaw = (rand()/double(RAND_MAX));
        smc.request.joint_positions.push_back(rand_yaw);

        smc.request.joint_positions.push_back(-1);

        rand_yaw = (-rand()/double(RAND_MAX));
        smc.request.joint_positions.push_back(rand_yaw);

        smc.request.joint_positions.push_back(-1.439);
        smc.request.joint_positions.push_back(-0.38930);
        smc.request.joint_positions.push_back(0);


        //        smc.request.joint_positions.push_back(-0.5571);
        //        smc.request.joint_positions.push_back(-0.2322);
        //        smc.request.joint_positions.push_back(-1);
        //        smc.request.joint_positions.push_back(-0.5811);
        //        smc.request.joint_positions.push_back(-1.439);
        //        smc.request.joint_positions.push_back(-0.38930);
        //        smc.request.joint_positions.push_back(0);



        smc.request.joint_names.push_back("l_shoulder_pan_joint");
        smc.request.joint_names.push_back("l_shoulder_lift_joint");
        smc.request.joint_names.push_back("l_upper_arm_roll_joint");
        smc.request.joint_names.push_back("l_elbow_flex_joint");
        smc.request.joint_names.push_back("l_forearm_roll_joint");
        smc.request.joint_names.push_back("l_wrist_flex_joint");
        smc.request.joint_names.push_back("l_wrist_roll_joint");

        rand_yaw = (rand()/double(RAND_MAX) * (1.5));
        smc.request.joint_positions.push_back(rand_yaw);

        rand_yaw = (rand()/double(RAND_MAX));
        smc.request.joint_positions.push_back(rand_yaw);

        smc.request.joint_positions.push_back(1);

        rand_yaw = (-rand()/double(RAND_MAX));
        smc.request.joint_positions.push_back(rand_yaw);

        smc.request.joint_positions.push_back(0.6196);
        smc.request.joint_positions.push_back(-0.3868);
        smc.request.joint_positions.push_back(0);


//        smc.request.joint_positions.push_back(0.5571);
//        smc.request.joint_positions.push_back(0.2322);
//        smc.request.joint_positions.push_back(1);
//        smc.request.joint_positions.push_back(-0.5887);
//        smc.request.joint_positions.push_back(0.6196);
//        smc.request.joint_positions.push_back(-0.3868);
//        smc.request.joint_positions.push_back(0);

        client.call(smc);
        sleep(1);

        robot_1_joint_1_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_2_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_3_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_4_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_5_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_6_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_7_pub.publish(default_action);ros::spinOnce();

        robot_2_joint_1_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_2_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_3_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_4_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_5_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_6_pub.publish(default_action);ros::spinOnce();
        robot_2_joint_7_pub.publish(default_action);ros::spinOnce();

        gazebo_msgs::GetLinkState gls;
        gls.request.link_name = "pr2::r_gripper_l_finger_tip_link";
        gls.request.reference_frame = "world";
        get_link_client.call(gls);
        if(!gls.response.success)
        {
            return;
        }
        robot1_end_x = gls.response.link_state.pose.position.x;
        robot1_end_y = gls.response.link_state.pose.position.y;
        robot1_end_z = gls.response.link_state.pose.position.z;


        gls.request.link_name = "pr2::l_gripper_l_finger_tip_link";
        gls.request.reference_frame = "world";
        get_link_client.call(gls);
        if(!gls.response.success)
        {
            return;
        }
        robot2_end_x = gls.response.link_state.pose.position.x;
        robot2_end_y = gls.response.link_state.pose.position.y;
        robot2_end_z = gls.response.link_state.pose.position.z;

        end_of_episode();
    }
    else
    {
        std_msgs::Float64 default_action;

        default_action.data = 0;

        robot_1_joint_1_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_2_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_3_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_4_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_5_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_6_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_7_pub.publish(default_action);ros::spinOnce();



        //'r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'
        //-1.004271311723702, -0.7906559642899138, -0.14037313024943465, 2.503580037754025, -0.35856359444032204, -0.38930867131256974, -0.8147091141622207

        cout << "Starting" << endl;
        gazebo_msgs::SetModelConfiguration smc;
        smc.request.model_name = "pr2";
        smc.request.urdf_param_name = "robot_description";


        smc.request.joint_names.push_back("r_shoulder_pan_joint");
        smc.request.joint_names.push_back("r_shoulder_lift_joint");
        smc.request.joint_names.push_back("r_upper_arm_roll_joint");
        smc.request.joint_names.push_back("r_elbow_flex_joint");
        smc.request.joint_names.push_back("r_forearm_roll_joint");
        smc.request.joint_names.push_back("r_wrist_flex_joint");
        smc.request.joint_names.push_back("r_wrist_roll_joint");

        //double rand_yaw = (rand()/double(RAND_MAX) * (1.570795 * 2) ) - 1.570795;
        double rand_yaw = (rand()/double(RAND_MAX) * (-2));
        //smc.request.joint_positions.push_back(rand_yaw);
        smc.request.joint_positions.push_back(-0.7906);
        rand_yaw = (rand()/double(RAND_MAX));
        //smc.request.joint_positions.push_back(rand_yaw);
        smc.request.joint_positions.push_back(-0.1403);
        rand_yaw = (rand()/double(RAND_MAX) * (-2));
        //smc.request.joint_positions.push_back(-1);
        smc.request.joint_positions.push_back(-1.0042);
        rand_yaw = (rand()/double(RAND_MAX) * -2);
        //smc.request.joint_positions.push_back(rand_yaw);
        smc.request.joint_positions.push_back(-0.3893);
        rand_yaw = (rand()/double(RAND_MAX) * (-1));
        //smc.request.joint_positions.push_back(-0.5);
        smc.request.joint_positions.push_back(2.5035);
        rand_yaw = (rand()/double(RAND_MAX) * (0.5 * 2) - 0.5);
        //smc.request.joint_positions.push_back(0);
        smc.request.joint_positions.push_back(-0.38930);
        rand_yaw = (rand()/double(RAND_MAX) * (-0.5 * 2));
        //smc.request.joint_positions.push_back(0);
        smc.request.joint_positions.push_back(-0.8147);
        client.call(smc);
        sleep(1);

        robot_1_joint_1_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_2_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_3_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_4_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_5_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_6_pub.publish(default_action);ros::spinOnce();
        robot_1_joint_7_pub.publish(default_action);ros::spinOnce();

    }
    restarting = false;

    new_pose = false;
    return;

}

vector<double> ArmTouch::get_state()
{
    vector<double> state;
    state.push_back(robot1_joint1);
    state.push_back(robot1_joint2);
    state.push_back(robot1_joint3);
    state.push_back(robot1_joint4);
    state.push_back(robot1_joint5);
    state.push_back(robot1_joint6);

    if(both_arms)
    {
        state.push_back(robot2_joint1);
        state.push_back(robot2_joint2);
        state.push_back(robot2_joint3);
        state.push_back(robot2_joint4);
        state.push_back(robot2_joint5);
        state.push_back(robot2_joint6);
    }
    return state;
}
void ArmTouch::step(int action)
{
    new_pose = false;
    ros::Rate poll_rate(50);
    //while(!new_pose)
    //{
        ros::spinOnce();
        vector<double> cur_action = m_action_mapping[action];
        r_w_action.force.x = cur_action[0];
        r_w_action.force.x = cur_action[1];
        r_w_action.force.x = cur_action[2];
        r_w_action.torque.x = cur_action[3];
        r_w_action.torque.x = cur_action[4];
        r_w_action.torque.x = cur_action[5];
        l_w_action.force.x = cur_action[6];
        l_w_action.force.x = cur_action[7];
        l_w_action.force.x = cur_action[8];
        l_w_action.torque.x = cur_action[9];
        l_w_action.torque.x = cur_action[10];
        l_w_action.torque.x = cur_action[11];
        robot_1_wrench_pub.publish(r_w_action);
        robot_2_wrench_pub.publish(l_w_action);
        poll_rate.sleep();
    //}
    while(!new_pose);
        //Effort Control
//        ros::spinOnce();
//        vector<double> cur_action = m_action_mapping[action];
//        std_msgs::Float64 robot_1_joint_1_action, robot_1_joint_2_action, robot_1_joint_3_action, robot_1_joint_4_action, robot_1_joint_5_action, robot_1_joint_6_action;
//        robot_1_joint_1_action.data = cur_action[0];
//        robot_1_joint_2_action.data = cur_action[1];
//        robot_1_joint_3_action.data = cur_action[2];
//        robot_1_joint_4_action.data = cur_action[3];
//        robot_1_joint_5_action.data = cur_action[4];
//        robot_1_joint_6_action.data = cur_action[5];

//        robot_1_joint_1_pub.publish(robot_1_joint_1_action);ros::spinOnce();
//        robot_1_joint_2_pub.publish(robot_1_joint_2_action);ros::spinOnce();
//        robot_1_joint_3_pub.publish(robot_1_joint_3_action);ros::spinOnce();
//        robot_1_joint_4_pub.publish(robot_1_joint_4_action);ros::spinOnce();
//        robot_1_joint_5_pub.publish(robot_1_joint_5_action);ros::spinOnce();
//        robot_1_joint_6_pub.publish(robot_1_joint_6_action);ros::spinOnce();

//        if(both_arms)
//        {
//            std_msgs::Float64 robot_2_joint_1_action, robot_2_joint_2_action, robot_2_joint_3_action, robot_2_joint_4_action, robot_2_joint_5_action, robot_2_joint_6_action;
//            robot_2_joint_1_action.data = cur_action[6];
//            robot_2_joint_2_action.data = cur_action[7];
//            robot_2_joint_3_action.data = cur_action[8];
//            robot_2_joint_4_action.data = cur_action[9];
//            robot_2_joint_5_action.data = cur_action[10];
//            robot_2_joint_6_action.data = cur_action[11];

//            robot_2_joint_1_pub.publish(robot_2_joint_1_action);ros::spinOnce();
//            robot_2_joint_2_pub.publish(robot_2_joint_2_action);ros::spinOnce();
//            robot_2_joint_3_pub.publish(robot_2_joint_3_action);ros::spinOnce();
//            robot_2_joint_4_pub.publish(robot_2_joint_4_action);ros::spinOnce();
//            robot_2_joint_5_pub.publish(robot_2_joint_5_action);ros::spinOnce();
//            robot_2_joint_6_pub.publish(robot_2_joint_6_action);ros::spinOnce();
//        }
//        poll_rate.sleep();
//    }
    ros::spinOnce();
    new_pose = false;
    return;
}

bool ArmTouch::end_of_episode()
{
        double reward_x = abs(robot1_end_x - robot2_end_x) * abs(robot1_end_x - robot2_end_x);
        double reward_y = abs(robot1_end_y - robot2_end_y) * abs(robot1_end_y - robot2_end_y);
        double reward_z = abs(robot1_end_z - robot2_end_z) * abs(robot1_end_z - robot2_end_z);
        last_dist = dist;
        dist = sqrt(reward_x + reward_y + reward_z);

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
    if(dist < 0.3) return true;
    return false;
}

double ArmTouch::get_reward()
{

    //'r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint',
    //'l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint',

    //-0.002376602139412043, 0.3961695624463202, 1.1060724063671143, -1.9054031436752554, -1.7288093523763806, -1.3699729978625435, -0.14905790416662335
    //2.005022865828421, 0.5138452736070978, -0.35342075614510726, -1.1016266634310066, -1.696285830382239, -0.8418950530472733, 1.6313729372928831
    bool win = end_of_episode();
    if(win)
    {
        return 100;
    }

    double reward = (last_dist - dist) * 10;
    //double reward = 1.0/(dist * dist);
    //double reward = -1;
    //    cout << reward << endl;
    return reward;
}

void ArmTouch::compute_possible_actions()
{

    //Wrench
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;
    vector<double> a;

    int vsize;
    if(both_arms)
    {
        vsize = 12;
        //    for (int i=0; i<vsize*2; i++){
        for (int i=0; i<vsize*2; i++){
            double val = 0.5;

            if(i <= 5) val = 2.0;
            if(i > 5 && i <= 11) val = 0.1;
            if(i > 11 && i <= 17) val = 2.0;
            if(i > 17 && i <=23) val = 0.1;

            possible_actions.push_back(i);
            a = std::vector<double>(vsize,0);
            a[i/2] = val*(pow(-1.0, i));

            action_mapping.insert(make_pair(i, a));
            a.clear();
        }
    }
    possible_actions.push_back(vsize*2);
    a.assign(vsize,0);
    action_mapping.insert(make_pair(vsize*2, a));
    set_possible_actions(possible_actions, action_mapping);

    //Effort
    /*
    vector<int> possible_actions;
    map<int, vector<double> > action_mapping;
    vector<double> a;

    int vsize;
    if(both_arms)
    {
        vsize = 12;
        //    for (int i=0; i<vsize*2; i++){
        for (int i=0; i<vsize*2; i++){
            double val = 0.5;

            if(i == 2 || i == 3) val = 0.1;
            if(i == 4 || i == 5) val = 2.0;
            if(i == 6 || i == 7) val = 2.0;
            if(i == 8 || i == 9) val = 0.1;
            if(i == 10 || i == 11) val = 0.1;

            if(i == 14 || i == 15) val = 0.1;
            if(i == 16 || i == 17) val = 2.0;
            if(i == 18 || i == 19) val = 2.0;
            if(i == 20 || i == 21) val = 0.1;
            if(i == 22 || i == 23) val = 0.1;

            possible_actions.push_back(i);
            a = std::vector<double>(vsize,0);
            a[i/2] = val*(pow(-1.0, i));

            action_mapping.insert(make_pair(i, a));
            a.clear();
        }
    }
    else
    {
        vsize = 6;
        //    for (int i=0; i<vsize*2; i++){
        for (int i=0; i<vsize*2; i++){
            double val = 0.5;

            if(i == 2 || i == 3) val = 0.1;
            if(i == 4 || i == 5) val = 2.0;
            if(i == 6 || i == 7) val = 2.0;
            if(i == 8 || i == 9) val = 0.1;
            if(i == 10 || i == 11) val = 0.1;

            possible_actions.push_back(i);
            a = std::vector<double>(vsize,0);
            a[i/2] = val*(pow(-1.0, i));

            action_mapping.insert(make_pair(i, a));
            a.clear();
        }
    }
    possible_actions.push_back(vsize*2);
    a.assign(vsize,0);
    action_mapping.insert(make_pair(vsize*2, a));
    set_possible_actions(possible_actions, action_mapping);
    */
}

double ArmTouch::get_performance()
{
    //    return hit_waypoints.size();
}

vector<double> ArmTouch::get_min_ranges()
{
    vector<double> min_ranges;
    min_ranges.push_back(-2.5);
    min_ranges.push_back(-0.5);
    min_ranges.push_back(-4);
    min_ranges.push_back(-2.5);
    min_ranges.push_back(-0.1);
    min_ranges.push_back(-2.5);

    if(both_arms)
    {
        min_ranges.push_back(-1);
        min_ranges.push_back(-0.5);
        min_ranges.push_back(-1);
        min_ranges.push_back(-2.5);
        min_ranges.push_back(-0.1);
        min_ranges.push_back(-2.5);
    }

    return min_ranges;
}

vector<double> ArmTouch::get_max_ranges()
{
    vector<double> max_ranges;
    max_ranges.push_back(1);
    max_ranges.push_back(1.5);
    max_ranges.push_back(1);
    max_ranges.push_back(0.1);
    max_ranges.push_back(7);
    max_ranges.push_back(0.1);

    if(both_arms)
    {
        max_ranges.push_back(2.5);
        max_ranges.push_back(1.5);
        max_ranges.push_back(4);
        max_ranges.push_back(0.1);
        max_ranges.push_back(7);
        max_ranges.push_back(0.1);
    }
    return max_ranges;
}

void ArmTouch::single_joint_subscriber(const sensor_msgs::JointState::ConstPtr& robot_1)
{
    //if(restarting) return;
    static bool first_call = true;
    static vector<int> locations(12);
    if(first_call)
    {
        first_call = false;
        for(unsigned int i = 0; i < robot_1->name.size(); i++)
        {
            if(robot_1->name[i].compare("r_shoulder_pan_joint") == 0) locations[0] = i;
            if(robot_1->name[i].compare("r_shoulder_lift_joint") == 0) locations[1] = i;
            if(robot_1->name[i].compare("r_upper_arm_roll_joint") == 0) locations[2] = i;
            if(robot_1->name[i].compare("r_elbow_flex_joint") == 0) locations[3] = i;
            if(robot_1->name[i].compare("r_forearm_roll_joint") == 0) locations[4] = i;
            if(robot_1->name[i].compare("r_wrist_flex_joint") == 0) locations[5] = i;
            if(robot_1->name[i].compare("l_shoulder_pan_joint") == 0) locations[6] = i;
            if(robot_1->name[i].compare("l_shoulder_lift_joint") == 0) locations[7] = i;
            if(robot_1->name[i].compare("l_upper_arm_roll_joint") == 0) locations[8] = i;
            if(robot_1->name[i].compare("l_elbow_flex_joint") == 0) locations[9] = i;
            if(robot_1->name[i].compare("l_forearm_roll_joint") == 0) locations[10] = i;
            if(robot_1->name[i].compare("l_wrist_flex_joint") == 0) locations[11] = i;
        }
    }
    //        static double robot1_joint1_min = 0, robot1_joint1_max = 0, robot1_joint2_min = 0, robot1_joint2_max = 0, robot1_joint3_min = 0, robot1_joint3_max = 0, robot1_joint4_min = 0, robot1_joint4_max = 0, robot1_joint5_min = 0, robot1_joint5_max = 0, robot1_joint6_min = 0, robot1_joint6_max = 0;
    //        static double robot1_vjoint1_min = 0, robot1_vjoint1_max = 0, robot1_vjoint2_min = 0, robot1_vjoint2_max = 0, robot1_vjoint3_min = 0, robot1_vjoint3_max = 0, robot1_vjoint4_min = 0, robot1_vjoint4_max = 0, robot1_vjoint5_min = 0, robot1_vjoint5_max = 0, robot1_vjoint6_min = 0, robot1_vjoint6_max = 0;

    robot1_joint1 = robot_1->position.at(locations[0]);
    robot1_joint2 = robot_1->position.at(locations[1]);
    robot1_joint3 = robot_1->position.at(locations[2]);
    robot1_joint4 = robot_1->position.at(locations[3]);
    robot1_joint5 = robot_1->position.at(locations[4]);
    robot1_joint6 = robot_1->position.at(locations[5]);

    robot1_vjoint1 = robot_1->velocity.at(locations[0]);
    robot1_vjoint2 = robot_1->velocity.at(locations[1]);
    robot1_vjoint3 = robot_1->velocity.at(locations[2]);
    robot1_vjoint4 = robot_1->velocity.at(locations[3]);
    robot1_vjoint5 = robot_1->velocity.at(locations[4]);
    robot1_vjoint6 = robot_1->velocity.at(locations[5]);

    robot1_joint5 = wrap(robot1_joint5);

    //        if(robot1_joint1 < robot1_joint1_min) robot1_joint1_min = robot1_joint1;
    //        if(robot1_joint1 > robot1_joint1_max) robot1_joint1_max = robot1_joint1;
    //        if(robot1_joint2 < robot1_joint2_min) robot1_joint2_min = robot1_joint2;
    //        if(robot1_joint2 > robot1_joint2_max) robot1_joint2_max = robot1_joint2;
    //        if(robot1_joint3 < robot1_joint3_min) robot1_joint3_min = robot1_joint3;
    //        if(robot1_joint3 > robot1_joint3_max) robot1_joint3_max = robot1_joint3;
    //        if(robot1_joint4 < robot1_joint4_min) robot1_joint4_min = robot1_joint4;
    //        if(robot1_joint4 > robot1_joint4_max) robot1_joint4_max = robot1_joint4;
    //        if(robot1_joint5 < robot1_joint5_min) robot1_joint5_min = robot1_joint5;
    //        if(robot1_joint5 > robot1_joint5_max) robot1_joint5_max = robot1_joint5;
    //        if(robot1_joint6 < robot1_joint6_min) robot1_joint6_min = robot1_joint6;
    //        if(robot1_joint6 > robot1_joint6_max) robot1_joint6_max = robot1_joint6;

    //        if(robot1_vjoint1 < robot1_vjoint1_min) robot1_vjoint1_min = robot1_vjoint1;
    //        if(robot1_vjoint1 > robot1_vjoint1_max) robot1_vjoint1_max = robot1_vjoint1;
    //        if(robot1_vjoint2 < robot1_vjoint2_min) robot1_vjoint2_min = robot1_vjoint2;
    //        if(robot1_vjoint2 > robot1_vjoint2_max) robot1_vjoint2_max = robot1_vjoint2;
    //        if(robot1_vjoint3 < robot1_vjoint3_min) robot1_vjoint3_min = robot1_vjoint3;
    //        if(robot1_vjoint3 > robot1_vjoint3_max) robot1_vjoint3_max = robot1_vjoint3;
    //        if(robot1_vjoint4 < robot1_vjoint4_min) robot1_vjoint4_min = robot1_vjoint4;
    //        if(robot1_vjoint4 > robot1_vjoint4_max) robot1_vjoint4_max = robot1_vjoint4;
    //        if(robot1_vjoint5 < robot1_vjoint5_min) robot1_vjoint5_min = robot1_vjoint5;
    //        if(robot1_vjoint5 > robot1_vjoint5_max) robot1_vjoint5_max = robot1_vjoint5;
    //        if(robot1_vjoint6 < robot1_vjoint6_min) robot1_vjoint6_min = robot1_vjoint6;
    //        if(robot1_vjoint6 > robot1_vjoint6_max) robot1_vjoint6_max = robot1_vjoint6;

    //        static int blah = 0;
    //        blah++;
    //        if(blah % 1000 == 0)
    //        {
    //            blah = 0;
    //            cout << robot1_joint1_min << ", " << robot1_joint1_max << endl;
    //            cout << robot1_joint2_min << ", " << robot1_joint2_max << endl;
    //            cout << robot1_joint3_min << ", " << robot1_joint3_max << endl;
    //            cout << robot1_joint4_min << ", " << robot1_joint4_max << endl;
    //            cout << robot1_joint5_min << ", " << robot1_joint5_max << endl;
    //            cout << robot1_joint6_min << ", " << robot1_joint6_max << endl;
    //            cout << "-------" << endl;
    //            cout << robot1_vjoint1_min << ", " << robot1_vjoint1_max << endl;
    //            cout << robot1_vjoint2_min << ", " << robot1_vjoint2_max << endl;
    //            cout << robot1_vjoint3_min << ", " << robot1_vjoint3_max << endl;
    //            cout << robot1_vjoint4_min << ", " << robot1_vjoint4_max << endl;
    //            cout << robot1_vjoint5_min << ", " << robot1_vjoint5_max << endl;
    //            cout << robot1_vjoint6_min << ", " << robot1_vjoint6_max << endl;
    //        }

    if(both_arms)
    {
        robot2_joint1 = robot_1->position.at(locations[6]);
        robot2_joint2 = robot_1->position.at(locations[7]);
        robot2_joint3 = robot_1->position.at(locations[8]);
        robot2_joint4 = robot_1->position.at(locations[9]);
        robot2_joint5 = robot_1->position.at(locations[10]);
        robot2_joint6 = robot_1->position.at(locations[11]);

        robot2_vjoint1 = robot_1->velocity.at(locations[6]);
        robot2_vjoint2 = robot_1->velocity.at(locations[7]);
        robot2_vjoint3 = robot_1->velocity.at(locations[8]);
        robot2_vjoint4 = robot_1->velocity.at(locations[9]);
        robot2_vjoint5 = robot_1->velocity.at(locations[10]);
        robot2_vjoint6 = robot_1->velocity.at(locations[11]);

        robot2_joint5 = wrap(robot2_joint5);
    }
    gazebo_msgs::GetLinkState gls;
    gls.request.link_name = "pr2::r_gripper_l_finger_tip_link";
    gls.request.reference_frame = "world";
    get_link_client.call(gls);
    if(!gls.response.success)
    {
        return;
    }
    robot1_end_x = gls.response.link_state.pose.position.x;
    robot1_end_y = gls.response.link_state.pose.position.y;
    robot1_end_z = gls.response.link_state.pose.position.z;


    gls.request.link_name = "pr2::l_gripper_l_finger_tip_link";
    gls.request.reference_frame = "world";
    get_link_client.call(gls);
    if(!gls.response.success)
    {
        return;
    }
    robot2_end_x = gls.response.link_state.pose.position.x;
    robot2_end_y = gls.response.link_state.pose.position.y;
    robot2_end_z = gls.response.link_state.pose.position.z;
    new_pose = true;
}

double ArmTouch::wrap(double rad)
{
    rad = fmod(rad, 2.0 * 3.14159);
    if (rad < 0.0)
        rad += 2.0 * 3.14159;
    return rad;
}

double ArmTouch::wrap(double rad, double min, double max)
{
    if(rad < min)
    {

    }
    if(rad > max)
    {

    }
    return rad;
}

ArmTouch::~ArmTouch() {
    client.shutdown();
    get_link_client.shutdown();
    robot_1_joint_1_pub.shutdown();
    robot_1_joint_2_pub.shutdown();
    robot_1_joint_3_pub.shutdown();
    robot_1_joint_4_pub.shutdown();
    robot_1_joint_5_pub.shutdown();
    robot_1_joint_6_pub.shutdown();
    robot_1_joint_7_pub.shutdown();

    if(both_arms)
    {
        robot_2_joint_1_pub.shutdown();
        robot_2_joint_2_pub.shutdown();
        robot_2_joint_3_pub.shutdown();
        robot_2_joint_4_pub.shutdown();
        robot_2_joint_5_pub.shutdown();
        robot_2_joint_6_pub.shutdown();
        robot_2_joint_7_pub.shutdown();
    }
    robot_1_subscriber.shutdown();
    //robot_1_subscriber->unsubscribe();

    clear_joint_client.shutdown();

    //delete robot_1_subscriber;

    // TODO Auto-generated destructor stub
}
