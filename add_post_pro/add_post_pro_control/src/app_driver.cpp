#include "app_driver.h"


// mc8 driver seems to have velocity control for each of the 4 tracks and position controllers for just the two arms

// Gonna try just using pos for arm and vel for turntable
Motors::Motors()
{

    traj_client_arm = new TrajClient("/add_post_pro_robot/arm_controller/follow_joint_trajectory", true);
    traj_client_table = new TrajClient("/add_post_pro_robot/arm_controller/follow_joint_trajectory", true);

    while(!traj_client_arm->waitForServer(ros::Duration(3.0))){
        ROS_INFO("Waiting for joint_trajectory_action server");
    }

    turntableVel = 
        nh.subscribe<std_msgs::Float64>(turntableVelTopic, 1, &Motors::turntableVelCallback, this);

    armPos1 = 
        nh.subscribe<std_msgs::Float64>(arm1PosTopic, 1, &Motors::arm1PosCallback, this);
    armPos2 = 
        nh.subscribe<std_msgs::Float64>(arm2PosTopic, 1, &Motors::arm2PosCallback, this);
    armPos3 = 
        nh.subscribe<std_msgs::Float64>(arm3PosTopic, 1, &Motors::arm3PosCallback, this);
    armPos4 = 
        nh.subscribe<std_msgs::Float64>(arm4PosTopic, 1, &Motors::arm4PosCallback, this);
    armPos5 = 
        nh.subscribe<std_msgs::Float64>(arm5PosTopic, 1, &Motors::arm5PosCallback, this);
    armPos6 = 
        nh.subscribe<std_msgs::Float64>(arm6PosTopic, 1, &Motors::arm6PosCallback, this);

    // Velocity control
    vel_goal.trajectory.joint_names.push_back("turntable_revolve_joint");
    for(int i=0;i<vel_goal.trajectory.points.size();i++){
        vel_goal.trajectory.points[i].positions.resize(NUM_VEL_JOINTS);
    }

    // Position control
    pos_goal.trajectory.joint_names.push_back("joint_1");
    pos_goal.trajectory.joint_names.push_back("joint_2");
    pos_goal.trajectory.joint_names.push_back("joint_3");
    pos_goal.trajectory.joint_names.push_back("joint_4");
    pos_goal.trajectory.joint_names.push_back("joint_5");
    pos_goal.trajectory.joint_names.push_back("joint_6");
    for(int i=0;i<pos_goal.trajectory.points.size();i++){
        pos_goal.trajectory.points[i].positions.resize(NUM_POS_JOINTS);
    }

}

Motors::~Motors(){
    delete traj_client_arm;
    delete traj_client_table;
}

void Motors::prePopulate()
{
    ROS_INFO("Pre-populating...");

    if(!joint_positions.empty())
    {

    }
}