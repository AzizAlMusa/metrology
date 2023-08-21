#ifndef APP_DRIVER
#define APP_DRIVER

#include <stdio.h>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <std_msgs/Int32.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <curses.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

#ifndef MC8_DRIVER
#define MC8_DRIVER

#include <stdio.h>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <std_msgs/Int32.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <curses.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class Motors{

  private:
    ros::NodeHandle nh;
    ros::Subscriber turntableVel, armVel1, armVel2, armVel3, armVel4, armVel5, armVel6;
    ros::Subscriber turntablePos, armPos1, armPos2, armPos3, armPos4, armPos5, armPos6, jointStates;
    boost::shared_ptr<sensor_msgs::JointState const> jointStatesOnce;
    
    // joint state number: 

    const int TURNTABLE_MOTOR = 0;
    const int ARM_JOINT_1 = 1;
    const int ARM_JOINT_2 = 2;
    const int ARM_JOINT_3 = 3;
    const int ARM_JOINT_4 = 4;
    const int ARM_JOINT_5 = 5;
    const int ARM_JOINT_6 = 6;
    
    const int OFFSET = 0;

    // parameters for way point generation
    const int NUM_POS_JOINTS = 6;
    const int NUM_VEL_JOINTS = 1;
    const int NUM_WAYPOINTS = 10;

    // const double ARM_VELOCITY = 0.02;            // arm rotation speed
    // const double BODY_VELOCITY_LIMIT = 0.1;  // body speed
    // const double TRACK_VELOCITY_LIMIT = 1.0;   // track speed
    // const double TIME_INTERVAL = 0.1/5;       // waypoint interval in seconds
    // const double TIME_OFFSET = 0.05/1.25;        // time offset between each iteration
    // const double ARM_MAX_ANGLE = 1.57*3;
    // const double ARM_MIN_ANGLE = -1.57*3;

    // topics 
    const std::string turntablePosTopic = "/add_post_pro_robot/turntable_position_controller/command";
    const std::string turntableVelTopic = "/add_post_pro_robot/turntable_velocity_controller/command";

    const std::string arm1PosTopic = "/add_post_pro_robot/arm_1_position_controller/command";
    const std::string arm1VelTopic = "/add_post_pro_robot/arm_1_velocity_controller/command";

    const std::string arm2PosTopic = "/add_post_pro_robot/arm_2_position_controller/command";
    const std::string arm2VelTopic = "/add_post_pro_robot/arm_2_velocity_controller/command";

    const std::string arm3PosTopic = "/add_post_pro_robot/arm_3_position_controller/command";
    const std::string arm3VelTopic = "/add_post_pro_robot/arm_3_velocity_controller/command";

    const std::string arm4PosTopic = "/add_post_pro_robot/arm_4_position_controller/command";
    const std::string arm4VelTopic = "/add_post_pro_robot/arm_4_velocity_controller/command";

    const std::string arm5PosTopic = "/add_post_pro_robot/arm_5_position_controller/command";
    const std::string arm5VelTopic = "/add_post_pro_robot/arm_5_velocity_controller/command";

    const std::string arm6PosTopic = "/add_post_pro_robot/arm_6_position_controller/command";
    const std::string arm6VelTopic = "/add_post_pro_robot/arm_6_velocity_controller/command";

    TrajClient* traj_client_arm;
    TrajClient* traj_client_table;
    control_msgs::FollowJointTrajectoryGoal pos_goal;
    control_msgs::FollowJointTrajectoryGoal vel_goal;

    double TIME_FROM_START = 20;
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> joint_velocities;
    std::vector<double> joint_efforts;
    bool first_iter = true;

    // subscriber callback
    void turntablePosCallback(const std_msgs::Float64::ConstPtr& cmd);
    void turntableVelCallback(const std_msgs::Float64::ConstPtr& cmd);

    void arm1PosCallback(const std_msgs::Float64::ConstPtr& cmd);
    void arm1VelCallback(const std_msgs::Float64::ConstPtr& cmd);

    void arm2PosCallback(const std_msgs::Float64::ConstPtr& cmd);
    void arm2VelCallback(const std_msgs::Float64::ConstPtr& cmd);
    
    void arm3PosCallback(const std_msgs::Float64::ConstPtr& cmd);
    void arm3VelCallback(const std_msgs::Float64::ConstPtr& cmd);    

    void arm4PosCallback(const std_msgs::Float64::ConstPtr& cmd);
    void arm4VelCallback(const std_msgs::Float64::ConstPtr& cmd);

    void arm5PosCallback(const std_msgs::Float64::ConstPtr& cmd);
    void arm5VelCallback(const std_msgs::Float64::ConstPtr& cmd);

    void arm6PosCallback(const std_msgs::Float64::ConstPtr& cmd);
    void arm6VelCallback(const std_msgs::Float64::ConstPtr& cmd);

  public:
    Motors();
    ~Motors();
    void prePopulate();
    void sendCommands();
    bool trajectoryInit(void);
};

#endif