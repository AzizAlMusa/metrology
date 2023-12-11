search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=add_post_pro_robot_l515.srdf
robot_name_in_srdf=add_post_pro_robot_l515
moveit_config_pkg=add_post_pro_robot_l515_moveit_config
robot_name=add_post_pro_robot_l515
planning_group_name=arm
ikfast_plugin_pkg=add_post_pro_robot_l515_arm_ikfast_plugin
base_link_name=world
eef_link_name=camera_depth_optical_frame
ikfast_output_path=/home/abdulaziz/ros_workspace/metrology/src/add_post_pro_robot_l515_arm_ikfast_plugin/src/add_post_pro_robot_l515_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
