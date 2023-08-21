search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=vs6577.srdf
robot_name_in_srdf=vs6577
moveit_config_pkg=vs6577_moveit_config
robot_name=vs6577
planning_group_name=arm
ikfast_plugin_pkg=vs6577_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=J6
ikfast_output_path=/home/abdulaziz/addpost_ws_new/src/vs6577_arm_ikfast_plugin/src/vs6577_arm_ikfast_solver.cpp

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
