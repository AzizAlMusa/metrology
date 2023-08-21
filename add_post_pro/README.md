# __**CMU Robotic Depowdering for Additive Manufacturing**__

## __**Project Compilation Instructions**__
 1. Find a location where you'd like to install the project to.
 2. Run the following commands:
 `git clone https://github.com/tswie/add_post_pro.git`
 `git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git`
 3. This will create a folder named 'add_post_pro' and download the project packages into it in addition to the 'gazebo_ros_link_attacher' package.
 4. Next, we will set up rosdep.
    1. If you haven't already, type the following command:
   `sudo rosdep init`
   This only needs to be done once per ROS installation, so only running the command once is necessary. If you are prompted
   with an error telling you a file already exists, you can move on safely.
    2. Make sure you are at the top directory of your workspace (e.g. `~/Path/To/Project/`). You should be viewing the directory
   containing the `src` and `scripts` directories. Once you are, run the command: `rosdep update`
    3. While at the top directory, type the following command:
   `rosdep install --from-paths src --ignore-src -r -y`

 5. Now, we will install the dependencies for the scanCONTROL 2900-100 laser profile scanner.
    **(SKIP IF ON SIMULATOR_ONLY BRANCH)**
    1. In a different location than your add_post_pro folder, download the following repository:
   `git clone https://github.com/tswie/depowdering_microepsilon_install.git` (if you lack access to this repository, please contact Tomasz Swierzewski).
    2. Follow the instructions inside to install the dependencies.

 6. Next, to install the realsense packages, you should be able to run the following commands:
 `sudo apt-get install ros-melodic-realsense2-camera`
 `sudo apt-get install ros-melodic-realsense2-description`
 `sudo apt-get install ros-melodic-realsense2-camera-dbgsym`

 1. Now go to the project root folder and run the following command:
 `catkin_make`. If there are any issues, please check the error section to see if it was previously fixed.

 8. Once compilation is done, add the following line to the end of your .bashrc file located in your home directory (`~/.bashrc`):
 `source $PATH_TO_PROJECT/devel/setup.bash`
 where $PATH_TO_PROJECT is the location of your project folder.

## __**Launch Instructions**__

Before launching the simulator, we need to let the system know
where our ROS packages and Gazebo models are.
We can do this by sourcing.
In the workspace, do:

```bash
source devel/setup.bash
source src/add_post_pro/set_gazebo_path.sh
```

To start the simulation (Rviz and Gazebo), use the following command:
`roslaunch add_post_pro_bringup add_post_pro_bringup.launch`

## __**Overview of Packages**__

These nodes and services are the ones that have been created for this project and will likely be the ones that you have to manually interact with the most:

### __**laser_scanner_pkg**__

- sim_process.cpp

  Runs services allowing the user to call for a single profile from the laser scanner.

### __**pcl_sensor_data**__

- access_data.cpp

  Provides services and topics for handling the laser scanner input (simulated and actual).

- laser_processor.cpp

  Creates services for starting, stopping, clearing, and processing laser scanner output.

- sensor_client.py

  Tests several package services.

- test_node.cpp

  Same purpose as sensor_client.py. Probably due to be cleaned up. **TODO**

### __**realsense_cam**__

- im_processing.py

  Provides image filtering and pointcloud processing for rough depowdering. **Included in the bringup.**

- pcl_process.cpp / pcl_process.py

  Converts messed up pointclouds from XYZ to XYZRGB.

- realsense_services.cpp

  Provides services for retrieving single images and pointclouds from the depth camera.  **Included in the bringup.**

- service_test.cpp

  Tests the realsense_services.

### __**realsense_gazebo_plugin**__

- This package provides the plugin used to simulate the realsense in Gazebo. This shouldn't be touched.

### __**realsense-ros**__

- Provides programs and descriptions for the realsense cameras. This shouldn't be touched.


## __**add_post_pro Specific Services**__

- `/add_post_pro_simulation/delete_model`
- `/add_post_pro_simulation/list_models`
- `/add_post_pro_simulation/spawn_model`
- `/add_post_pro_simulation/stl_conversion`
- `/realsense_services/find_box_points`
- `/realsense_services/find_workspace_center`
- `/realsense_services/next_depth`
- `/realsense_services/next_pointcloud`
- `/realsense_services/next_rgb`
- `/realsense_services/save_pcl`
- `/realsense_services/save_rgb`

## __**Test bunny object on turntable**__

1. First, lauunch the simulator: `roslaunch add_post_pro_bringup add_post_pro_bringup.launch`
2. Then, spawn an object on the turntable:

    ```bash
    rosservice call /add_post_pro_simulation/spawn_model "model_name: 'test_bunny'
    instance_name: 'test_bunny'
    turntable: true
    pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
    parent_frame: 'turntable_plate'"
    ```
3. Finally, move the arm to scan the object on the turntable:

   ```bash
   rosrun add_post_pro_simulation turntable_scanning_demo
   ```
## Directly move the manipulator by providing step by step joint values

Run the simulator:
```bash
roslaunch add_post_pro_bringup add_post_pro_bringup.launch
```

To execute the joint values in *add_post_pro_control/config/manipulator_motion_plan.csv*
using a time step of 2 s:

```bash
rosrun add_post_pro_control manipulator_direct_control.py
```

