# __**Depowdering Bringup**__

## Directories
### DyCalibResult
Results of D415 calibration. Probably going to be removed.

### gazebo_models
Stores Gazebo models (SDF and meshes) for the environment around the robot arm.

### launch
Contains the launch files needed to load up various parts of the project. 

### rviz
Stores Rviz configuration files that can be loaded up to open submenus on launch.

### src 
Programs to load parts of the environment or attach end effectors to the robot arm. These load them in as 
MoveIt! collision objects to aide in collision detection. These programs will NOT load models into Gazebo. 
#### Programs within src
 - load_environment.cpp
   * Adds the workspace frame to the MoveIt-detected environment, meaning it will be used for collision avoidance.  
     This node is not responsible for spawning the frame into the Gazebo simulation. **Included in the bringup.**

 - attach_box.cpp
   * Creates the default workspace box as seen in Hamerschlag B120. This node follows the same rules as load_environment. **Included in the bringup.**

  - attach_vacuum.cpp
    * Attaches the vacuum end effector to the robot's MoveIt environment.

  - load_scan_surface.cpp
    * Adds a large flat surface to the environment simulating a slightly slanted surface. This is used to test the line scanner's alignment program.

  - spawn_surface.py
    * Provides services for object spawning in different contexts. Will be generalized and optimized soon.
    
### urdf
Meshes and URDFs (in xacro format) for the depowdering environment.

### world
Stores the gazebo environment(s) that are loaded up in simulation mode. 