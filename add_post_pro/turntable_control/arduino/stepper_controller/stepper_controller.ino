#include <ros.h>
#include <trajectory_msgs/JointTrajectory.h>

// Define the callback function for handling incoming trajectory messages
void trajectoryCallback(const trajectory_msgs::JointTrajectory& msg) {
  // Get the number of points in the trajectory
  int numPoints = msg.points.size();
  
  // Loop through the points in the trajectory
  for (int i = 0; i < numPoints; i++) {
    // Get the number of positions in each point
    int numPositions = msg.points[i].positions.size();
    
    // Loop through the positions in each point
    for (int j = 0; j < numPositions; j++) {
      float position = msg.points[i].positions[j];
      // Do something with the joint position, e.g., control your motors
      // Example: motor.write(position);
    }
  }
}

// Initialize a ROS node and subscriber
ros::NodeHandle nh;
ros::Subscriber<trajectory_msgs::JointTrajectory> sub("arduino_trajectory", &trajectoryCallback);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize the ROS node
  nh.initNode();
  
  // Subscribe to the "arduino_trajectory" topic
  nh.subscribe(sub);
}

void loop() {
  // Handle ROS communication
  nh.spinOnce();
}
