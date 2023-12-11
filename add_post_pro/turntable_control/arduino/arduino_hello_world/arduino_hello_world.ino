#include <ros.h>
#include <turntable_control/MotorControl.h>  // Include the message header file
#include <turntable_control/MotorControlService.h>  // Include the service header file
#include <std_msgs/Bool.h>  // Include the Bool message type
#include <AccelStepper.h>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif
#define PUL_PIN 9   // Connect to the PUL pin of your stepper driver
#define DIR_PIN 8   // Connect to the DIR pin of your stepper driver
#define MICROSTEPS 200  // Change this value based on your stepper driver's microstepping settings
#define GEAR 90
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

ros::NodeHandle nh;

enum State {
    IDLE,
    MOVING
};

State motorState = IDLE;

void handle_motor_control(turntable_control::MotorControlService::Request &req,
                          turntable_control::MotorControlService::Response &res) {
  nh.loginfo("Executing server commands");  // Log motor command execution

  // Assuming req.request.positions[0] is the angle in radians
  float angleInRadians = req.request.positions[0];

  // Calculate the number of steps
  int numberOfSteps = static_cast<int>(angleInRadians / (2.0 * M_PI) * MICROSTEPS * GEAR);  // 400 steps per revolution
  
  // Move to the target position
  stepper.moveTo(numberOfSteps);

  motorState = MOVING;
  res.response = true;
}

ros::ServiceServer<turntable_control::MotorControlService::Request, turntable_control::MotorControlService::Response> service("motor_control_service", &handle_motor_control);

void setup() {
  nh.initNode();
  nh.advertiseService(service);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(1000);
}

void loop() {
  nh.spinOnce();

  // Check the motor state and run it if it's moving
  if (motorState == MOVING) {
    stepper.run();
    
    // Check if the motor has reached the target position
    if (stepper.distanceToGo() == 0) {
      motorState = IDLE;
    }
  }
  
  // You may add other non-blocking tasks here if necessary.
}


// AccelStepper stepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);

// bool handle_motor_control(turntable_control::MotorControlService::Request &req,
//                           turntable_control::MotorControlService::Response &res) {
  
//   nh.loginfo("Executing server commands");  // Log motor command execution

//   // Assuming req.request.positions[0] is the angle in radians
//   float angleInRadians = req.request.positions[0];


//   // Calculate the number of steps
//   int numberOfSteps = static_cast<int>(angleInRadians / (2.0 * M_PI) * MICROSTEPS * GEAR * -1);  // 400 steps per revolution
  
//   // Move to the target position
//   stepper.moveTo(numberOfSteps);
//   stepper.runToPosition();
//   // Monitor the stepper's position
//   // while (stepper.isRunning()) {
//   //   stepper.run();
//   //   // nh.spinOnce(); // Process ROS messages
//   // }

//   res.response = true;
//   return true;  // Acknowledgement
// }

// ros::ServiceServer<turntable_control::MotorControlService::Request, turntable_control::MotorControlService::Response> 
//   service("motor_control_service", &handle_motor_control);

// void setup() {
//   nh.initNode();
//   nh.advertiseService(service);
//   stepper.setMaxSpeed(2500);
//   stepper.setAcceleration(1200);
// }

// void loop() {
//   nh.spinOnce();
//   // stepper.run();
// }
