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
#define MICROSTEPS 1600  // Change this value based on your stepper driver's microstepping settings
#define GEAR 90
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

ros::NodeHandle nh;

// AccelStepper stepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);

bool handle_motor_control(turntable_control::MotorControlService::Request &req,
                          turntable_control::MotorControlService::Response &res) {
  
  nh.loginfo("Executing server commands");  // Log motor command execution

  // Assuming req.request.positions[0] is the angle in radians
  float angleInRadians = req.request.positions[0];


  // Calculate the number of steps
  int numberOfSteps = static_cast<int>(angleInRadians / (2.0 * M_PI) * MICROSTEPS * GEAR * -1);  // 400 steps per revolution
  
  // Move to the target position
  stepper.moveTo(numberOfSteps);


  res.response = true;
  return true;  // Acknowledgement
}

ros::ServiceServer<turntable_control::MotorControlService::Request, turntable_control::MotorControlService::Response> 
  service("motor_control_service", &handle_motor_control);

void setup() {
  nh.initNode();
  nh.advertiseService(service);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
}

void loop() {
  nh.spinOnce();
  stepper.run();
}
