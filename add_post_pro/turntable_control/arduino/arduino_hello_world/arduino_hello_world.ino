#include <ros.h>
#include <std_msgs/Int32.h>
#include <AccelStepper.h>

ros::NodeHandle nh;

AccelStepper stepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5);  // Define stepper motor connections

void messageCallback(const std_msgs::Int32& msg)
{
  // Handle the received integer data here
  int received_integer = msg.data;

  // Move the stepper motor the same amount of steps as the received integer
  stepper.moveTo(received_integer);

  // Run the stepper motor
  stepper.runToPosition();
}

ros::Subscriber<std_msgs::Int32> sub("integer_topic", &messageCallback);

void setup()
{
  nh.initNode();

  // Set the maximum speed and acceleration for the stepper motor
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  nh.subscribe(sub);

}

void loop()
{
  nh.spinOnce();
}
