#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from turntable_control.msg import MotorControl
from turntable_control.srv import MotorControlService, MotorControlServiceRequest
from sensor_msgs.msg import JointState
import pdb

# Initialize the current joint state with zero values
current_joint_state = JointState()
current_joint_state.name = ['turntable_joint']  # Replace with actual joint name(s)
current_joint_state.position = [0]
current_joint_state.velocity = [0]
current_joint_state.effort = [0]

def convert_trajectory_to_motor_commands(trajectory):
    """Convert a joint trajectory to motor commands"""
    motor_commands = []
    for point in trajectory.points:
        command = MotorControl()
        command.positions = point.positions
        command.velocities = point.velocities
        command.accelerations = point.accelerations
        command.effort = []  # Assuming no effort data is available
        command.time_from_start = point.time_from_start
        motor_commands.append(command)
    return motor_commands

def create_joint_state_message(command):
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['turntable_joint']  # Replace with the actual joint name(s)
    # Negate positions and velocities
    joint_state.position = [-pos for pos in command.positions]
    joint_state.velocity = [-vel for vel in command.velocities]
    
    joint_state.effort = command.effort
    return joint_state


def publish_joint_state(event):
    global current_joint_state
    if current_joint_state is not None:
        joint_state_publisher.publish(current_joint_state)


def on_goal(goal_handle):

    global current_joint_state

    goal_handle.set_accepted()  # Transition goal to active state
    goal = goal_handle.get_goal()
    trajectory = goal.trajectory
    result = FollowJointTrajectoryResult()
    motor_control_service = rospy.ServiceProxy('motor_control_service', MotorControlService)
    


    try:
        motor_commands = convert_trajectory_to_motor_commands(trajectory)
       
        #Only send the final position
        final_command = motor_commands[-1]  # Get the last motor command from the list
        print(final_command)
        # Create a request object and fill it with the final command data
        req = MotorControlServiceRequest()
        req.request = final_command
        motor_control_service(req)  # Synchronous call to Arduino service

         # Update the global joint state variable
        for command in motor_commands:
            # Create a request object and fill it with the command data
            current_joint_state = create_joint_state_message(command)
            joint_state_publisher.publish(current_joint_state)

        result.error_code = result.SUCCESSFUL




    except Exception as e:
        rospy.logerr("Error executing trajectory: %s", str(e))
        result.error_code = result.PATH_TOLERANCE_VIOLATED

    goal_handle.set_succeeded(result)  # Transition goal to succeeded state

def on_goal_dummy(goal_handle):
    """Callback function for when a new goal is received"""
    goal_handle.set_accepted()  # Transition goal to active state
    goal = goal_handle.get_goal()
    trajectory = goal.trajectory
    result = FollowJointTrajectoryResult()
    rospy.loginfo("arm server callback confirmed!")
    goal_handle.set_succeeded(result)  # Transition goal to succeeded state



def main():

    global joint_state_publisher

    rospy.init_node('moveit_arduino_bridge')


    action_server = actionlib.ActionServer(
        '/physical_turntable/turntable_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction,
        on_goal,
        auto_start=False
    )

    joint_state_publisher = rospy.Publisher('turntable_joint_states', JointState, queue_size=10)

    # Start the timer to continuously publish joint states
    rospy.Timer(rospy.Duration(0.1), publish_joint_state)  # Adjust the duration as needed
    # action_server_dummy = actionlib.ActionServer(
    #     '/add_post_pro_equiv/arm_controller/follow_joint_trajectory',
    #     FollowJointTrajectoryAction,
    #     on_goal_dummy,
    #     auto_start=False
    # )

    action_server.start()
    # action_server_dummy.start()

    rospy.spin()

if __name__ == '__main__':
    main()