#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from turntable_control.msg import MotorControl
from turntable_control.srv import MotorControlService, MotorControlServiceRequest
import pdb

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

def on_goal(goal_handle):
    goal_handle.set_accepted()  # Transition goal to active state
    goal = goal_handle.get_goal()
    trajectory = goal.trajectory
    result = FollowJointTrajectoryResult()
    motor_control_service = rospy.ServiceProxy('motor_control_service', MotorControlService)

    try:
        motor_commands = convert_trajectory_to_motor_commands(trajectory)
        for command in motor_commands:
            # Create a request object and fill it with the command data
            req = MotorControlServiceRequest()
            req.request = command
            motor_control_service(req)  # Synchronous call to Arduino service

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
    rospy.init_node('moveit_arduino_bridge')

    action_server = actionlib.ActionServer(
        '/add_post_pro_equiv/turntable_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction,
        on_goal,
        auto_start=False
    )

    action_server_dummy = actionlib.ActionServer(
        '/add_post_pro_equiv/arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction,
        on_goal_dummy,
        auto_start=False
    )

    action_server.start()
    action_server_dummy.start()

    rospy.spin()

if __name__ == '__main__':
    main()
