#!/usr/bin/env python
#!/usr/bin/env python

'''
This script contains functions and unit tests for
    calling the services started by `turtlebot_server.py`.
You may copy this file to your project repo
    and import `class TurtlebotClient` for controlling Turtlebot. 
'''


from ros_turtlebot_control.srv import GetPose
from ros_turtlebot_control.srv import MoveToPoint
from ros_turtlebot_control.srv import MoveToPose
from ros_turtlebot_control.srv import MoveToRelativePoint
from ros_turtlebot_control.srv import MoveToRelativePose
from ros_turtlebot_control.srv import ResetPose
from ros_turtlebot_control.srv import SetPose
from ros_turtlebot_control.srv import StopMoving
from ros_turtlebot_control.srv import IsMoving, IsMovingResponse

import rospy
import sys

import numpy as np
import math

SRV_NAMESPACE = "turtle"


def call_ros_service(service_name, service_type,
                     service_args=None, srv_namespace=SRV_NAMESPACE):
    ''' Call a ROS service.
    '''
    service_name = srv_namespace + "/" + service_name
    rospy.wait_for_service(service_name)
    try:
        func = rospy.ServiceProxy(service_name, service_type)
        resp = func(*service_args) if service_args else func()
        return resp
    except rospy.ServiceException as e:
        print("Failed to call service:", service_name)
        sys.exit()


class TurtleClient(object):
    def __init__(self):
        pass

    def get_pose(self):
        resp = call_ros_service(
            "get_pose", GetPose)
        x, y, theta = resp.x, resp.y, resp.theta
        return x, y, theta

    def move_to_point(self, x, y):
        call_ros_service("move_to_point", MoveToPoint,
                         service_args=(x, y))

    def move_to_pose(self, x, y, theta):
        call_ros_service("move_to_pose", MoveToPose,
                         service_args=(x, y, theta))

    def move_to_relative_point(self, x, y):
        call_ros_service("move_to_relative_point", MoveToRelativePoint,
                         service_args=(x, y))

    def move_to_relative_pose(self, x, y, theta):
        call_ros_service("move_to_relative_pose", MoveToRelativePose,
                         service_args=(x, y, theta))

    def reset_pose(self):
        call_ros_service("reset_pose", ResetPose)
        rospy.sleep(0.05)

    def set_pose(self, x, y, theta):
        rospy.logwarn("`set_pose` is only supported for "
                      "Gazebo simultion, not real robot !!!")
        call_ros_service("set_pose", SetPose,
                         service_args=(x, y, theta))
        rospy.sleep(0.05)

    def stop_moving(self):
        call_ros_service("stop_moving", StopMoving)
        rospy.sleep(0.05)

    def is_moving(self):
        resp = call_ros_service("is_moving", IsMoving)
        return resp.is_moving

    def is_at(self, x, y, theta=None,
              x_tol=None, y_tol=None, theta_tol=None):
        ''' Whether the robot is at the pose (x, y, theta). '''
        x_r, y_r, theta_r = self.get_pose()
        if theta is None:
            theta = theta_r
        return self.are_two_poses_near(
            x, y, theta,
            x_r, y_r, theta_r,
            x_tol, y_tol, theta_tol)

    def are_two_poses_near(self,
                           x1, y1, theta1,
                           x2, y2, theta2,
                           x_tol=None, y_tol=None, theta_tol=None
                           ):
        x_tol = 0.015 if x_tol is None else x_tol
        y_tol = 0.015 if y_tol is None else y_tol
        theta_tol = 0.08 if theta_tol is None else theta_tol
        b1 = abs(x1 - x2) <= x_tol
        b2 = abs(y1 - y2) <= y_tol
        b3 = abs(theta1 - theta2) <= theta_tol
        return b1 and b2 and b3

    def wait_until_stop(self):
        while self.is_moving():
            rospy.sleep(0.1)


def test_set_pose_IN_SIMULATION_ONLY():
    rospy.loginfo("Testing `set_pose`...")
    turtle = TurtleClient()
    turtle.reset_pose()
    x, y, theta = 1, 0, 1.57
    turtle.set_pose(x=x, y=y, theta=theta)
    assert turtle.is_at(x, y, theta), "`set_pose` failed."
    rospy.loginfo("Test `set_pose` succeeds !!!")


def test_get_and_reset_pose():
    turtle = TurtleClient()

    # -- Test `get_pose`.
    rospy.loginfo("Testing `get_pose`...")

    # Change robot position.
    turtle.reset_pose()
    x, y, theta = 0.1, 0.1, 0
    turtle.move_to_pose(x, y, theta)
    turtle.wait_until_stop()

    # Get pose.
    x0, y0, theta0 = turtle.get_pose()
    assert turtle.are_two_poses_near(
        x0, y0, theta0, x, y, theta), "`get_pose` failed"
    rospy.loginfo("Test `get_pose` succeeds !!!")

    # -- Test `reset_pose`.
    rospy.loginfo("Testing `reset_pose`...")
    turtle.reset_pose()
    assert turtle.is_at(x=0, y=0, theta=0), "`reset_pose` failed"
    rospy.loginfo("Test `reset_pose` succeeds !!!")


def test_move_to_poses():
    turtle = TurtleClient()
    turtle.reset_pose()

    # -- Test `move_to_pose`.
    rospy.loginfo("Testing `move_to_pose`...")
    x, y, theta = 0.5, 0, 1.57
    # Move half way, to avoid ambiguity with `move_to_relative_pose`.
    turtle.move_to_pose(x/2.0, y/2.0, 0.0)
    turtle.wait_until_stop()
    turtle.move_to_pose(x, y, theta)  # Then, move to target.
    turtle.wait_until_stop()
    assert turtle.is_at(x, y, theta), "`move_to_pose` failed."
    rospy.loginfo("Test `move_to_pose` succeeds !!!")

    # -- Test `move_to_relative_pose`.
    rospy.loginfo("Testing `move_to_relative_pose`...")
    x, y, theta = -0.5, 0.5, -1.57
    turtle.move_to_relative_pose(x, y, theta)
    turtle.wait_until_stop()
    is_position_correct = turtle.is_at(
        x=0, y=-0.5, theta=0,  # Global pose = (0, -1, 0).
        # Due to the error of `move_to_pose`,
        #   here we allow larger error.
        x_tol=0.05, y_tol=0.05, theta_tol=0.1)
    assert is_position_correct, "`move_to_relative_pose` failed."
    rospy.loginfo("Test `move_to_relative_pose` succeeds !!!")


def test_move_to_points():
    turtle = TurtleClient()
    turtle.reset_pose()

    # -- Test `move_to_point`.
    rospy.loginfo("Testing `move_to_point`...")
    x, y = 0.5, 0
    # Move half way, to avoid ambiguity with `move_to_relative_point`.
    turtle.move_to_point(x/2.0, y/2.0)
    turtle.wait_until_stop()
    turtle.move_to_point(x, y)  # Then, move to target.
    turtle.wait_until_stop()
    assert turtle.is_at(x, y), "`move_to_point` failed."
    rospy.loginfo("Test `move_to_point` succeeds !!!")

    # -- Test `move_to_relative_point`.
    rospy.loginfo("Testing `move_to_relative_point`...")
    x, y = -0.5, 0.5
    turtle.move_to_relative_point(x, y)
    turtle.wait_until_stop()
    is_position_correct = turtle.is_at(
        x=0, y=0.5,  # Global position = (0, 1).
        # Due to the error of `move_to_point`,
        #   here we allow larger error.
        x_tol=0.05, y_tol=0.05)
    assert is_position_correct, "`move_to_relative_point` failed."
    rospy.loginfo("Test `move_to_relative_point` succeeds !!!")

    turtle.move_to_pose(x=0, y=0.5, theta=0)  # Rotate to face front.


def test_change_target_and_stop():
    turtle = TurtleClient()
    turtle.reset_pose()

    # -- Test `change target`.
    rospy.loginfo("Testing `change target`...")

    # Set the initial target as x=-0.5.
    turtle.move_to_relative_pose(-0.5, 0, 0)
    rospy.sleep(0.1)

    # Change target to be x=1.0.
    turtle.move_to_relative_pose(1.0, 0, 0)
    rospy.sleep(2.0)

    # Check that the robot is not at old target.
    is_at_old_position = turtle.is_at(
        x=-0.5, y=0, theta=0)
    assert not is_at_old_position, "`change target` failed."
    rospy.loginfo("Test `change target` succeeds !!!")

    # -- Test `stop_moving`.
    turtle.stop_moving()
    assert not turtle.is_moving(),  "`stop_moving` failed."
    rospy.loginfo("Test `stop_moving` succeeds !!!")

    # Move back to origin.
    rospy.sleep(1.0)
    rospy.loginfo("Move back to origin.")
    turtle.move_to_pose(0, 0, 0)


if __name__ == "__main__":
    rospy.init_node("turtlebot_client")
    test_set_pose_IN_SIMULATION_ONLY()  # Not for real robot.
    test_get_and_reset_pose()
    test_move_to_poses()
    test_move_to_points()
    test_change_target_and_stop()
