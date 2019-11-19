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
        call_ros_service("set_pose", SetPose,
                         service_args=(x, y, theta))
        rospy.sleep(0.05)

    def stop_moving(self):
        call_ros_service("stop_moving", StopMoving)

    def is_moving(self):
        resp = call_ros_service("is_moving", IsMoving)
        return resp.is_moving
    
    def is_at(self, x, y, theta):
        ''' Whether the robot is at the pose (x, y, theta). '''
        x_r, y_r, theta_r = self.get_pose()
        return self.are_two_poses_near(x, y, theta, x_r, y_r, theta_r)

    def are_two_poses_near(self,
                           x1, y1, theta1,
                           x2, y2, theta2,
                           x_tol=0.01, y_tol=0.01, theta_tol=0.1
                           ):
        b1 = abs(x1 - x2) <= x_tol
        b2 = abs(y1 - y2) <= y_tol
        b3 = abs(theta1 - theta2) <= theta_tol
        return b1 and b2 and b3

    def wait_until_stop(self):
        while self.is_moving():
            rospy.sleep(0.1)


def test_set_pose_IN_SIMULATION_ONLY():
    turtle = TurtleClient()
    turtle.reset_pose()
    x, y, theta = 1, 0, 1.57
    turtle.set_pose(x=x, y=y, theta=theta)
    assert(turtle.is_at(x, y, theta))
    rospy.loginfo("Test `set_pose` succeeds !!!")


def test_get_and_reset_pose():
    turtle = TurtleClient()

    # Change robot position.
    turtle.reset_pose()
    x, y, theta = 0.1, 0.1, 0
    turtle.move_to_pose(x, y, theta)
    turtle.wait_until_stop()

    # Test `get_pose`.
    x0, y0, theta0 = turtle.get_pose()
    assert(turtle.are_two_poses_near(
        x0, y0, theta0, x, y, theta))
    rospy.loginfo("Test `get_pose` succeeds !!!")

    # Test `reset_pose`.
    turtle.reset_pose()
    turtle.is_at(x=0, y=0, theta=0)
    rospy.loginfo("Test `reset_pose` succeeds !!!")


if __name__ == "__main__":
    rospy.init_node("turtlebot_client")
    test_get_and_reset_pose()
    test_set_pose_IN_SIMULATION_ONLY()
