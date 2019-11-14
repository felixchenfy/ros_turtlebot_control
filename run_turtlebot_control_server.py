#!/usr/bin/env python

''' -------------------------------------------------------------------------- '''
import sys
import os
import yaml
import threading

import rospy

from lib_turtlebot import Turtle
from ros_turtlebot_control.srv import MoveToPoint, MoveToPointResponse
from ros_turtlebot_control.srv import MoveToPose, MoveToPoseResponse
from ros_turtlebot_control.srv import MoveToRelativePoint, MoveToRelativePointResponse
from ros_turtlebot_control.srv import MoveToRelativePose, MoveToRelativePoseResponse
from ros_turtlebot_control.srv import StopMoving, StopMovingResponse
from ros_turtlebot_control.srv import SetPose, SetPoseResponse
from ros_turtlebot_control.srv import ResetPose, ResetPoseResponse

''' -------------------------------------------------------------------------- '''

ROOT = os.path.dirname(os.path.abspath(__file__))+"/"
CONFIG_FILEPATH = ROOT + "config/config.yaml"
global NODE_NAME, SRV_NAMESPACE, turtle

''' -------------------------------------------------------------------------- '''


class SrvTemplate(object):
    ''' A ROS service template '''

    def __init__(self, srv_name,
                 srv_in_type,
                 srv_out_type):

        if SRV_NAMESPACE:
            srv_name = SRV_NAMESPACE + "/" + srv_name  # Add name space

        self._srv = rospy.Service(
            srv_name, srv_in_type, self._callback)
        rospy.loginfo("  Service starts: " + srv_name)
        self._srv_name = srv_name
        self._srv_in_type = srv_in_type
        self._srv_out_type = srv_out_type

    def _callback(self, req):
        raise NotImplementedError("Please overload this function!")


def start_thread_with_no_args(func):
    thread = threading.Thread(
        target=func,
        args=[])
    thread.start()


class HandleMoveToPoint(SrvTemplate):
    def __init__(self):
        super(HandleMoveToPoint, self).__init__(
            srv_name='move_to_point',
            srv_in_type=MoveToPoint,
            srv_out_type=MoveToPointResponse,
        )

    def _callback(self, req):
        print(self._srv_name + ": "
              "req.x={}, req.y={}".format(
                  req.x, req.y))

        def control_loop():
            turtle.move_to_pose(x_goal_w=req.x,
                                y_goal_w=req.y)
            rospy.loginfo("Service: " + self._srv_name + ": is completed!")
        start_thread_with_no_args(control_loop)
        return self._srv_out_type()


class HandleMoveToPose(SrvTemplate):
    def __init__(self):
        super(HandleMoveToPose, self).__init__(
            srv_name='move_to_pose',
            srv_in_type=MoveToPose,
            srv_out_type=MoveToPoseResponse,
        )

    def _callback(self, req):
        print(self._srv_name + ": "
              "req.x={}, req.y={}, req.theta={}".format(
                  req.x, req.y, req.theta))

        def control_loop():
            turtle.move_to_pose(x_goal_w=req.x,
                                y_goal_w=req.y,
                                theta_goal_w=req.theta)
            rospy.loginfo("Service: " + self._srv_name + ": is completed!")
        start_thread_with_no_args(control_loop)
        return self._srv_out_type()


class HandleMoveToRelativePoint(SrvTemplate):
    def __init__(self):
        super(HandleMoveToRelativePoint, self).__init__(
            srv_name='move_to_relative_point',
            srv_in_type=MoveToRelativePoint,
            srv_out_type=MoveToRelativePointResponse,
        )

    def _callback(self, req):
        print(self._srv_name + ": "
              "req.x={}, req.y={}".format(
                  req.x, req.y))

        def control_loop():
            turtle.move_to_relative_pose(
                x_goal_r=req.x,
                y_goal_r=req.y)
            rospy.loginfo("Service: " + self._srv_name + ": is completed!")
        start_thread_with_no_args(control_loop)
        return self._srv_out_type()


class HandleMoveToRelativePose(SrvTemplate):
    def __init__(self):
        super(HandleMoveToRelativePose, self).__init__(
            srv_name='move_to_relative_pose',
            srv_in_type=MoveToRelativePose,
            srv_out_type=MoveToRelativePoseResponse,
        )

    def _callback(self, req):
        print(self._srv_name + ": "
              "req.x={}, req.y={}, req.theta={}".format(
                  req.x, req.y, req.theta))

        def control_loop():
            turtle.move_to_relative_pose(
                x_goal_r=req.x,
                y_goal_r=req.y,
                theta_goal_r=req.theta)
            rospy.loginfo("Service: " + self._srv_name + ": is completed!")
        start_thread_with_no_args(control_loop)
        return self._srv_out_type()


class HandleStopMoving(SrvTemplate):
    def __init__(self):
        super(HandleStopMoving, self).__init__(
            srv_name='stop_moving',
            srv_in_type=StopMoving,
            srv_out_type=StopMovingResponse,
        )

    def _callback(self, req):
        turtle.stop_moving()
        rospy.loginfo("Service: " + self._srv_name + ": is completed!")
        return self._srv_out_type()


class HandleSetPose(SrvTemplate):
    def __init__(self):
        super(HandleSetPose, self).__init__(
            srv_name='set_pose',
            srv_in_type=SetPose,
            srv_out_type=SetPoseResponse,
        )

    def _callback(self, req):
        turtle.set_pose(req.x, req.y, req.theta)
        rospy.loginfo("Service: " + self._srv_name + ": is completed!")
        return self._srv_out_type()


class HandleResetPose(SrvTemplate):
    def __init__(self):
        super(HandleResetPose, self).__init__(
            srv_name='reset_pose',
            srv_in_type=ResetPose,
            srv_out_type=ResetPoseResponse,
        )

    def _callback(self, req):
        turtle.reset_pose()
        rospy.loginfo("Service: " + self._srv_name + ": is completed!")
        return self._srv_out_type()


def read_yaml_file(filepath):
    ''' Read contents from the yaml file.
    Output:
        data_dict {dict}: contents of the yaml file.
            The keys of the dict are `str` type.
    '''
    with open(filepath, 'r') as stream:
        data_dict = yaml.safe_load(stream)
    return data_dict


def main():

    global NODE_NAME, SRV_NAMESPACE, turtle

    NODE_NAME = 'run_turtlebot_control_server'
    rospy.init_node(NODE_NAME)
    rospy.loginfo("Node starts: " + NODE_NAME)

    # Init turtle.
    turtle = Turtle(CONFIG_FILEPATH)
    cfg = read_yaml_file(CONFIG_FILEPATH)
    SRV_NAMESPACE = cfg["srv_namespace"]
    rospy.on_shutdown(lambda: turtle.set_speed(v=0, w=0))

    # Init ROS service servers
    h1 = HandleMoveToPoint()
    h2 = HandleMoveToPose()
    h3 = HandleMoveToRelativePoint()
    h4 = HandleMoveToRelativePose()
    h5 = HandleStopMoving()
    h6 = HandleSetPose()
    h7 = HandleResetPose()

    # Loop
    rospy.spin()
    rospy.loginfo("Node stops: " + NODE_NAME)


if __name__ == "__main__":
    main()
