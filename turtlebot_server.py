#!/usr/bin/env python

'''
This script starts all Turtlebot control services 
    which are defined under `srv/` folder.
The key setence is:
    turtle_services = TurtlebotControlRosServices()
    turtle_services.start()
WARNING: `SetPose` is not supported for real robot, only for Gazebo simulation.
'''

from ros_turtlebot_control.srv import GetPose, GetPoseResponse
from ros_turtlebot_control.srv import MoveToPoint, MoveToPointResponse
from ros_turtlebot_control.srv import MoveToPose, MoveToPoseResponse
from ros_turtlebot_control.srv import MoveToRelativePoint, MoveToRelativePointResponse
from ros_turtlebot_control.srv import MoveToRelativePose, MoveToRelativePoseResponse
from ros_turtlebot_control.srv import ResetPose, ResetPoseResponse
from ros_turtlebot_control.srv import SetPose, SetPoseResponse
from ros_turtlebot_control.srv import StopMoving, StopMovingResponse
from ros_turtlebot_control.srv import IsMoving, IsMovingResponse

import rospy
import threading
import yaml
import os
import sys
from turtle_lib import Turtle
from utils.commons import read_yaml_file


ROOT = os.path.dirname(os.path.abspath(__file__))+"/"
CONFIG_FILEPATH = ROOT + "config.yaml"
NODE_NAME = 'run_turtlebot_control_server'
SRV_NAMESPACE, turtle = None, None  # To be initialized later.

# ============================================================================== #


def _srv_callback_wrapper(callback_func):
    ''' Print messages before and after the callback function. '''

    def new_callback_func(self, req):
        '''Argument: `req` is the input of the ROS service call. '''
        rospy.loginfo("Service: " + self._srv_name +
                      ": Receive request: {}".format(req))
        response = callback_func(self, req)
        rospy.loginfo("Service: " + self._srv_name +
                      ": Request has been sent to turtlebot_lib.py!")
        return response
    return new_callback_func


class _SrvTemplate(object):
    ''' A template for creating ROS service. '''

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

# ============================================================================== #


class TurtlebotControlRosServices(object):
    def __init__(self):
        self._is_start = False

    def start(self):
        self._h1 = TurtlebotControlRosServices.ServiceMoveToPoint()
        self._h2 = TurtlebotControlRosServices.ServiceMoveToPose()
        self._h3 = TurtlebotControlRosServices.ServiceMoveToRelativePoint()
        self._h4 = TurtlebotControlRosServices.ServiceMoveToRelativePose()
        self._h5 = TurtlebotControlRosServices.ServiceStopMoving()
        self._h6 = TurtlebotControlRosServices.ServiceSetPose()
        self._h7 = TurtlebotControlRosServices.ServiceResetPose()
        self._h8 = TurtlebotControlRosServices.ServiceGetPose()
        self._h9 = TurtlebotControlRosServices.ServiceIsMoving()
        self._is_start = True

    def __del__(self):
        if self._is_start:
            turtle.stop_moving()

    class ServiceMoveToPoint(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceMoveToPoint, self).__init__(
                srv_name='move_to_point',
                srv_in_type=MoveToPoint,
                srv_out_type=MoveToPointResponse,
            )

        def _callback(self, req):
            turtle.move_to_pose(x_goal_w=req.x,
                                y_goal_w=req.y)
            return self._srv_out_type()

    class ServiceMoveToPose(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceMoveToPose, self).__init__(
                srv_name='move_to_pose',
                srv_in_type=MoveToPose,
                srv_out_type=MoveToPoseResponse,
            )

        @_srv_callback_wrapper
        def _callback(self, req):
            turtle.move_to_pose(x_goal_w=req.x,
                                y_goal_w=req.y,
                                theta_goal_w=req.theta)
            return self._srv_out_type()

    class ServiceMoveToRelativePoint(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceMoveToRelativePoint, self).__init__(
                srv_name='move_to_relative_point',
                srv_in_type=MoveToRelativePoint,
                srv_out_type=MoveToRelativePointResponse,
            )

        @_srv_callback_wrapper
        def _callback(self, req):
            turtle.move_to_relative_pose(
                x_goal_r=req.x,
                y_goal_r=req.y)
            return self._srv_out_type()

    class ServiceMoveToRelativePose(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceMoveToRelativePose, self).__init__(
                srv_name='move_to_relative_pose',
                srv_in_type=MoveToRelativePose,
                srv_out_type=MoveToRelativePoseResponse,
            )

        @_srv_callback_wrapper
        def _callback(self, req):
            turtle.move_to_relative_pose(
                x_goal_r=req.x,
                y_goal_r=req.y,
                theta_goal_r=req.theta)
            return self._srv_out_type()

    class ServiceStopMoving(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceStopMoving, self).__init__(
                srv_name='stop_moving',
                srv_in_type=StopMoving,
                srv_out_type=StopMovingResponse,
            )

        @_srv_callback_wrapper
        def _callback(self, req):
            turtle.stop_moving()
            return self._srv_out_type()

    class ServiceSetPose(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceSetPose, self).__init__(
                srv_name='set_pose',
                srv_in_type=SetPose,
                srv_out_type=SetPoseResponse,
            )

        @_srv_callback_wrapper
        def _callback(self, req):
            turtle.set_pose(req.x, req.y, req.theta)
            return self._srv_out_type()

    class ServiceResetPose(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceResetPose, self).__init__(
                srv_name='reset_pose',
                srv_in_type=ResetPose,
                srv_out_type=ResetPoseResponse,
            )

        @_srv_callback_wrapper
        def _callback(self, req):
            turtle.reset_pose()
            return self._srv_out_type()

    class ServiceGetPose(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceGetPose, self).__init__(
                srv_name='get_pose',
                srv_in_type=GetPose,
                srv_out_type=GetPoseResponse,
            )

        def _callback(self, req):
            res = GetPoseResponse()
            x, y, theta = turtle.get_pose()
            return GetPoseResponse(x, y, theta)

    class ServiceIsMoving(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceIsMoving, self).__init__(
                srv_name='is_moving',
                srv_in_type=IsMoving,
                srv_out_type=IsMovingResponse,
            )

        def _callback(self, req):
            is_moving = turtle.is_moving()
            return IsMovingResponse(is_moving)


def main():

    rospy.init_node(NODE_NAME)
    rospy.loginfo("Node starts: " + NODE_NAME)

    # Define global variables.
    global turtle, SRV_NAMESPACE
    turtle = Turtle(CONFIG_FILEPATH)
    SRV_NAMESPACE = read_yaml_file(CONFIG_FILEPATH)["srv_namespace"]

    # ROS Node deconstructor.
    rospy.on_shutdown(lambda: turtle.stop_moving())

    # Start ROS services.
    turtle_services = TurtlebotControlRosServices()
    turtle_services.start()

    # Loop.
    rospy.spin()
    rospy.loginfo("Node stops: " + NODE_NAME)


if __name__ == "__main__":
    main()
