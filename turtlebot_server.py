#!/usr/bin/env python

''' -------------------------------------------------------------------------- '''

from ros_turtlebot_control.srv import ResetPose, ResetPoseResponse
from ros_turtlebot_control.srv import SetPose, SetPoseResponse
from ros_turtlebot_control.srv import StopMoving, StopMovingResponse
from ros_turtlebot_control.srv import MoveToRelativePose, MoveToRelativePoseResponse
from ros_turtlebot_control.srv import MoveToRelativePoint, MoveToRelativePointResponse
from ros_turtlebot_control.srv import MoveToPose, MoveToPoseResponse
from ros_turtlebot_control.srv import MoveToPoint, MoveToPointResponse
import rospy
import threading
import yaml
import os
import sys
from utils.turtle import Turtle
from utils.commons import read_yaml_file


ROOT = os.path.dirname(os.path.abspath(__file__))+"/"
CONFIG_FILEPATH = ROOT + "config/config.yaml"
NODE_NAME = 'run_turtlebot_control_server'
SRV_NAMESPACE, turtle = None, None  # To be initialized later.


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


def start_thread_with_no_args(func):
    thread = threading.Thread(
        target=func,
        args=[])
    thread.start()


class TurtlebotControlRosServices(object):

    def start(self):
        self._h1 = TurtlebotControlRosServices.ServiceMoveToPoint()
        self._h2 = TurtlebotControlRosServices.ServiceMoveToPose()
        self._h3 = TurtlebotControlRosServices.ServiceMoveToRelativePoint()
        self._h4 = TurtlebotControlRosServices.ServiceMoveToRelativePose()
        self._h5 = TurtlebotControlRosServices.ServiceStopMoving()
        self._h6 = TurtlebotControlRosServices.ServiceSetPose()
        self._h7 = TurtlebotControlRosServices.ServiceResetPose()

    class ServiceMoveToPoint(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceMoveToPoint, self).__init__(
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

    class ServiceMoveToPose(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceMoveToPose, self).__init__(
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

    class ServiceMoveToRelativePoint(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceMoveToRelativePoint, self).__init__(
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

    class ServiceMoveToRelativePose(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceMoveToRelativePose, self).__init__(
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

    class ServiceStopMoving(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceStopMoving, self).__init__(
                srv_name='stop_moving',
                srv_in_type=StopMoving,
                srv_out_type=StopMovingResponse,
            )

        def _callback(self, req):
            turtle.stop_moving()
            rospy.loginfo("Service: " + self._srv_name + ": is completed!")
            return self._srv_out_type()

    class ServiceSetPose(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceSetPose, self).__init__(
                srv_name='set_pose',
                srv_in_type=SetPose,
                srv_out_type=SetPoseResponse,
            )

        def _callback(self, req):
            turtle.set_pose(req.x, req.y, req.theta)
            rospy.loginfo("Service: " + self._srv_name + ": is completed!")
            return self._srv_out_type()

    class ServiceResetPose(_SrvTemplate):
        def __init__(self):
            super(TurtlebotControlRosServices.ServiceResetPose, self).__init__(
                srv_name='reset_pose',
                srv_in_type=ResetPose,
                srv_out_type=ResetPoseResponse,
            )

        def _callback(self, req):
            turtle.reset_pose()
            rospy.loginfo("Service: " + self._srv_name + ": is completed!")
            return self._srv_out_type()


def main():

    rospy.init_node(NODE_NAME)
    rospy.loginfo("Node starts: " + NODE_NAME)

    # Vars.
    global turtle, SRV_NAMESPACE
    turtle = Turtle(CONFIG_FILEPATH)
    SRV_NAMESPACE = read_yaml_file(CONFIG_FILEPATH)["srv_namespace"]

    # Shutdown function, which is executed right before the node stops.
    rospy.on_shutdown(lambda: turtle.set_speed(v=0, w=0))

    # Init ROS service servers.
    turtle_services = TurtlebotControlRosServices()
    turtle_services.start()

    # Loop.
    rospy.spin()
    rospy.loginfo("Node stops: " + NODE_NAME)


if __name__ == "__main__":
    main()
