'''
Main classes and functions:

* The class that represents the turtlebot:
    class Turtle

* The main control algorithm is implemented in:
    def _control_robot_to_reach_pose

'''

#!/usr/bin/env python

''' -------------------------------------------------------------------------- '''

# Common libraries


# ROS

# Geometric transformations

# Messages


''' -------------------------------------------------------------------------- '''

import argparse
import numpy as np
import sys
import math
import yaml
import time
import sys
import os
import rospy
import roslib
import rospy
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import rotation_matrix, quaternion_from_matrix
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Pose, Twist, Vector3
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
ROOT = os.path.dirname(os.path.abspath(__file__))+"/"

''' -------------------------------------------------------------------------- '''


class Math:
    # Mathmatical operations

    @staticmethod
    def xytheta_to_T(x, y, theta):
        ''' Convert robot 2D pose (x, y, theta)
            to 3x3 transformation matrix '''
        c = np.cos(theta)
        s = np.sin(theta)
        T = np.array([
            [c, -s, x, ],
            [s,  c, y, ],
            [0,  0, 1, ],
        ])
        return T

    @staticmethod
    def T_to_xytheta(T):
        ''' Convert 3x3 transformation matrix
            to robot 2D pose (x, y, theta) '''
        assert T.shape == (3, 3)
        x = T[0, 2]
        y = T[1, 2]
        s, c = T[1, 0], T[0, 0]
        theta = np.arctan2(s, c)
        return x, y, theta

    @staticmethod
    def _euler_from_quaternion(quat_xyzw):
        ''' An overload of tf.transformations.euler_from_quaternion.
        Argument:
            quat_xyzw {list, np.ndarray, or geometry_msgs.msg.Quaternion}.
        Output:
            euler_xyz {np.ndarray}: [euler_x, euler_y, euler_z].
        '''
        def convert_quaternion_data_to_list(quat_xyzw):
            if type(quat_xyzw) != list and type(quat_xyzw) != np.ndarray:
                quat_xyzw = [quat_xyzw.x, quat_xyzw.y,
                             quat_xyzw.z, quat_xyzw.w]
            return quat_xyzw
        quat_xyzw = convert_quaternion_data_to_list(quat_xyzw)
        euler_xyz = euler_from_quaternion(quat_xyzw, 'rxyz')
        return euler_xyz

    @staticmethod
    def pose_to_xytheta(pose):
        '''
        Argument:
            pose {geometry_msgs.msg}: Robot pose.
        Output:
            x, y, theta
        '''
        x = pose.position.x
        y = pose.position.y
        euler_xyz = Math._euler_from_quaternion(pose.orientation)
        theta = euler_xyz[2]
        return x, y, theta

    @staticmethod
    def theta_to_rotation_matrix(theta):
        '''
        Arguments:
            theta {float}: robot orientation in world frame.
        Output:
            R {np.ndarray: 4x4 rotation matrix}
        '''
        Z_AXIS = (0, 0, 1)
        R = rotation_matrix(theta, Z_AXIS)
        return R

    @staticmethod
    def theta_to_quaternion(theta):
        '''
        Arguments:
            theta {float}: robot orientation in world frame.
        Output:
            q {geometry_msgs.msg.Quaternion}
        '''
        R = Math.theta_to_rotation_matrix(theta)
        q_list = quaternion_from_matrix(R)

        def list_to_ros_quaternion(l):
            q = Quaternion(l[0], l[1], l[2], l[3])
            return q
        q = list_to_ros_quaternion(q_list)
        return q

    @staticmethod
    def calc_dist(x1, y1, x2, y2):
        return ((x1 - x2)**2 + (y1 - y2)**2)**(0.5)

    @staticmethod
    def pi2pi(theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi


''' -------------------------------------------------------------------------- '''


class SimpleNamespace:
    ''' This is the same as `from type import SimpleNamespace` in Python3 '''

    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

    def __repr__(self):
        keys = sorted(self.__dict__)
        items = ("{}={!r}".format(k, self.__dict__[k]) for k in keys)
        return "{}({})".format(type(self).__name__, ", ".join(items))

    def __eq__(self, other):
        return self.__dict__ == other.__dict__


def dict2class(d):
    ''' Convert a dictionary to a class.
    The keys in the dictionary must be the type `str`.
    '''
    args = SimpleNamespace()
    args.__dict__.update(**d)
    return args


def ReadYamlFile(filepath):
    ''' Read contents from the yaml file.
    Output:
        data_loaded {dict}: contents of the yaml file.
            The keys of the dict are `str` type.
    '''
    with open(filepath, 'r') as stream:
        data_loaded = yaml.safe_load(stream)
    return data_loaded


''' -------------------------------------------------------------------------- '''


def call_ros_service(service_name, service_type, service_args=None):
    ''' Call a ROS service.
    '''
    rospy.wait_for_service(service_name)
    try:
        func = rospy.ServiceProxy(service_name, service_type)
        func(*service_args) if service_args else func()
    except rospy.ServiceException as e:
        print("Failed to call service:", service_name)
        sys.exit()


''' -------------------------------------------------------------------------- '''


class PidController(object):
    ''' PID controller '''

    def __init__(self, T, P=0.0, I=0.0, D=0.0):
        ''' Arguments
        T {float}: Control period. Unit: second.
            This is the inverse of control frequency.
        P {float or np.array}: Proportional control coefficient.
        I {float or np.array}: Integral control coefficient.
        D {float or np.array}: Differential control coefficient.
        '''

        # -- Check input data
        b1 = all(isinstance(d, float) for d in [P, I, D])
        b2 = all(isinstance(d, np.ndarray) for d in [P, I, D])
        if not b1 and not b2:
            pid_coef_types = [type(d) for d in [P, I, D]]
            err_msg = "PidController: Data type of P,I,D coefficients "\
                "are wrong: " + str(pid_coef_types)
            raise RuntimeError(err_msg)
        dim = 1 if b1 else len(P)  # Dimension of the control variable

        # -- Initialize arguments.
        self._T = T
        self._P = np.zeros(dim)+P
        self._I = np.zeros(dim)+I
        self._D = np.zeros(dim)+D
        self._err_inte = np.zeros(dim)  # Integration error.
        self._err_prev = np.zeros(dim)  # Previous error.

    def compute(self, err):
        ''' Given the error, compute the desired control value . '''

        ctrl_val = 0
        err = np.array(err)

        # P
        ctrl_val += np.dot(err, self._P)

        # I
        self._err_inte += err
        ctrl_val += self._T * np.dot(self._err_inte, self._I)

        # D
        ctrl_val += np.dot(err-self._err_prev, self._D) / self._T

        self._err_prev = err
        return ctrl_val


''' -------------------------------------------------------------------------- '''


class Turtle(object):
    ''' A `turtle` class which represents the turtlebot,
        and provides the APIs for controlling turtlebot.
    '''

    def __init__(self,
                 config_filepath=ROOT + "config/config.yaml"):

        # Read configurations from yaml file.
        self._cfg = dict2class(ReadYamlFile(config_filepath))
        self._cfg_ctrl = dict2class(self._cfg.control_settings)

        # Publisher.
        self._pub_speed = rospy.Publisher(
            self._cfg.topic_set_turlte_speed, Twist, queue_size=10)

        # Subscriber.
        if self._cfg.is_in_simulation:
            self._sub_pose = rospy.Subscriber(
                self._cfg.topic_get_turtle_speed_env_sim,
                ModelStates, self._callback_sub_pose_env_sim)
        else:
            self._sub_pose = rospy.Subscriber(
                self._cfg.topic_get_turtle_speed_env_real,
                Odometry, self._callback_sub_pose_env_real)

        # Robot state.
        self._time0 = self._reset_time()
        self._pose = Pose()
        self._twist = Twist()

    def reset_pose(self, sleep_time=0.1):
        ''' Reset Robot pose.
        If in simulation, reset the simulated robot pose to (0, 0, 0).
        If real robot, reset the odometry and IMU data to zero.
        '''
        rospy.loginfo("Resetting robot state...")
        if self._cfg.is_in_simulation:
            self._reset_pose_env_sim()
        else:
            self._reset_pose_env_real()
        rospy.sleep(sleep_time)
        rospy.loginfo("Reset robot state completes")

    def set_pose(self, x, y, theta, sleep_time=0.1):
        ''' Set Robot pose.
        If in simulation, reset the simulated robot pose.
        If real robot, this function does nothing !!!
        '''
        if self._cfg.is_in_simulation:
            rospy.loginfo("Setting robot state...")
            self._set_pose_env_sim(x, y, theta)
        else:
            raise RuntimeError(
                "Set real robot's pose is not supported.")  # TODO
            # self._set_pose_env_real(x, y, theta)
        rospy.sleep(sleep_time)
        rospy.loginfo("Set robot state completes")

    def set_speed(self, v, w):
        ''' Set robot speed.
        Arguments:
            v {float}: linear speed.
            w {float}: angular speed.
        '''
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self._pub_speed.publish(twist)

    def get_pose(self):
        x, y, theta = Math.pose_to_xytheta(self._pose)
        return x, y, theta

    def print_state(self, x, y, theta, v=np.nan, w=np.nan):
        ''' Print the robot pose and speed. '''
        print("Robot pose: "
              "x = {:.3f}, "
              "y = {:.3f}, "
              "theta = {:.3f}, "
              "v = {:.3f}, "
              "w = {:.3f}".format(
                  x, y, theta, v, w))

    def move_a_circle(self, v=0.1, w=0.1):
        ''' Control the turtlebot to move in a circle
            until the program stops.
        '''
        while not rospy.is_shutdown():
            self._set_twist(v, w)

            # Print state
            x, y, theta = self._get_pose()
            self._print_state(x, y, theta, v, w)
            print("Moving in circle ...")
            rospy.sleep(0.5)
        return True

    def move_forward(self, v=0.1):
        ''' Control the turtlebot to move forward
            until the program stops.
        '''
        w = 0
        while not rospy.is_shutdown():
            self._set_twist(v, w)

            # Print state
            x, y, theta = self._get_pose()
            self._print_state(x, y, theta, v, w)
            print("Moving forward ...")
            rospy.sleep(0.5)
        return True

    def move_to_pose(self, x_goal_w, y_goal_w, theta_goal_w=None):
        '''
        Control the turtlebot to move towards the target pose (Absolute pose).
            This function returns after the robot gets very close to the target.
        Arguments:
            x_goal_r {float}
            y_goal_r {float}
            theta_goal_r {float or None}
        The input arguments are the target pose represented in the world frame.
        '''

        print("\nMove robot to the global pose: {}, {}, {}\n".format(
            x_goal_w, y_goal_w, theta_goal_w))

        self._control_robot_to_reach_pose(x_goal_w, y_goal_w, theta_goal_w)

        return True

    def move_to_relative_pose(self, x_goal_r, y_goal_r, theta_goal_r=None):
        '''
        Control the turtlebot to move towards the target pose (Relative pose).
            This function returns after the robot gets very close to the target
        Arguments:
            x_goal_r {float}
            y_goal_r {float}
            theta_goal_r {float or None}
        The input arguments are the target pose represented in the robot frame,
            where x is front, y is left, and theta is from x to y.
        '''

        # Convert target pose from robot frame to world frame.
        x_goal_w, y_goal_w, theta_goal_w = self._pose_robot2world(
            x_goal_r,
            y_goal_r,
            theta_goal_r if theta_goal_r is not None else 0.0)
        if theta_goal_r is None:
            theta_goal_w = None

        # Move.
        print("\nMove robot to the global pose: {}, {}, {}\n".format(
            x_goal_w, y_goal_w, theta_goal_w))
        self._control_robot_to_reach_pose(x_goal_w, y_goal_w, theta_goal_w)

        return True

    def _pose_robot2world(self, x_rg, y_rg, theta_rg):
        '''
        Transform the coordinate of a pose `g` (x_rg, y_rg, theta_rg):
            from:   robot_frame's (X_rg)
            to:     world_frame's (X_wg)
        '''
        x_wr, y_wr, theta_wr = self.get_pose()
        print(x_wr, y_wr, theta_wr)  # feiyu
        T_wr = Math.xytheta_to_T(x_wr, y_wr, theta_wr)  # T_world_to_robot
        T_rg = Math.xytheta_to_T(x_rg, y_rg, theta_rg)  # T_robot_to_goal
        T_wg = np.dot(T_wr, T_rg)
        x_wg, y_wg, theta_wg = Math.T_to_xytheta(T_wg)
        return x_wg, y_wg, theta_wg

    def _control_robot_to_reach_pose(
            self, x_goal, y_goal, theta_goal=None):
        '''
        Control the turlebot to the target pose.
        This function returns after the robot gets very close to the target
            specified by the (x_tol, y_tol, theta_tol).

        If theta_goal is None,
            the robot is drived to the point (x_goal, y_goal),
            by setting p_beta=0 in the algorithm.
        If theta_goal is a valid float,
            the robot is drived to the pose  (x_goal, y_goal, theta_goal).

        Arguments:
            cfg_ctrl {SimpleNamespace}: Parameters for control. 
                This is read from the config file.

        Reference: page 129 of the book "Robotics, Vision, and Control".
        '''

        # =======================================
        # Get configurations
        #   See `config/config.yaml` and the content of `control_settings`.
        cfg = self._cfg_ctrl

        # -- PID control
        # Control period
        T = cfg.control_period  # s

        # Drive the robot closer to the target point.
        p_rho = cfg.p_rho

        # Rotate the robot direction towards the target point.
        p_alpha = cfg.p_alpha

        # Rotate the robot orientation towards the target orientation.
        p_beta = 0.0 if theta_goal is None else cfg.p_beta

        # Pure spin
        p_spin = cfg.p_spin

        # -- Flags
        is_theta_considered = theta_goal is not None
        is_pure_spin = False

        # -- Others
        max_v = cfg.max_v  # m/s
        max_w = cfg.max_w  # rad/s
        theta_goal = 0.0 if theta_goal is None else Math.pi2pi(theta_goal)

        # =======================================
        # Init PID controllers
        loop_control = rospy.Rate(1.0 / T)
        pid_rho = PidController(T, P=p_rho)
        pid_alpha = PidController(T, P=p_alpha)
        pid_beta = PidController(T, P=p_beta)

        # =======================================
        # Loop and control
        cnt_steps = 0
        while not rospy.is_shutdown():
            cnt_steps += 1

            x, y, theta = self.get_pose()

            rho = Math.calc_dist(x, y, x_goal, y_goal)
            alpha = Math.pi2pi(math.atan2(y_goal - y, x_goal - x) - theta)
            beta = Math.pi2pi(theta_goal - theta) - alpha

            # Check moving direction
            sign = 1
            if abs(alpha) > math.pi/2:  # the goal is behind the robot
                alpha = Math.pi2pi(math.pi - alpha)
                beta = Math.pi2pi(math.pi - beta)
                # alpha = Math.pi2pi(alpha)
                # beta = Math.pi2pi(beta)
                sign = -1

            # PID control
            val_rho = pid_rho.compute(err=rho)[0]
            val_alpha = pid_alpha.compute(err=alpha)[0]
            val_beta = pid_beta.compute(err=beta)[0]

            # Pure spin:
            #   If the robot is very close to the (x_goal, y_goal),
            #   enalbe pure spin.
            if is_pure_spin:
                sign = 1
                val_rho = 0
                val_alpha = 0
                val_beta = Math.pi2pi(theta_goal - theta) * p_spin
            else:
                if self.is_close_to_target(x_goal, y_goal):
                    is_pure_spin = True

            # Get desired speed
            v = sign * val_rho
            w = sign * (val_alpha + val_beta)

            # Threshold on velocity
            v = min(abs(v), max_v) * \
                (1 if v > 0 else -1)  # limit v
            w = min(abs(w), max_w) * \
                (1 if w > 0 else -1)  # limit w

            # Output
            self.set_speed(v, w)  # Output speed.
            if cnt_steps % 10 == 0:  # Print robot pose and velocity.
                self.print_state(x, y, theta, v, w)
                print("\trho = {:.3f}, alpha = {:.3f}, beta = {:.3f}".format(
                    val_rho, val_alpha, val_beta))

            # Check stop condition
            if self.is_close_to_target(x_goal, y_goal, theta_goal):
                break

            loop_control.sleep()

        self.set_speed(v=0, w=0)
        print("Reach the target. Control completes.\n")
        print("\tGoal: x = {:.3f}, y = {:.3f}, theta = {}\n".format(
            x_goal, y_goal, theta_goal))

    def is_close_to_target(
            self,
            x_goal, y_goal,
            theta_goal=None,  # If None, not consider theta.
            x_tol=None, y_tol=None, theta_tol=None):

        # -- Check input
        if x_tol is None:
            x_tol = self._cfg_ctrl.x_tol

        if y_tol is None:
            y_tol = self._cfg_ctrl.y_tol

        if theta_tol is None:
            theta_tol = self._cfg_ctrl.theta_tol

        # -- Check if the robot is close to the target
        x, y, theta = self.get_pose()
        b1 = True if x_goal is None else abs(x - x_goal) < x_tol
        b2 = True if y_goal is None else abs(y - y_goal) < y_tol
        b3 = True if theta_goal is None else \
            abs(Math.pi2pi(theta - theta_goal)) < theta_tol
        return b1 and b2 and b3

    def _reset_pose_env_real(self):
        ''' Reset the robot pose (For real robot mode).
        This is the same as the terminal command:
            $ rostopic pub /reset std_msgs/Empty  '{}'
        '''
        if self._cfg.is_in_simulation:
            raise RuntimeError(
                "In `simulation` mode, this function shouldn't be called.")
        reset_odom = rospy.Publisher(
            self._cfg.topic_reset_pose_env_real,
            Empty, queue_size=10)
        reset_odom.publish(Empty())

    def _reset_pose_env_sim(self):
        ''' Reset the robot pose (For simulation mode).
        This is the same as the terminal command:
            $ rostopic pub /gazebo/set_model_state gazebo_msgs/ModelState \
            '{  model_name: turtlebot3_waffle_pi, \
                pose: {     position: { x: 0, y: 0, z: 0 }, \
                            orientation: {x: 0, y: 0, z: 0, w: 1 } }, \
                twist: {    linear: { x: 0, y: 0, z: 0 }, \
                            angular: { x: 0, y: 0, z: 0}  }, \
                reference_frame: world }'
        '''
        self._set_pose_env_sim(x=0.0, y=0.0, theta=0.0)

    def _set_pose_env_real(self, x, y, theta):
        raise RuntimeError("Set real robot's pose is not supported.")  # TODO

    def _set_pose_env_sim(self, x, y, theta):

        # Sanity check.
        if not self._cfg.is_in_simulation:
            raise RuntimeError(
                "In `real robot` mode, this function shouldn't be called.")

        # Set goal state.
        p = Point(x=x, y=y, z=0)
        q = Math.theta_to_quaternion(theta)
        state = ModelState(
            pose=Pose(position=p, orientation=q),
            twist=Twist(),
            model_name=self._cfg.turtle_name,
            reference_frame=self._cfg.ref_frame)

        # Call ROS service to set robot pose.
        call_ros_service(
            service_name=self._cfg.srv_reset_pose_env_sim,
            service_type=SetModelState,
            service_args=(state, )
        )

    def _callback_sub_pose_env_sim(self, model_states):
        ''' ROS topic subscriber's callback function
            for receiving and updating robot pose when running simulation.
        '''
        idx = model_states.name.index(self._cfg.turtle_name)
        self._pose = model_states.pose[idx]
        self._twist = model_states.twist[idx]

    def _callback_sub_pose_env_real(self, odometry):
        ''' ROS topic subscriber's callback function
            for receiving and updating robot pose when running robot.
        '''
        # Contents of odometry:
        #   frame_id: "odom"
        #   child_frame_id: "base_footprint"
        self._pose = odometry.pose.pose
        self._twist = odometry.twist.twist

    def _reset_time(self):
        self._time0 = rospy.get_time()
        return self._time0

    def _query_time(self):
        return rospy.get_time() - self._time0
