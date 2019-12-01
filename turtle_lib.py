#!/usr/bin/env python

''' 
A `Turtle` class for controlling Turtlebot3.
'''


import utils.geo_maths as geo_maths
from utils.commons import dict2class, read_yaml_file
from utils.pid_controller import PidController

from geometry_msgs.msg import Point, Pose, Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion
import tf

import numpy as np
import rospy
import sys
import math
import threading
import collections


def call_ros_service(service_name, service_type, service_args=None):
    ''' Call a ROS service.
    '''
    rospy.wait_for_service(service_name)
    try:
        func = rospy.ServiceProxy(service_name, service_type)
        resp = func(*service_args) if service_args else func()
        return True, resp
    except:
        rospy.logwarn("Failed to call service:", service_name)
        return False, None
    # except rospy.ServiceException as e:
    #     print("Failed to call service:", service_name)
    #     sys.exit()


class Trajectory(object):
    ''' To store the path poses of the robot. '''

    def __init__(self, buffer_size=20):
        self._q_x = collections.deque()
        self._q_y = collections.deque()
        self._q_theta = collections.deque()
        self._buffer_size = buffer_size

    def add_pose(self, x, y, theta):
        self._q_x.append(x)
        self._q_y.append(y)
        self._q_theta.append(theta)
        if len(self._q_x) > self._buffer_size:
            self._q_x.popleft()
            self._q_y.popleft()
            self._q_theta.popleft()

    def is_static(self, thresh_std_variance=0.0001):
        ''' If the standard variance of the historical data
            is smaller than threshold, than the robot is static.
        '''
        if len(self._q_x) < self._buffer_size:
            return False
        std_var = (np.std(self._q_x) + np.std(self._q_y) +
                   np.std(self._q_theta))
        return std_var < thresh_std_variance


class _TurtleDecorators(object):

    @staticmethod
    def manage_moving_state(func_to_control_turtlebot):
        ''' 
        Function:
            If the turtle is executing an existing control loop, stops it.
            Then, set `_is_moving` to true,
                and start a new thread to control the robot.
            After robot completes moving, set `_is_moving` to false.

        Usage: This decorator should be added to 
            all Turtle's public motion control functions. 
        '''

        def new_func_to_control_turtlebot(self, *args, **kwargs):

            # Stop current control loop.
            if self._is_moving:
                rospy.logwarn("Wait for previous control loop to stop ...")
                self.stop_moving()
                rospy.logwarn("Previous control loop has stopped.")

            # Run the control in a new thread.
            def _thread_func():
                self._is_moving = True
                func_to_control_turtlebot(self, *args, **kwargs)
                self._is_moving = False
            thread = threading.Thread(
                target=_thread_func, args=[])
            thread.start()

        return new_func_to_control_turtlebot


class Turtle(object):

    def __init__(self,
                 config_filepath="config.yaml"):

        # -- Read configurations from yaml file.
        self._cfg = dict2class(read_yaml_file(config_filepath))
        self._cfg_ctrl = dict2class(self._cfg.control_settings)

        # -- Turtle speed publisher.
        self._pub_speed = rospy.Publisher(
            self._cfg.topic_set_turlte_speed, Twist, queue_size=10)

        # -- Turtle speed tf inquiry.
        self._tf_listener = tf.TransformListener()
        self._odom_frame = self._cfg.odom_frame
        self._base_frame = self._cfg.base_frame
        # self._setup_pose_subscriber()  # Deprecated. Don't use.

        # -- Initialize robot state.

        # Moving state: for starting/stopping control loop in multi threading.
        self._is_moving = False
        self._enable_moving = True

    def stop_moving(self):
        if self._is_moving:
            self._enable_moving = False
            while (not rospy.is_shutdown()) and self._is_moving:
                rospy.sleep(0.001)
            self._enable_moving = True

    def is_moving(self):
        return self._is_moving

    def is_stopped(self):
        return not self._is_moving

    def wait_until_stop(self):
        while self._is_moving:
            rospy.sleep(0.1)

    def get_pose(self):
        try:
            (trans, rot) = self._tf_listener.lookupTransform(
                self._odom_frame,
                self._base_frame,
                rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception in Turtle::get_pose()")
            return
        x, y, theta = trans[0], trans[1], rotation[2]
        return x, y, theta

    def set_pose(self, x, y, theta, sleep_time=0.1):
        ''' Set Robot pose (Only supported for simulation.)
        If in simulation, reset the simulated robot pose.
        If real robot, this function throws error!
        '''
        rospy.loginfo("Setting robot state...")
        self.stop_moving()
        is_succeed = self._set_pose_for_simulation(x, y, theta)
        if not is_succeed:
            e = "Set pose failed. Setting pose only supports for simulation, not real robot!"
            rospy.logerr(e)
            raise RuntimeError(e)
        rospy.sleep(sleep_time)
        rospy.loginfo("Set robot state completes")

    def reset_pose(self, sleep_time=0.1):
        ''' Reset Robot pose.
        If in simulation, reset the simulated robot pose to (0, 0, 0).
        If real robot, reset the odometry and IMU data to zero.
        '''
        rospy.loginfo("Resetting robot state...")
        self.stop_moving()

        # Try the resetting command for simulation.
        is_succeed = self._set_pose_for_simulation(x=0.0, y=0.0, theta=0.0)
        mode = "simulation" if is_succeed else "real_robot"
        if mode == "simulation":
            rospy.loginfo("Reset robot pose in simulation... Done.")

        else:
            # Try the resetting command for real robot.
            reset_odom = rospy.Publisher(
                self._cfg.reset_pose_for_real_robot["topic_reset_pose_for_real_robot"],
                Empty, queue_size=10)
            reset_odom.publish(Empty())
            rospy.loginfo("Reset robot pose for the real robot... Done.")

        rospy.sleep(sleep_time)
        if not self.is_close_to_target(x_goal=0, y_goal=0, theta_goal=0):
            raise RuntimeError(
                "Reset robot pose failed for {} mode".format(mode))

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

    def print_state(self, x, y, theta, v=np.nan, w=np.nan):
        ''' Print the robot pose and speed. '''
        print("Robot pose: "
              "x = {:.3f}, "
              "y = {:.3f}, "
              "theta = {:.3f}, "
              "v = {:.3f}, "
              "w = {:.3f}".format(
                  x, y, theta, v, w))

    @_TurtleDecorators.manage_moving_state
    def move_a_circle(self, v=0.1, w=0.1):
        ''' Control the turtlebot to move in a circle
            until the program stops.
        '''
        while not rospy.is_shutdown() and self._enable_moving:
            self._set_twist(v, w)

            # Print state
            x, y, theta = self._get_pose()
            self._print_state(x, y, theta, v, w)
            print("Moving in circle ...")
            rospy.sleep(0.5)
        return True

    @_TurtleDecorators.manage_moving_state
    def move_forward(self, v=0.1):
        ''' Control the turtlebot to move forward
            until the program stops.
        '''
        w = 0
        while not rospy.is_shutdown() and self._enable_moving:
            self._set_twist(v, w)

            # Print state
            x, y, theta = self._get_pose()
            self._print_state(x, y, theta, v, w)
            print("Moving forward ...")
            rospy.sleep(0.5)
        return True

    @_TurtleDecorators.manage_moving_state
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

        self._control_robot_to_pose(x_goal_w, y_goal_w, theta_goal_w)

        return True

    @_TurtleDecorators.manage_moving_state
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
        self._control_robot_to_pose(x_goal_w, y_goal_w, theta_goal_w)

        return True

    def is_close_to_target(
            self,
            x_goal=None,
            y_goal=None,
            theta_goal=None,
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
            abs(geo_maths.pi2pi(theta - theta_goal)) < theta_tol
        return b1 and b2 and b3

    def _pose_robot2world(self, x_rg, y_rg, theta_rg):
        '''
        Transform the coordinate of a pose `g` (x_rg, y_rg, theta_rg):
            from:   robot_frame's (X_rg)
            to:     world_frame's (X_wg)
        '''
        x_wr, y_wr, theta_wr = self.get_pose()
        print(x_wr, y_wr, theta_wr)  # feiyu
        T_wr = geo_maths.xytheta_to_T(x_wr, y_wr, theta_wr)  # T_world_to_robot
        T_rg = geo_maths.xytheta_to_T(x_rg, y_rg, theta_rg)  # T_robot_to_goal
        T_wg = np.dot(T_wr, T_rg)
        x_wg, y_wg, theta_wg = geo_maths.T_to_xytheta(T_wg)
        return x_wg, y_wg, theta_wg

    def _control_robot_to_pose(
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

        rospy.loginfo("Control: Start moving to global pose: {}, {}, {}\n".format(
            x_goal, y_goal, theta_goal))

        # =======================================
        # Get configurations
        #   See `config.yaml` and the content of `control_settings`.
        cfg = self._cfg_ctrl

        # -- PID control
        # Control period
        T = cfg.control_period  # s

        # Drive the robot closer to the target point.
        P_RHO = cfg.p_rho  # Coef for proportion control.

        # Rotate the robot direction towards the target point.
        P_ALPHA = cfg.p_alpha  # Coef for proportion control.

        # Rotate the robot orientation towards the target orientation.
        P_BETA = 0.0 if theta_goal is None else cfg.p_beta

        # Pure spin
        P_SPIN = cfg.p_spin  # Coef for proportion control.

        # -- Flags
        # If True, the robot drives to target pose (with theta).
        # If False, the robot drives to target point.
        IS_THETA_CONSIDERED = theta_goal is not None

        # Enable pure spin after reaching the target point.
        is_pure_spin = False

        # -- Others
        max_v = cfg.max_v  # m/s
        max_w = cfg.max_w  # rad/s
        theta_goal = 0.0 if theta_goal is None else geo_maths.pi2pi(theta_goal)
        traj = Trajectory(buffer_size=10)

        # =======================================
        # Init PID controllers
        loop_control = rospy.Rate(1.0 / T)
        pid_rho = PidController(T, P=P_RHO)
        pid_alpha = PidController(T, P=P_ALPHA)
        pid_beta = PidController(T, P=P_BETA)

        # =======================================
        # Loop and control
        cnt_steps = 0
        while not rospy.is_shutdown() and self._enable_moving:
            cnt_steps += 1

            x, y, theta = self.get_pose()

            rho = geo_maths.calc_dist(x, y, x_goal, y_goal)
            alpha = geo_maths.pi2pi(math.atan2(y_goal - y, x_goal - x) - theta)
            beta = geo_maths.pi2pi(theta_goal - theta) - alpha

            # Check moving direction
            sign = 1
            if abs(alpha) > math.pi/2:  # the goal is behind the robot
                alpha = geo_maths.pi2pi(math.pi - alpha)
                beta = geo_maths.pi2pi(math.pi - beta)
                # alpha = geo_maths.pi2pi(alpha)
                # beta = geo_maths.pi2pi(beta)
                sign = -1

            # PID control
            val_rho = pid_rho.compute(err=rho)[0]
            val_alpha = pid_alpha.compute(err=alpha)[0]
            val_beta = pid_beta.compute(err=beta)[0]

            # Pure spin:
            #   If the robot is very close to the (x_goal, y_goal),
            #   enalbe pure spin.
            if not is_pure_spin:
                if self.is_close_to_target(x_goal, y_goal):
                    is_pure_spin = True
            if is_pure_spin and IS_THETA_CONSIDERED:
                sign = 1  # Default to be forward.
                val_rho = 0  # No linear motion.
                val_alpha = 0  # No rotating towards target point.
                # Rotate towards target orientation.
                val_beta = geo_maths.pi2pi(theta_goal - theta) * P_SPIN

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

            # Print robot pose and velocity.
            if self._cfg_ctrl.is_print_current_state and cnt_steps % 25 == 0:
                self.print_state(x, y, theta, v, w)
                print("\trho = {:.3f}, alpha = {:.3f}, beta = {:.3f}".format(
                    val_rho, val_alpha, val_beta))
            if cnt_steps % 5 == 0:
                traj.add_pose(x, y, theta)
                if traj.is_static():
                    msg_warn = "Control: The control output is too week. Stopped."
                    rospy.logwarn(msg_warn)
                    break
            # Check stop condition
            if self.is_close_to_target(
                    x_goal,
                    y_goal,
                    theta_goal if IS_THETA_CONSIDERED else None):
                break

            loop_control.sleep()

        self.set_speed(v=0, w=0)
        str_target = "x = {:.3f}, y = {:.3f}, theta = {}".format(
            x_goal, y_goal, theta_goal)

        if self._enable_moving:
            rospy.loginfo("Control: Reach the target: " + str_target)
            rospy.loginfo("Control: Completes.")
        else:
            rospy.logwarn("Control interrupted. Target ({}) is canceled.".format(
                str_target))

    def _set_pose_for_simulation(self, x, y, theta):

        # Set goal state.
        p = Point(x=x, y=y, z=0)
        q = geo_maths.theta_to_quaternion(theta)
        state = ModelState(
            pose=Pose(position=p, orientation=q),
            twist=Twist(),
            model_name=self._cfg.set_pose_in_simulation["model_name"],
            reference_frame=self._cfg.set_pose_in_simulation["reference_frame"])

        # Call ROS service to set robot pose.
        is_succeed, response = call_ros_service(
            service_name=self._cfg.set_pose_in_simulation["srv_set_pose"],
            service_type=SetModelState,
            service_args=(state, )
        )
        return is_succeed

    # =============================== Deprecated functions ===============================

    def _setup_pose_subscriber(self):
        if self._cfg.is_in_simulation:
            self._sub_pose = rospy.Subscriber(
                self._cfg.topic_get_turtle_speed_env_sim,
                ModelStates, self._callback_sub_pose_env_sim)
        else:
            self._sub_pose = rospy.Subscriber(
                self._cfg.topic_get_turtle_speed_env_real,
                Odometry, self._callback_sub_pose_env_real)
