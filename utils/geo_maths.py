''' Math and geometrical transformations.
'''

import numpy as np
import math

from tf.transformations import rotation_matrix
from tf.transformations import euler_from_quaternion, quaternion_from_matrix
from geometry_msgs.msg import Pose, Quaternion


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


def T_to_xytheta(T):
    ''' Convert 3x3 transformation matrix
        to robot 2D pose (x, y, theta) '''
    if T.shape != (3, 3):
        raise RuntimeError("Shape of T should be (3, 3).")
    x = T[0, 2]
    y = T[1, 2]
    s, c = T[1, 0], T[0, 0]
    theta = np.arctan2(s, c)
    return x, y, theta


def euler_from_quaternion_enhanced(quat_xyzw):
    ''' An overload of tf.transformations.euler_from_quaternion,
        with support of multiply input data type.
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


def pose_to_xytheta(pose):
    '''
    Argument:
        pose {geometry_msgs.msg}: Robot pose.
    Output:
        x, y, theta
    '''
    x = pose.position.x
    y = pose.position.y
    euler_xyz = euler_from_quaternion_enhanced(pose.orientation)
    theta = euler_xyz[2]
    return x, y, theta


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


def theta_to_quaternion(theta):
    '''
    Arguments:
        theta {float}: robot orientation in world frame.
    Output:
        q {geometry_msgs.msg.Quaternion}
    '''
    R = theta_to_rotation_matrix(theta)
    q_list = quaternion_from_matrix(R)

    def list_to_ros_quaternion(l):
        q = Quaternion(l[0], l[1], l[2], l[3])
        return q
    q = list_to_ros_quaternion(q_list)
    return q


def calc_dist(x1, y1, x2, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2)**(0.5)


def pi2pi(theta):
    return (theta + math.pi) % (2 * math.pi) - math.pi
