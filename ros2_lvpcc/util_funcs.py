#!/usr/bin/env python

import numpy as np
from geometry_msgs.msg import Quaternion
import math

def pcc_kinematics( L, theta, s ):

    return L*np.array([math.sin(s*theta)/theta, (1-math.cos(s*theta))/theta])

def rotation_matrix_y_2d(angle):
    """
    Rotation matrix around the y axis.

    Parameters
    ----------
    angle: float
        Rotation angle in radians

    Returns
    -------
    np.array
        Rotation matrix

    """
    return np.array([[np.cos(angle), np.sin(angle)],
                     [-np.sin(angle), np.cos(angle)]])

def rotation_matrix_x(angle):
    """
    Rotation matrix around the x axis.

    Parameters
    ----------
    angle: float
        Rotation
    Returns
    -------
    np.array
        Rotation matrix

    """
    return np.array([[1, 0, 0],
                     [0, np.cos(angle), -np.sin(angle)],
                     [0, np.sin(angle), np.cos(angle)]])


def rotation_matrix_y(angle):
    """
    Rotation matrix around the y axis.

    Parameters
    ----------
    angle: float
        Rotation angle in radians

    Returns
    -------
    np.array
        Rotation matrix

    """
    return np.array([[np.cos(angle), 0, np.sin(angle)],
                     [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]])


def rotation_matrix_z(angle):
    """
    Rotation matrix around the z axis.

    Parameters
    ----------
    angle: float
        Rotation angle in radians

    Returns
    -------
    np.array
        Rotation matrix

    """
    return np.array([[np.cos(angle), -np.sin(angle), 0],
                     [np.sin(angle), np.cos(angle), 0],
                     [0, 0, 1]])

def quaternion_to_euler(
    quaternion: Quaternion,
    radians: bool = True
) -> np.array:
    """
    Convert quaternion to degrees.

    Parameters
    ----------
    quaternion: Quaternion
        ROS Quaternion Msg to be converted
    radians: bool
        If True, the output will be in radians. If False, the output will be
        in degrees.

    Returns
    -------
    np.array
        Quaternion converted to degrees.

    """
    quaternion = np.array([quaternion.x,
                           quaternion.y,
                           quaternion.z,
                           quaternion.w])

    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    ori_angles = []

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    ori_angles.append(np.arctan2(t0, t1))  # roll_x

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    ori_angles.append(np.arcsin(t2))  # pitch_y

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    ori_angles.append(np.arctan2(t3, t4))  # yaw_z

    ori_angles = np.array(ori_angles)
    if not radians:
        ori_angles = np.degrees(ori_angles)

    return ori_angles

def generate_phi_theta(self):

    self.n_links = 1
    self.n_imus = 1
    rot_mat_world_zyx = [np.eye(3)]*self.n_links
    rot_mat_reconstructed = [np.eye(3)]*self.n_links
    state = np.zeros((self.n_links*2, 1))

    phi_list = [0]*self.n_links

    for imu_index in range(self.n_imus):

        # seg_names = self.imu_seg_names[imu_index+1]
        state_imu = self.euler[:]
        rotation_x = rotation_matrix_x(state_imu[0])
        rotation_y = rotation_matrix_y(state_imu[1])
        rotation_z = rotation_matrix_z(state_imu[2])

        # Rotation matrices global are calculated
        # with respect to the world frame
        # rot_mat_world_zyx = rotation_x @ rotation_y @ rotation_z
        rot_mat_world_zyx = rotation_z @ rotation_y @ rotation_x

        # Calculate the phi angle or the orientation (rotation around Z)
        if imu_index == 0:
            z_proj_prev_frame = rot_mat_world_zyx[:, 2]
            phi = np.arctan2(z_proj_prev_frame[1], z_proj_prev_frame[0])

            # Theta signal is necessary to keep the theta angle between
            # -pi/2 and pi/2, according to the phi angle
            # NOTE: the signal must be after the phi calculation without
            # correction
            theta_signal = -1 if phi > np.pi/2 or phi < -np.pi/2 else 1

            phi = fix_phi_angle(phi)
            phi_list[imu_index] = phi

            # It is necessary to normalize the third column of the
            # rotation matrix to avoid floating point approximation
            # errors
            length = np.linalg.norm(rot_mat_world_zyx[:, 2])
            normalized_third_column = rot_mat_world_zyx[:, 2] / length
            theta = np.arccos(normalized_third_column[2]) * theta_signal

            # We need to reconstruct the homogeneous transformation matrix
            # based on the phi and theta angles because the orientation
            # of the next segment affects the orientation of the previous
            # segment and we loose reference
            rotation_y = rotation_matrix_y(theta)
            rotation_z = rotation_matrix_z(phi)
            rot_mat_reconstructed[imu_index] = rotation_z @ rotation_y
        else:
            # Local rotation matrices are calculated
            # with respect to the previous segment
            rot_mat_local =\
                np.linalg.inv(rot_mat_reconstructed[imu_index-1]) \
                @ rot_mat_world_zyx
            z_proj_prev_frame = rot_mat_local[:, 2]
            phi = np.arctan2(z_proj_prev_frame[1], z_proj_prev_frame[0])

            # Theta signal is necessary to keep the theta angle between
            # -pi/2 and pi/2, according to the phi angle
            # NOTE: the signal must be after the phi calculation without
            # correction
            theta_signal = -1 if phi > np.pi/2 or phi < -np.pi/2 else 1

            # Sum all previous phi angles
            if imu_index < self.n_imus - 1:
                for phi_i in range(0, imu_index):
                    phi += phi_list[phi_i]

            phi = fix_phi_angle(phi)
            phi_list[imu_index] = phi

            # It is necessary to normalize the third column of the
            # rotation matrix to avoid floating point approximation
            # errors
            length = np.linalg.norm(rot_mat_local[:, 2])
            normalized_third_column = rot_mat_local[:, 2] / length
            theta = np.arccos(normalized_third_column[2]) * theta_signal
            rot_mat_reconstructed[imu_index] = rot_mat_world_zyx

        state[imu_index*2:imu_index*2+2] =\
            np.array([phi, theta]).reshape(-1, 1)

    return state

def fix_phi_angle(phi):
    """
    Fix the phi angle.

    The phi angle must be between -pi/2 and pi/2.

    Parameters
    ----------
    phi : float
        Angle in radians.

    Returns
    -------
    float
        Angle in radians.

    """
    if phi > np.pi/2:
        phi = phi - np.pi
    elif phi < -np.pi/2:
        phi = phi + np.pi
    return phi