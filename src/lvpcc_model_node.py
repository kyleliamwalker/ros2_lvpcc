#!/usr/bin/env python

from imu_ros2_driver.utils import quaternion_to_euler, fix_phi_angle
from imu_ros2_driver.kinematics import rotation_matrix_x, rotation_matrix_y, rotation_matrix_z
import rclpy
from rclpy.node import Node
from labjack import ljm
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import numpy as np

class LVPCC( Node ):

    def __init__(self):

        super().__init__('lvpcc')

        self.enc_ticks = 0
        self.L_init = 0 # mm
        # need to set up a service here to read initial encoder state at launch
        self.prev_enc_state = [0, 0]

        self.length_sub = self.create_subscription(Float32MultiArray, '/labjack_encoder', self.update_length, 10)
        self.imu_sub = self.create_subscription(Pose, '/position_from_imu_seg1', self.update_imu, 10)

        # init timer to publish encoder position
        timer_period = 0.05 # seconds
        self.model_timer = self.create_timer(timer_period, self.lvpcc_model)

    def lvpcc_model(self):

        # insert kinematics
        print("")

    def update_imu(self, data):

        self.euler = quaternion_to_euler(data.orientation)
        state = self.generate_phi_theta()
        self.phi = np.rad2deg(state[0].item())
        self.theta = np.rad2deg(state[1].item())

    def update_length(self, data):

        enc_state = data.data
        self.L = (enc_state[0]/1.98)/4 + self.L_init

        self.get_logger().info("Length: %s" % self.L)


        # if enc_state[0] !=  self.prev_enc_state[0] or enc_state[1] !=  self.prev_enc_state[1]:

        #     if enc_state[0] == 1 and enc_state[1] == 0:
        #         self.enc_ticks += 1
        #     elif enc_state[0] == 0 and enc_state[1] == 1:
        #         self.enc_ticks -= 1

        # self.prev_enc_state = enc_state

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

        
        

def main():
    rclpy.init()
    lvpcc = LVPCC()
    try:
        rclpy.spin(lvpcc)
    except KeyboardInterrupt:
        print("Shutting down lvpcc node...")
    finally:
        lvpcc.destroy_node()

if __name__ == '__main__':
    main()