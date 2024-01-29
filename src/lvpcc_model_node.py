#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from labjack import ljm
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import numpy as np
from ros2_lvpcc.util_funcs import *

class LVPCC( Node ):

    def __init__(self):

        super().__init__('lvpcc')

        self.L_t = 1 # length of backbone
        # lengths of sensors
        self.L_init = np.array([0.5, 0.5]) # mm
        self.L = self.L_init
        # normalised point of sensor attachment, (0,1)
        self.P1 = 0.3
        self.P2 = self.P1


        self.length_sub = self.create_subscription(Float32MultiArray, '/labjack_encoder', self.update_length, 10)
        self.imu_sub = self.create_subscription(Pose, '/position_from_imu_seg1', self.update_imu, 10)

        # init timer to publish encoder position
        timer_period = 0.05 # seconds
        self.model_timer = self.create_timer(timer_period, self.lvpcc_model)

    def lvpcc_model(self):

        M1 = self.L[0]
        M2 = self.L[1]
        P1 = self.P1
        P2 = self.P2
        L = self.L_t
        q = 0.1
        # offset of sensors from centre line
        r = 0

        # calculation of model kinematics
        A1 = P2*(M1-P1*L+P1*q*r)
        B1 = L-M2-P1*L-P2*L-q*r+M1*P2+M2*P1+P1*q*r+P2*q*r
        Pt = A1/B1

        A2 = P1*L**2 - M1*L + M1*M2 + P1*q**2*r**2 + M1*P2*L - M2*P1*L + M1*q*r
        - P1*P2*L**2 + M2*P1*q*r - 2*P1*q*L*r + P1*P2*q*L*r
        B2 = L*r - M2*r - q*r**2 + M1*P2*r + M2*P1*r - P1*L*r - P2*L*r + P1*q*r**2 + P2*q*r**2
        q1 = A2/B2

        A3 = -(P1*L**2 - M1*L + q**2*r**2 + M1*M2 - P2*q**2*r**2 + M1*P2*L - M2*P1*L
                + M1*q*r + M2*q*r - q*L*r - P1*P2*L**2 - M1*P2*r*q - P1*q*L*r + P2*q*L*r + P1*P2*q*L*r)
        B3 = L*r - M2*r - q*r**2 + M1*P2*r + M2*P1*r - P1*L*r - P2*L*r + P1*q*r**2 + P2*q*r**2
        q2 = A3/B3

        # and then calculate all forward kinematics for the two curves

    # update imu reading
    def update_imu(self, data):

        self.euler = quaternion_to_euler(data.orientation)
        state = generate_phi_theta()
        self.phi = np.rad2deg(state[0].item())
        self.theta = np.rad2deg(state[1].item())

    # update length readings
    def update_length(self, data):

        enc_state = list(data.data)
        self.L = [(x/1.98)/4 + self.L_init for x in enc_state]

        self.get_logger().info("Length: %s" % self.L)

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