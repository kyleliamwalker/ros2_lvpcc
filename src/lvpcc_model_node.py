#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import numpy as np
from ros2_lvpcc.util_funcs import *
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class LVPCC( Node ):

    def __init__(self):

        super().__init__('lvpcc')

        self.L_t = 0.3 # length of backbone, m
        # lengths of sensors
        self.L_init = np.array([0.185, 0.185]) # m
        self.L = self.L_init
        # normalised point of sensor attachment, (0,1)
        self.P1 = self.L_init[0]/self.L_t
        self.P2 = self.L_init[1]/self.L_t

        self.length_sub = self.create_subscription(Float32MultiArray, '/labjack_encoder', self.update_length, 10)
        self.imu_sub = self.create_subscription(Pose, '/position_from_imu_seg1', self.update_imu, 10)
        
        # self.fig, self.ax = plt.subplots()
        # self.ax.set_xlim(-2, 2)
        # self.ax.set_xlim(-2, 2)
        # self.scatter1, = self.ax.plot([], [], 'bo', label='seg1')
        # self.scatter2, = self.ax.plot([], [], 'ro', label='seg2')
        # self.ax.legend()

        self.s = np.linspace(0.01, 1, 100)
        self.pos1 = np.empty((2, len(self.s)))
        self.pos2 = np.empty((2, len(self.s)))
        
        # self.animation = FuncAnimation(self.fig, self.plotting, interval=100)

        # init timer to publish encoder position
        timer_period = 0.05 # seconds
        self.model_timer = self.create_timer(timer_period, self.lvpcc_model)

    def plotting(self):

        self.scatter1.set_data(self.pos1[0,:], self.pos1[1,:])
        self.scatter2.set_data(self.pos2[0,:], self.pos2[1,:])

    def lvpcc_model(self):

        M1 = self.L[0]
        M2 = self.L[1]
        P1 = self.P1
        P2 = self.P2
        L = self.L_t
        
        # self.get_logger().info("q: %s, M1: %s, M2: %s, P1: %s, P2: %s, L: %s" % (self.theta, M1, M2, P1, P2, L))
        
        q = np.deg2rad(self.theta)
        # offset of sensors from centre line
        r = 0.1

        # self.get_logger().info("q: %s" % q)

        # calculation of model kinematics
        A1 = P2*(M1-P1*L+P1*q*r)
        B1 = L-M2-P1*L-P2*L-q*r+M1*P2+M2*P1+P1*q*r+P2*q*r
        Pt = A1/B1

        # self.get_logger().info("A1: %s, B1: %s, Pt: %s" % (A1, B1, Pt))

        A2 = P1*L**2 - M1*L + M1*M2 + P1*q**2*r**2 + M1*P2*L - M2*P1*L + M1*q*r - P1*P2*L**2 + M2*P1*q*r - 2*P1*q*L*r + P1*P2*q*L*r
        B2 = L*r - M2*r - q*r**2 + M1*P2*r + M2*P1*r - P1*L*r - P2*L*r + P1*q*r**2 + P2*q*r**2
        q1 = A2/B2

        self.get_logger().info("A2: %s, B2: %s, q1: %s" % (A2, B2, np.rad2deg(q1)))

        A3 = -(P1*L**2 - M1*L + q**2*r**2 + M1*M2 - P2*q**2*r**2 + M1*P2*L - M2*P1*L + M1*q*r + M2*q*r - q*L*r - P1*P2*L**2 - M1*P2*r*q - P1*q*L*r + P2*q*L*r + P1*P2*q*L*r)
        B3 = L*r - M2*r - q*r**2 + M1*P2*r + M2*P1*r - P1*L*r - P2*L*r + P1*q*r**2 + P2*q*r**2
        q2 = A3/B3

        self.get_logger().info("A3: %s, B3: %s, q2: %s" % (A3, B3, np.rad2deg(q2)))

        i = 0
        for x in self.s:
            self.pos1[:,i] = pcc_kinematics( M1, q1, x )
            self.pos2[:,i]= self.pos1[:,i] + rotation_matrix_y_2d(q1) @ pcc_kinematics( M2, q2, x )
            i += 1

        self.get_logger().info("pos2 = %s, %s" % (self.pos2[0,-1], self.pos2[1,-1]))

    # update imu reading
    def update_imu(self, data):        

        self.euler = quaternion_to_euler(data.orientation)
        state = generate_phi_theta(self)
        self.phi = np.rad2deg(state[0].item())
        self.theta = np.rad2deg(state[1].item())
        # self.get_logger().info("Theta: %s, Phi: %s" % (self.theta, self.phi))

    # update length readings
    def update_length(self, data):

        enc_state = list(data.data)
        self.L = [(x/1.98)/4/1000 + self.L_init[0] for x in enc_state]

        # self.get_logger().info("Length: %s" % enc_state)

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