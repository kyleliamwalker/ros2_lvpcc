#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from labjack import ljm
from std_msgs.msg import Float32MultiArray

class LVPCC( Node ):

    def __init__(self):

        super().__init__('lvpcc')

        self.enc_ticks = 0
        self.L_init = 0 # mm
        # need to set up a service here to read initial encoder state at launch
        self.prev_enc_state = [0, 0]

        self.length_sub = self.create_subscription(Float32MultiArray, '/labjack_encoder', self.update_length, 10)


    def lvpcc_model(self, measurements):

        print("")

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