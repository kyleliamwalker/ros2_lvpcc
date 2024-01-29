#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from labjack import ljm
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Float32

# have to call this locally to avoid trying to open multiple LJ instances
class LabJack( Node ):

    def __init__(self):

        super().__init__('labjack')
        # Open first found LabJack
        self.lj_handle = ljm.openS("T7", "ANY", "ANY")
        self.info = ljm.getHandleInfo(self.lj_handle)

        self.get_logger().info("Opened a LabJack with Device type: %i, Connection type: %i,\n"
              "Serial number: %i, IP address: %s, Port: %i,\nMax bytes per MB: %i" %
              (self.info[0], self.info[1], self.info[2], ljm.numberToIP(self.info[3]), self.info[4], self.info[5]))

        ljm.eWriteNames(self.lj_handle, 6, ["DIO0_EF_ENABLE", "DIO1_EF_ENABLE", "DIO0_EF_INDEX", 
                                            "DIO1_EF_INDEX", "DIO0_EF_ENABLE", "DIO1_EF_ENABLE"], 
                                            [0, 0, 10, 10, 1, 1])
        ljm.eWriteNames(self.lj_handle, 6, ["DIO2_EF_ENABLE", "DIO3_EF_ENABLE", "DIO2_EF_INDEX", 
                                            "DIO3_EF_INDEX", "DIO2_EF_ENABLE", "DIO3_EF_ENABLE"], 
                                            [0, 0, 10, 10, 1, 1])

        # init timer to publish encoder position
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.labjack_callback)
        self.encoder_pub = self.create_publisher(Float32MultiArray, '/labjack_encoder', 10)

    def labjack_callback(self):

        # DIO0_EF_READ_A_F displays counts both positive and negative
        # DIO0_EF_READ_A displays counts positive and negative as two's complement
        data = ljm.eReadNames(self.lj_handle, 2, ["DIO0_EF_READ_A_F", "DIO2_EF_READ_A_F"])
        self.get_logger().info("Output : %f, %f" % (data[0], data[1]))

        # pub msg
        encoder_msg = Float32MultiArray()
        encoder_msg.data = data
        # encoder_msg.layout.data_offset = 0 

        # create two dimensions in the dim array
        encoder_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]

        # dim[0] is AIN0
        encoder_msg.layout.dim[0].label = "DIO0_A"
        encoder_msg.layout.dim[0].size = 1
        encoder_msg.layout.dim[0].stride = 1
        # dim[1] is AIN1
        encoder_msg.layout.dim[1].label = "DIO2_A"
        encoder_msg.layout.dim[1].size = 1
        encoder_msg.layout.dim[1].stride = 1

        self.encoder_pub.publish(encoder_msg)


def main():
    rclpy.init()
    labjack = LabJack()
    try:
        rclpy.spin(labjack)
    except KeyboardInterrupt:
        print("Shutting down labjack node...")
    finally:
        ljm.close(labjack.lj_handle)
        labjack.destroy_node()

if __name__ == '__main__':
    main()