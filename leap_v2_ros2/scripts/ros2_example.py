#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from leap_hand.srv import LeapPosition, LeapVelocity, LeapEffort, LeapPosVelEff
import time
import numpy as np

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.pos = self.create_client(LeapPosition, '/leap_position')
        self.vel = self.create_client(LeapVelocity, '/leap_velocity')
        self.eff = self.create_client(LeapEffort, '/leap_effort')
        self.pos_vel = self.create_client(LeapPosVelEff, '/leap_pos_vel')
        ##Note if you need to read multiple values this is faster than calling each service individually for the motors
        while not self.pos_vel.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LeapPosVelEff.Request()
        self.pub_hand = self.create_publisher(JointState, '/cmd_leap', 10) 

    def send_request(self):
        self.future = self.pos_vel.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    x = list(np.zeros(8))
    y = 0.025
    while True:
        response = minimal_client.send_request()
        print(response)  ##Receive 
        time.sleep(0.05)
        stater = JointState()
        x[1] = x[1] + y
        if x[1] > 0.8:
            y = - 0.025
        if x[1] < 0:
            y = 0.025
        print(x)
        stater.position = x  ##You can set the position this way
        minimal_client.pub_hand.publish(stater)  # Choose the right embodiment here
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()