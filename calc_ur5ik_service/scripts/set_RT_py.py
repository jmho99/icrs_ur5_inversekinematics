#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

from calc_interfaces.srv import SixTheta

pi = 3.141592

m_x = float(0.0)
m_y = float(0.6)
m_z = float(0.3)
m_roll = float(pi)
m_pitch = float(0.0)
m_yaw = float(0.0)


class set_RT(Node):

    def __init__(self):
        super().__init__('set_RT')
        self.cli = self.create_client(SixTheta, 'calc_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SixTheta.Request()

    def send_request(self):
        self.req.srv_target[0] = m_x
        self.req.srv_target[1] = m_y
        self.req.srv_target[2] = m_z
        self.req.srv_target[3] = m_roll
        self.req.srv_target[4] = m_pitch
        self.req.srv_target[5] = m_yaw
        return self.cli.call_async(self.req)

def main():
    rclpy.init()
    send_target = set_RT()
    get_theta = send_target.send_request()
    rclpy.spin_until_future_complete(send_target, get_theta)
    response = get_theta.result()
    send_target.get_logger().info(
        'theta : %f, %f, %f, %f, %f, %f' %
        (response.srv_theta[0], response.srv_theta[1], response.srv_theta[2],
         response.srv_theta[3], response.srv_theta[4], response.srv_theta[5])
    )
    send_target.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
