#!/usr/bin/env python3
import roboticstoolbox as rtb
from spatialmath import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

class ur5ik(Node):

    def __init__(self):
        super().__init__('ur5ik')
        self.m_publisher = self.create_subscription(
            Float64MultiArray,
            'target_pos',
            self.listener_callback,
            10)
        self.subscriptions

    def listener_callback(self, msg):
        x = msg.data[0]
        y = msg.data[1]
        z = msg.data[2]
        roll = msg.data[3]
        pitch = msg.data[4]
        yaw = msg.data[5]

        robot = rtb.models.UR5()
        Tep = SE3.Trans(x, y, z) * SE3.Rz(yaw, 'rad') * SE3.Ry(pitch, 'rad') * SE3.Rx(roll, 'rad')
        sol = robot.ik_LM(Tep)         # solve IK

        theta1 = sol[0][0]
        theta2 = sol[0][1]
        theta3 = sol[0][2]
        theta4 = sol[0][3]
        theta5 = sol[0][4]
        theta6 = sol[0][5]

        self.get_logger().info('theta1 : "%f"' % theta1)
        self.get_logger().info('theta2 : "%f"' % theta2)
        self.get_logger().info('theta3 : "%f"' % theta3)
        self.get_logger().info('theta4 : "%f"' % theta4)
        self.get_logger().info('theta5 : "%f"' % theta5)
        self.get_logger().info('theta6 : "%f"' % theta6)
        self.get_logger().info('-----------------')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ur5ik())
    rclpy.shutdown()

if __name__ == '__main__':
    main()