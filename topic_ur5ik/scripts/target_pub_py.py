#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

pi = 3.141592

m_x = float(0.0)
m_y = float(0.6)
m_z = float(0.3)
m_roll = float(pi)
m_pitch = float(0.0)
m_yaw = float(0.0)

class target_pub(Node):

    def __init__(self):
        super().__init__('target_pub')
        self.m_publisher = self.create_publisher(Float64MultiArray, 'topic_ik', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.layout = MultiArrayLayout(
            dim = [MultiArrayDimension(label = 'values', size = 6, stride = 1)],
            data_offset=0
        )
        msg.data = [m_x, m_y, m_z, m_roll, m_pitch, m_yaw] #% self.i
        self.m_publisher.publish(msg)
        self.get_logger().info('Publishing : "%f"' %msg.data[0])
        self.get_logger().info('Publishing : "%f"' %msg.data[1])
        self.get_logger().info('Publishing : "%f"' %msg.data[2])
        self.get_logger().info('Publishing : "%f"' %msg.data[3])
        self.get_logger().info('Publishing : "%f"' %msg.data[4])
        self.get_logger().info('Publishing : "%f"' %msg.data[5])
        self.get_logger().info('-----------------')
        #self.i += 1

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(target_pub())
    rclpy.shutdown()

if __name__ == '__main__':
    main()