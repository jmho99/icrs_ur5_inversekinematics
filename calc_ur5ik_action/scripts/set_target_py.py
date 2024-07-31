#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from calc_interfaces.action import CalcTheta

class set_target(Node):
    def __init__(self):
        super().__init__('set_target')
        self.m_action_client = ActionClient(self, CalcTheta, 'action_ik')

    def send_goal(self):
        goal_msg = CalcTheta.Goal()
        goal_msg.act_target[0] = 0.0
        goal_msg.act_target[1] = 0.6
        goal_msg.act_target[2] = 0.3
        goal_msg.act_target[3] = 3.1415
        goal_msg.act_target[4] = 0.0
        goal_msg.act_target[5] = 0.0

        self.m_action_client.wait_for_server()

        self.send_goal_future = self.m_action_client.send_goal_async(goal_msg)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Action rejected :(')
            return
        self.get_logger().info('Action acecpted')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Theta: {0}'.format(result.act_theta))
        rclpy.shutdown()

def main():
    rclpy.init()
    action_client = set_target()
    action_client.send_goal()
    rclpy.spin(action_client)
    # rclpy.shutdown()

if __name__ == '__main__':
    main()