#!/usr/bin/env python3
import roboticstoolbox as rtb
from spatialmath import *

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from calc_interfaces.action import CalcTheta

class ur5ik(Node):
    def __init__(self):
        super().__init__('ur5ik')
        self.m_action_server = ActionServer(
            self,
            CalcTheta,
            'action_ik',
            self.execute_callback )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Execute goal')

        request = CalcTheta.Goal()

        #calc inverse kinematics
        x = request.act_target[0]
        y = request.act_target[1]
        z = request.act_target[2]
        roll = request.act_target[3]
        pitch = request.act_target[4]
        yaw = request.act_target[5]

        robot = rtb.models.UR5()
        Tep = SE3.Trans(x, y, z) * SE3.Rz(yaw, 'rad') * SE3.Ry(pitch, 'rad') * SE3.Rx(roll, 'rad')
        sol = robot.ik_LM(Tep)         # solve IK

        theta1 = sol[0][0]
        theta2 = sol[0][1]
        theta3 = sol[0][2]
        theta4 = sol[0][3]
        theta5 = sol[0][4]
        theta6 = sol[0][5]

        goal_handle.succeed()
        result = CalcTheta.Result()

        result.act_theta[0] = theta1
        result.act_theta[1] = theta2
        result.act_theta[2] = theta3
        result.act_theta[3] = theta4
        result.act_theta[4] = theta5
        result.act_theta[5] = theta6
        self.get_logger().info('theta: {0}'.format(result.act_theta))
        return result

def main():
    rclpy.init()
    rclpy.spin(ur5ik())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
