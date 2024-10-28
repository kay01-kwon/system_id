#! /usr/bin/env python3.8

import os
import sys

'''
Append nmpc_pkg directory using sys module
'''
dir_path = os.path.dirname(os.path.realpath(__file__))
# print(dir_path)

pkg_dir = dir_path + '/nmpc_attitude_pkg'
# print(pkg_dir)
sys.path.append(dir_path + '/nmpc_attitude_pkg')


import numpy as np
import nmpc_attitude_pkg.tools
from nmpc_attitude_pkg import ocp_solver
import rospy
from sensor_msgs.msg import Imu
from ros_libcanard.msg import cmd_raw

class nmpc_quad_node:
    def __init__(self):
        '''
        Initialize the ocp solver and store data for state, reference and so on.
        '''
        rospy.init_node('nmpc_quad', anonymous=True)
        # Create ocp solver object
        self.ocp_solver_obj = ocp_solver.OcpSolver()

        self.state = np.zeros((7,))
        self.state[0] = 1.0

        self.ref = np.zeros((7,))
        self.ref[0] = 1.0

        self.u = np.zeros((4,))
        self.rpm_des = np.zeros((4,))
        self.u_msg = cmd_raw()

        self.C_T = 1.481e-07

        self.MaxBit = 8191
        self.MaxRPM = 9800
        self.RPM2Bit = self.MaxBit/self.MaxRPM

        self.ros_setup()

    def ros_setup(self):
        '''
        Publisher and subscriber setup
        :return: None
        '''

        # Construct message filter to subscribe
        # imu (quaternion and angular velocity), and reference

        self.imu_sub = rospy.Subscriber('/imu',
                                        Imu,
                                        self.Imu_callback,
                                        queue_size=1)

        self.ref_sub = rospy.Subscriber('/ref',
                                        Imu,
                                        self.Ref_callback,
                                        queue_size=1)


        # Input publisher to esc
        self.input_pub = rospy.Publisher('/cmd_raw',
                                         cmd_raw,
                                         queue_size=1)
        self.ros_rate = rospy.Rate(100)

    def Imu_callback(self, msg):

        self.state[0] = msg.orientation.w
        self.state[1] = msg.orientation.x
        self.state[2] = msg.orientation.y
        self.state[3] = msg.orientation.z

        self.state[4] = msg.angular_velocity.x
        self.state[5] = msg.angular_velocity.y
        self.state[6] = msg.angular_velocity.z

    def Ref_callback(self, msg):
        self.ref[0] = msg.orientation.w
        self.ref[1] = msg.orientation.x
        self.ref[2] = msg.orientation.y
        self.ref[3] = msg.orientation.z

    def publish_control_input(self):
        status, self.u = self.ocp_solver_obj.ocp_solve(self.state, self.ref)

        # u[i] = C_lift * rpm[i]^2
        # rpm[i] = sqrt(u[i]/C_lift)

        self.u_msg.stamp = rospy.Time.now()

        for i in range(4):
            self.rpm_des[i] = np.sqrt(self.u[i]/self.C_T)
            self.u_msg.raw[i] = int(self.rpm_des[i] * self.RPM2Bit)

        self.input_pub.publish(self.u_msg)

    def run(self):
        # rospy.spin()
        while not rospy.is_shutdown():
            self.publish_control_input()
            self.ros_rate.sleep()

if __name__ == '__main__':
    # main()
    nmpc_quad_node = nmpc_quad_node()
    nmpc_quad_node.run()
