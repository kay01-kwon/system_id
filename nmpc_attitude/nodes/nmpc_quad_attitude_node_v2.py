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
import message_filters
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ros_libcanard.msg import cmd_raw
from std_msgs.msg import Float64

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

        self.psi_prev = 0

        self.ros_setup()

    def ros_setup(self):
        '''
        Publisher and subscriber setup
        :return: None
        '''

        self.odom_sub = rospy.Subscriber('/vins_estimator/odometry',
                                        Odometry,
                                        self.odom_callback,
                                        queue_size=1)

        self.ref_sub = rospy.Subscriber('/ref',
                                        Float64,
                                        self.Ref_callback,
                                        queue_size=1)


        # self.synchronizer.registerCallback(self.odom_imu_callback)


        # Input publisher to esc
        self.input_pub = rospy.Publisher('/cmd_raw',
                                         cmd_raw,
                                         queue_size=1)
        self.ros_rate = rospy.Rate(100)
    def odom_callback(self, msg):
        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z

        psi = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

        self.state[0] = np.cos(psi / 2)
        self.state[1] = 0
        self.state[2] = 0
        self.state[3] = np.sin(psi / 2)

        self.state[4] = 0
        self.state[5] = 0
        self.state[6] = 0

        # print('Callback')
        # print(self.state[6])

    def Ref_callback(self, msg):
        yaw = msg.data*np.pi/180
        self.ref[0] = np.cos(yaw/2)
        self.ref[1] = 0
        self.ref[2] = 0
        self.ref[3] = np.sin(yaw/2)

        self.ref[4] = 0
        self.ref[5] = 0
        self.ref[6] = 0

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
