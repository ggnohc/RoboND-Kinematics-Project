#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
# from sympy import *
import numpy as np
# from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

#GC
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Create Modified DH parameters
        s = {alpha0:     0,  a0:      0,  d1:  0.75, q1: q1,
             alpha1: -pi/2,  a1:   0.35,  d2:     0, q2: q2-pi/2,
             alpha2:     0,  a2:   1.25,  d3:     0, q3: q3,
             alpha3: -pi/2,  a3: -0.054,  d4:   1.5, q4: q4,
             alpha4:  pi/2,  a4:      0,  d5:     0, q5: q5,
             alpha5: -pi/2,  a5:      0,  d6:     0, q6: q6,
             alpha6:     0,  a6:      0,  d7: 0.303, q7: 0}

        #### Homogeneous Transforms
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                   0,                   0,            0,               1]])
        T0_1 = T0_1.subs(s)

        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])
        T1_2 = T1_2.subs(s)

        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [                   0,                   0,            0,               1]])
        T2_3 = T2_3.subs(s)

        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                   0,                   0,            0,               1]])
        T3_4 = T3_4.subs(s)

        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                   0,                   0,            0,               1]])
        T4_5 = T4_5.subs(s)

        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                   0,                   0,            0,               1]])
        T5_6 = T5_6.subs(s)

        T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                   0,                   0,            0,               1]])
        T6_G = T6_G.subs(s)

        # Transform from base link to gripper
        T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)

        # Added correction rotation matrix to compensate for orientation difference
          # of gripper link in URDF vs DH convention
        R_y = Matrix([[ cos(-pi/2),  0, sin(-pi/2), 0],
                      [          0,  1,          0, 0],
                      [ -sin(-pi/2), 0, cos(-pi/2), 0],
                      [       0,     0,          0, 1]])

        R_z = Matrix([[ cos(pi), -sin(pi), 0, 0],
                      [ sin(pi),  cos(pi), 0, 0],
                      [       0,        0, 1, 0],
                      [       0,        0, 0, 1]])

        R_corr = simplify(R_z * R_y)

        T_tot = simplify(T0_G * R_corr)

        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            r, p, y = symbols('r p y')

            ROT_x = Matrix([[1,              0,          0],
                            [0,      cos(r), -sin(r)],
                            [0,      sin(r), cos(r)]])

            ROT_y = Matrix([[cos(p),     0, sin(p)],
                            [         0,     1,          0],
                            [-sin(p),    0, cos(p)]])

            ROT_z = Matrix([[cos(y), -sin(y),        0],
                            [sin(y),  cos(y),        0],
                            [       0,         0,        1]])
            Rrpy_ee = ROT_z * ROT_y * ROT_x

	    # Compensate for rotation discrepancy between DH parameters and Gazebo
    	    R_corr = ROT_z.subs(y, pi)*ROT_y.subs(p, -pi/2)  #TODO: Check why exercise use Homogeneous matrix instead of rotation matrix
            Rrpy_ee = ROT_z * ROT_y * ROT_x * R_corr
            Rrpy_ee = Rrpy_ee.subs({'r': roll, 'p': pitch, 'y': yaw})

	    # Calculate joint angles using Geometric IK method
            # EE position w.r.t base_link
            EE = Matrix([px, py, pz])

            # Wrist center  w.r.t base_link
            d7 = 0.33 # from URDF
            WC = EE - d7*Rrpy_ee*Matrix([0,0,1])
            print ("WC: {}".format(WC))

            # Find theta1
            theta1 = atan2(WC[1], WC[0])
            print("theta1: {}".format(theta1))

            # Find q2
            a1 = 0.35
            #J2_X = a1*cos(theta1)
            J2_X = 0.35*cos(theta1)
            print("J2_X: {}".format(J2_X))
            #J2_Y = a1*sin(theta1)
            J2_Y = 0.35*sin(theta1)
            print("J2_Y: {}".format(J2_Y))
            d1 = 0.75
            J2_Z = d1
            print("J2_Z: {}".format(J2_Z))
            J5_X = WC[0]
            print("J5_X: {}".format(J5_X))
            J5_Y = WC[1]
            print("J5_Y: {}".format(J5_Y))
            J5_Z = WC[2]
            print("J5_Z: {}".format(J5_Z))

            side_A = 1.4995 #from URDF and cosine rule
            side_B = sqrt((J5_X-J2_X)**2 + (J5_Y-J2_Y)**2 + (J5_Z-J2_Z)**2)
            side_C = 1.25 #from URDF

            #Now that we have all the sides, we can find angle_a with simple trigonometry
            angle_a = acos((side_B**2 + side_C**2 - side_A**2)/(2*side_B*side_C))
            angle_b = acos((side_A**2 + side_C**2 - side_B**2)/(2*side_A*side_C))
            angle_c = acos((side_A**2 + side_B**2 - side_C**2)/(2*side_A*side_B))

            # theta2 = pi/2 - angle_a - atan2(J5_Z-J2_Z, sqrt(J5_X**2+J5_Y**2)-a1)
            theta2 = pi/2 - angle_a - atan2(J5_Z-J2_Z, sqrt((J5_X-J2_X)**2 + (J5_Y-J2_Y)**2 ))
            print("theta2: {}".format(theta2))

            theta3 = pi/2 - (angle_b + 0.036) #TODO: check the value of 0.036 being sag in L4 of -0.054
            print("theta3: {}".format(theta3))

            # Use Inverse Position to calculate Theta4, 5 and 6.
            R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3:theta3})

            R3_6 = R0_3.inv("LU") * Rrpy_ee

            # Euler angles from a rotation matrix lesson TODO: Check the formula against lecture
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
