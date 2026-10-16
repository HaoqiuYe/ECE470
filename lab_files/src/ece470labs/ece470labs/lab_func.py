#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from math import pi
import math
from numpy import linalg as la

l1 =  152
l2 = 120
l3 = 244
l4 = 93
l5 = 213
l6 = 104
l7 = 85
l8 = 92
"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def VecToso3(omg):
    """Converts a 3-vector to an so(3) representation
    :param omg: A 3-vector
    :return: The skew symmetric representation of omg
    Example Input:
        omg = np.array([1, 2, 3])
    Output:
        np.array([[ 0, -3,  2],
                  [ 3,  0, -1],
                  [-2,  1,  0]])
    """
    return np.array([[0,      -omg[2],  omg[1]],
                     [omg[2],       0, -omg[0]],
                     [-omg[1], omg[0],       0]])
def VecTose3(V):
    """Converts a spatial velocity vector into a 4x4 matrix in se3
    :param V: A 6-vector representing a spatial velocity
    :return: The 4x4 se3 representation of V
    Example Input:
        V = np.array([1, 2, 3, 4, 5, 6])
    Output:
        np.array([[ 0, -3,  2, 4],
                  [ 3,  0, -1, 5],
                  [-2,  1,  0, 6],
                  [ 0,  0,  0, 0]])
    """
    return np.r_[np.c_[VecToso3([V[0], V[1], V[2]]), [V[3], V[4], V[5]]],
                 np.zeros((1, 4))]

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	M[:3,3] = np.array([392, 432, 215.5])
	M[3,3] = 1

	M[:3,0] = np.array([0,0,1])
	M[:3,1] = np.array([-1,0,0])
	M[:3,2] = np.array([0,-1,0])
	
	S[:3,0] = np.array([0,0,1])
	S[3:6, 0] = np.cross(-S[:3,0], np.array([-150,150,10]))

	S[:3,1] = np.array([0,1,0])
	S[3:6, 1] = np.cross(-S[:3,1], np.array([-150,150+l2,l1+10]))

	S[:3,2] = np.array([0,1,0])
	S[3:6, 2] = np.cross(-S[:3,2], np.array([-150+l3,150+l2,l1+10]))
 
	S[:3,3] = np.array([0,1,0])
	S[3:6, 3] = np.cross(-S[:3,3], np.array([-150+l3+l5,150+l2-l4,l1+10]))

	S[:3,4] = np.array([1,0, 0])
	S[3:6, 4] = np.cross(-S[:3,4], np.array([-150+l3+l5,150+l2-l4+l6,l1+10]))

	S[:3,5] = np.array([0,1, 0])
	S[3:6, 5] = np.cross(-S[:3,5], np.array([-150+l3+l5+l7,150+l2-l4+l6,l1+10]))

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]
	t =[theta1, theta2, theta3, theta4, theta5, theta6]
	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	T = [ [1.0, 0.0, 0.0], \
      [0.0, 1.0, 0.0], \
      [0.0, 0.0, 1.0] ]
	# ==============================================================#

	M, S = Get_MS()
	temp = M.copy()
	for i in range(6):
		temp = expm(VecTose3(S[:, 5-i]) *  (t[5-i])) @ temp 

	print()
	print(str(temp) + "\n")

	return_value[0] = theta1 + pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*pi)
	return_value[4] = theta5
	return_value[5] = theta6




	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    # Position inputs are in meters; yaw is in degrees.
    # Internal distances are in mm to match link lengths.

    # 1. Convert world coordinates to robot base coordinates.
    x0grip = 1000 * xWgrip + 150
    y0grip = 1000 * yWgrip - 150
    z0grip = 1000 * zWgrip - 10

    yaw = np.deg2rad(yaw_WgripDegree)

    # 2. Find the virtual wrist center.
    alu_plate_length = 53.5

    xcen = x0grip - alu_plate_length * np.cos(yaw)
    ycen = y0grip - alu_plate_length * np.sin(yaw)
    zcen = z0grip

    # 3. Solve for theta1 using the top view.
    d = l2 - l4 + l6       # 131 mm sideways offset
    f = np.sqrt(xcen**2 + ycen**2)

    theta1 = np.arctan2(ycen, xcen) - np.arcsin(d / f)

    # 4. Solve for theta6 using the desired yaw.
    theta6 = pi / 2 + theta1 - yaw

    # 5. Find the projected endpoint of the two-link arm.
    x3end = xcen + d * np.sin(theta1) - l7 * np.cos(theta1)
    y3end = ycen - d * np.cos(theta1) - l7 * np.sin(theta1)
    z3end = zcen + l8 + 59

    # 6. Solve the side-view triangle for the elbow-up configuration.
    r = x3end * np.cos(theta1) + y3end * np.sin(theta1)
    s = z3end - l1
    c = np.sqrt(r**2 + s**2)

    theta3 = pi - np.arccos(
        (l3**2 + l5**2 - c**2) / (2 * l3 * l5)
    )

    beta = np.arccos(
        (l3**2 + c**2 - l5**2) / (2 * l3 * c)
    )

    theta2 = -np.arctan2(s, r) - beta
    theta4 = -theta2 - theta3
    theta5 = -pi / 2

    # 7. Pass the angles to lab_fk, which applies controller offsets.
    return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)

def main(args=None):
	lab_fk(-46.73, -75.65, 98.23, -59.53, -112.40,89.88)
	M,S = Get_MS()

	print("M: ")
	print(M)
	print("S: ")
	print(S)

if __name__ == '__main__':
    main()
