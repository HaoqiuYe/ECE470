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
		temp = expm(VecTose3(S[:, 5-i]) *  np.radians(t[5-i])) @ temp 

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
	# =================== Your code starts here ====================#
	
	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0
	
	# ==============================================================#
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
