import roboticstoolbox as rtb
import swift
import numpy as np
import spatialmath as sm
import spatialgeometry as sg
import math
import matplotlib
from matplotlib import pyplot as plt


# The rotation axis, to modify as requested
u = np.array([0, 0, 1])

# Normalisation of the rotation axis (do not modify)
u = u / np.linalg.norm(u)

# Arrays for storing data
theta_real = np.array([]) #pour stocker les vrais theta entre -pi et pi
theta_acos = np.array([]) # le theta obtenu via arcos(tr(R)-1 / 2)
theta_atan2 = np.array([]) #theta obtenu


# Computing theta using using acos and atan2
for theta in np.arange(-np.pi,np.pi,0.001):
	
	# The rotation matrix
	R = np.array([[u[0]**2*(1-np.cos(theta))+np.cos(theta), u[0]*u[1]*(1-np.cos(theta))-u[2]*np.sin(theta), u[0]*u[2]*(1-np.cos(theta))+u[1]*np.sin(theta)] ,\
			  [u[0]*u[1]*(1-np.cos(theta))+u[2]*np.sin(theta), u[1]**2*(1-np.cos(theta))+np.cos(theta), u[1]*u[2]*(1-np.cos(theta))-u[0]*np.sin(theta)] ,\
			  [u[0]*u[2]*(1-np.cos(theta))-u[1]*np.sin(theta), u[1]*u[2]*(1-np.cos(theta))+u[0]*np.sin(theta), u[2]**2*(1-np.cos(theta))+np.cos(theta)  ]])

	# Extracting and normalising the rotation axis
	l = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
	ln = np.linalg.norm(l)

	# Computing the cos of the rotation angle
	arg = (np.trace(R)-1)/2

	arg = arg + 1e-4
	if (arg < -1):
		print('arg = ', arg)
	if (arg > 1):
		print('arg = ', arg)


	# Storing the ground truth theta
	theta_real = np.r_[theta_real, theta]
	# ainsi theta_real = [ theta_real, theta]

	# Storing theta computed using acos
	theta_acos = np.r_[theta_acos, np.arccos(arg)]

	# Storing theta computed using atan2 (to complete)
	theta_atan2 = np.r_[theta_atan2, np.arctan2(ln/2,arg)]

fig, ax = plt.subplots()

plt.plot(theta_real, theta_acos,'r.',markersize=2, label='acos')
plt.plot(theta_real, theta_atan2,'b.',markersize=2, label='atan2')


plt.xlabel('rad')
plt.ylabel('rad')
plt.legend(facecolor='white', framealpha=0)

plt.show()



