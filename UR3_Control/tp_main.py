import roboticstoolbox as rtb
import swift
import numpy as np
import spatialmath as sm
import spatialgeometry as sg
from commande import *
from matplotlib import pyplot as plt
#Create swift instance
env = swift.Swift()
env.launch(realtime=True)


#initialise the model
robot = rtb.models.UR3()

#initialise the robot joints at the intial configuration
robot.q_init = np.array([0, -np.pi/3, np.pi/3, np.pi/3, np.pi/2, 0])
robot.q = robot.q_init

# Creation of the robot end-effector frame in the initial configuration : Tee
Tee = robot.fkine(robot.q)
robot_ee_frame = sg.Axes(length=0.1, pose=Tee)


# Creation of the robot end-effector frame in the target configuration : Tet 


#translation


#Tet = robot.fkine(robot.q) *sm.SE3.Rz(0.0)
Tet = robot.fkine(robot.q) *sm.SE3.Rz(np.pi/4)*sm.SE3.Tx(0.05) * sm.SE3.Ty(0.02) * sm.SE3.Tz(0.08)

#Tet = robot.fkine(robot.q) * sm.SE3.Tx(0.05) * sm.SE3.Ty(0.02) * sm.SE3.Tz(0.01) *sm.SE3.RPY([0.1, 0.2, 0.1], order='xyz')
#Tet = robot.fkine(robot.q) * sm.SE3.Tx(0.05) * sm.SE3.Ty(0.02) * sm.SE3.Tz(0.01) *sm.SE3.RPY([-1.57, 0.0, 0.0], order='xyz')





robot_et_frame = sg.Axes(length=0.1, pose=Tet)


# Adding objetcs to the environnement 
env.add(robot)
env.add(robot_ee_frame)
env.add(robot_et_frame)


# Time step
dt = 0.02

# proportional Controller
K = 0.5

arrived = False
threshold = 1e-3

x_store = np.array([])
y_store = np.array([])
z_store = np.array([])
theta_store = np.array([])


while not arrived:

	# Synchronizing the display of the end_effector frame
	Tee_current = robot.fkine(robot.q)
	robot_ee_frame.T = Tee_current.A

	
	# computing the robot cartesian control (the output robot.qd makes the robot move at the desired velocities)
	robot.qd, arrived,e, theta = cartesian_control(robot, Tee_current.A, Tet.A, K, threshold,arrived)



	x_store = np.r_[x_store, e[0]]
	y_store = np.r_[y_store, e[1]]
	z_store = np.r_[z_store, e[2]]
	theta_store = np.r_[theta_store, theta]

	env.step(dt)
	print("NOT ARRIVED")

print('end')

fig, ax = plt.subplots()

plt.plot(theta_store,'r',markersize=2, label='theta error')
plt.xlabel('sample time')
plt.ylabel('rad')
plt.legend(facecolor='white', framealpha=0)
plt.title('theta error')

fig1, ax1 = plt.subplots()

plt.plot(x_store,'r',markersize=2, label='x error')
plt.xlabel('sample time')
plt.ylabel('m')
plt.legend(facecolor='white', framealpha=0)
plt.title('x error')

fig2, ax2 = plt.subplots()

plt.plot(y_store,'r',markersize=2, label='y error')
plt.xlabel('sample time')
plt.ylabel('m')
plt.legend(facecolor='white', framealpha=0)
plt.title('y error')

fig2, ax2 = plt.subplots()

plt.plot(z_store,'r',markersize=2, label='z error')
plt.xlabel('sample time')
plt.ylabel('m')
plt.legend(facecolor='white', framealpha=0)
plt.title('z error')

plt.show()

#Stop the browser tab from closing
env.hold()


