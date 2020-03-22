#!/usr/bin/env python
from __future__ import division
import numpy as np
from scipy.linalg import expm

import numpy as np
import math
import sim

tstep = 5 
jointNum = 6
baseName = 'UR3'
jointName = 'UR3_joint'


"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))
	
	# find M and S by measuring the UR3 robot arm
	M = np.array([[1.,0.,0.,-0.38113],[0.,1.,0.,0.000085324],[0.,0.,1.,0.6511], [0.,0.,0.,1.]])
	S = np.array([[0.,-1.,-1.,-1.,0.,-1.], [0.,0.,0.,0.,0.,0.],[1.,0.,0.,0.,1.,0.],[8.55770000000000e-05,0,0,0,8.51300000000000e-05,0],[-0.000119999999999981,-0.108870000000000,-0.352520000000000,-0.565800000000000,0.112225000000000,-0.651100000000000],[0,5.44190000000000e-05,0.000130300000000000,8.51570000000000e-05,0,8.50860000000000e-05]])

	
	# ==============================================================#
	return M, S



"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

	s1_mat = np.array([[0, -S[2][0], S[1][0], S[3][0]],[S[2][0], 0, -S[0][0], S[4][0]], [-S[1][0], S[0][0], 0, S[5][0]], [0,0,0,0]])
	s2_mat = np.array([[0, -S[2][1], S[1][1], S[3][1]],[S[2][1], 0, -S[0][1], S[4][1]], [-S[1][1], S[0][1], 0, S[5][1]], [0,0,0,0]])
	s3_mat = np.array([[0, -S[2][2], S[1][2], S[3][2]],[S[2][2], 0, -S[0][2], S[4][2]], [-S[1][2], S[0][2], 0, S[5][2]], [0,0,0,0]])
	s4_mat = np.array([[0, -S[2][3], S[1][3], S[3][3]],[S[2][3], 0, -S[0][3], S[4][3]], [-S[1][3], S[0][3], 0, S[5][3]], [0,0,0,0]])
	s5_mat = np.array([[0, -S[2][4], S[1][4], S[3][4]],[S[2][4], 0, -S[0][4], S[4][4]], [-S[1][4], S[0][4], 0, S[5][4]], [0,0,0,0]])
	s6_mat = np.array([[0, -S[2][5], S[1][5], S[3][5]],[S[2][5], 0, -S[0][5], S[4][5]], [-S[1][5], S[0][5], 0, S[5][5]], [0,0,0,0]])

	T = expm(s1_mat*theta1/(180 / math.pi)).dot(expm(s2_mat*theta2/(180 / math.pi))).dot(expm(s3_mat*theta3/(180 / math.pi))).dot(expm(s4_mat*theta4/(180 / math.pi))).dot(expm(s5_mat*theta5/(180 / math.pi))).dot(expm(s6_mat*theta6/(180 / math.pi))).dot(M)



	# ==============================================================#
	
	print(str(T) + "\n")

	return_value[0] = theta1
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


print('Program started')
sim.simxFinish(-1) #close all potential connection
#try to connect every 0.2 seconds
while True:
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID > -1:
        break
    else:
        time.sleep(0.2)
        print("Failed  to connect!")
print("Connection successful!")

sim.simxSetFloatingParameter(clientID, sim.sim_floatparam_simulation_time_step, tstep, sim.simx_opmode_oneshot)
# start synchronize
sim.simxSynchronous(clientID, True) 
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

# read for UR3 handles(joints and base)
jointHandle = np.zeros((jointNum,), dtype=np.int) # 注意是整型
for i in range(jointNum):
    _, returnHandle = sim.simxGetObjectHandle(clientID, jointName + str(i+1), sim.simx_opmode_blocking)
    jointHandle[i] = returnHandle
_, baseHandle = sim.simxGetObjectHandle(clientID, baseName, sim.simx_opmode_blocking)

_,MotorHandle_Left = sim.simxGetObjectHandle(clientID,'Revolute_left',sim.simx_opmode_blocking)
_,MotorHandle_Right = sim.simxGetObjectHandle(clientID,'Revolute_right',sim.simx_opmode_blocking)

_,TCP = sim.simxGetObjectHandle(clientID,'TCP',sim.simx_opmode_blocking)
#_, baseHandle = sim.simxGetObjectHandle(clientID, 'CarBody', sim.simx_opmode_blocking)
print('Handles Found!')

# read the initial value of joints
jointConfig = np.zeros((jointNum,))
for i in range(jointNum):
    _, jpos = sim.simxGetJointPosition(clientID, jointHandle[i], sim.simx_opmode_streaming)
    jointConfig[i] = jpos



lastCmdTime=sim.simxGetLastCmdTime(clientID)  # record the current time
sim.simxSynchronousTrigger(clientID)  


#iii = 0 #loop index
#increase_flag = 0
# trigger the sim
while sim.simxGetConnectionId(clientID) != -1:
    currCmdTime=sim.simxGetLastCmdTime(clientID)
    dt = currCmdTime - lastCmdTime # delta time
    # read the current status of joints for every step
    for i in range(jointNum):
        _, jpos = sim.simxGetJointPosition(clientID, jointHandle[i], sim.simx_opmode_buffer)
        # print(round(jpos * 180 / math.pi, 2))
        jointConfig[i] = jpos
        
    _,MotorHandle_Left = sim.simxGetObjectHandle(clientID,'Revolute_left',sim.simx_opmode_blocking)
    _,MotorHandle_Right = sim.simxGetObjectHandle(clientID,'Revolute_right',sim.simx_opmode_blocking)
    _,TCP = sim.simxGetObjectHandle(clientID,'TCP',sim.simx_opmode_blocking)
    theta1 = 100
    theta2 = 20
    theta3 = 0
    theta4 = 0
    theta5 = 30
    theta6 = 0
    Left_vel = 0
    Right_vel = 0


    theta_out = lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)

    # pause the stream to wait for command on all joints
    sim.simxPauseCommunication(clientID, True)
    sim.simxSetJointTargetPosition(clientID, jointHandle[0], theta_out[0]/(180 / math.pi), sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, jointHandle[1], theta_out[1]/(180 / math.pi), sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, jointHandle[2], theta_out[2]/(180 / math.pi), sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, jointHandle[3], theta_out[3]/(180 / math.pi), sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, jointHandle[4], theta_out[4]/(180 / math.pi), sim.simx_opmode_oneshot)
    sim.simxSetJointTargetPosition(clientID, jointHandle[5], theta_out[5]/(180 / math.pi), sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID,MotorHandle_Left,Left_vel,sim.simx_opmode_oneshot  )
    sim.simxSetJointTargetVelocity(clientID,MotorHandle_Right,Right_vel,sim.simx_opmode_oneshot  )
    
    #if(iii==90):
       # increase_flag = 1
    #if(iii==0):
       # increase_flag = 0
    #if(increase_flag==0):    
       # iii = iii+1
    #else:
        #iii = iii-1

    print('real position:',sim.simxGetObjectPosition(clientID,TCP,baseHandle,sim.simx_opmode_oneshot))
    
    sim.simxPauseCommunication(clientID, False)
    lastCmdTime=currCmdTime    
    sim.simxSynchronousTrigger(clientID)  # on next step
    sim.simxGetPingTime(clientID)    


