
import sim
import time
#import core
import math

from PIL import Image as I

import cv2, numpy
import numpy as np


tstep = 5 
jointNum = 6
baseName = 'UR3'
jointName = 'UR3_joint'

M = np.array([[1.,0.,0.,-0.38113],[0.,1.,0.,0.000085324],[0.,0.,1.,0.6511], [0.,0.,0.,1.]])
S = np.array([[0.,-1.,-1.,-1.,0.,-1.], [0.,0.,0.,0.,0.,0.],[1.,0.,0.,0.,1.,0.],[8.55770000000000e-05,0,0,0,8.51300000000000e-05,0],[-0.000119999999999981,-0.108870000000000,-0.352520000000000,-0.565800000000000,0.112225000000000,-0.651100000000000],[0,5.44190000000000e-05,0.000130300000000000,8.51570000000000e-05,0,8.50860000000000e-05]])


# function based on: 
#   https://github.com/simondlevy/OpenCV-Python-Hacks/blob/master/greenball_tracker.py
#  adapted on Apr.9th, 2020 by Xiaoyu Ma
def track_green_object(image):

    # Blur the image to reduce noise
    blur = cv2.GaussianBlur(image, (5,5),0)

    # Convert RGB to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)

    # Threshold the HSV image for only green colors
    lower_green = numpy.array([40,70,70])
    upper_green = numpy.array([80,255,255])

    # Threshold the HSV image
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Blur the mask
    bmask = cv2.GaussianBlur(mask, (5,5),0)

    # Take the moments to get the centroid
    moments = cv2.moments(bmask)
    m00 = moments['m00']
    centroid_x, centroid_y = None, None
    if m00 != 0:
        centroid_x = int(moments['m10']/m00)
        centroid_y = int(moments['m01']/m00)
        #print("found!")
    # Assume no centroid
    ctr = None

    # Use centroid if it exists
    if centroid_x != None and centroid_y != None:
        ctr = (centroid_x, centroid_y)
    return ctr

def track_red_object(image):

    # Blur the image to reduce noise
    blur = cv2.GaussianBlur(image, (5,5),0)

    # Convert RGB to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)
    #print( hsv_red )

    # Threshold the HSV image
    lower_red = numpy.array([0,70,70])
    upper_red = numpy.array([50,255,255])

    # Threshold the HSV image
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # Blur the mask
    bmask = cv2.GaussianBlur(mask, (5,5),0)

    # Take the moments to get the centroid
    moments = cv2.moments(bmask)
    m00 = moments['m00']
    centroid_x, centroid_y = None, None
    if m00 != 0:
        centroid_x = int(moments['m10']/m00)
        centroid_y = int(moments['m01']/m00)
        #print("found!")
    # Assume no centroid
    ctr = None

    # Use centroid if it exists
    if centroid_x != None and centroid_y != None:
        ctr = (centroid_x, centroid_y)
    return ctr


def track_blue_object(image):

    # Blur the image to reduce noise
    blur = cv2.GaussianBlur(image, (5,5),0)

    # Convert RGB to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_RGB2HSV)


    # Threshold the HSV image
    lower_red = numpy.array([110,60,60])
    upper_red = numpy.array([130,255,255])

    # Threshold the HSV image 
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # Blur the mask
    bmask = cv2.GaussianBlur(mask, (5,5),0)

    # Take the moments to get the centroid
    moments = cv2.moments(bmask)
    m00 = moments['m00']
    centroid_x, centroid_y = None, None
    if m00 != 0:
        centroid_x = int(moments['m10']/m00)
        centroid_y = int(moments['m01']/m00)
        #print("found!")
    # Assume no centroid
    ctr = None

    # Use centroid if it exists
    if centroid_x != None and centroid_y != None:
        ctr = (centroid_x, centroid_y)
    return ctr

sim.simxFinish(-1)

clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID!=-1:
  print ('Connected to remote API server')

  # get vision sensor objects
  res, v0 = sim.simxGetObjectHandle(clientID, 'v0', sim.simx_opmode_oneshot_wait)
  res, v1 = sim.simxGetObjectHandle(clientID, 'v1', sim.simx_opmode_oneshot_wait)


  err, resolution, image = sim.simxGetVisionSensorImage(clientID, v0, 0, sim.simx_opmode_streaming)
  time.sleep(1)
  
  
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
  time.sleep(1)
  
  initial_flag = 1

  while (sim.simxGetConnectionId(clientID) != -1):
    # get image from vision sensor 'v0'
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, v0, 0, sim.simx_opmode_buffer)
    if err == sim.simx_return_ok:
      image_byte_array = numpy.array(image,dtype=numpy.uint8)
      image_buffer = I.frombuffer("RGB", (resolution[0],resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
      img2 = numpy.asarray(image_buffer)

      # try to find desired color blocks
      ret = track_green_object(img2)
      ret2 = track_red_object(img2)
      ret3 = track_blue_object(img2)

      # overlay rectangle marker if something is found by OpenCV
      if ret:
        cv2.rectangle(img2,(ret[0]-5,ret[1]-5), (ret[0]+5,ret[1]+5), (0xff,0xf4,0x0d), 1)
        
      if ret2:
        cv2.rectangle(img2,(ret2[0]-5,ret2[1]-5), (ret2[0]+5,ret2[1]+5), (0xff,0xf4,0x0d), 1)  
    
      if ret3:
        cv2.rectangle(img2,(ret3[0]-5,ret3[1]-5), (ret3[0]+5,ret3[1]+5), (0xff,0xf4,0x0d), 1)
        
      #print(ret2)
      #print(ret3)
      # return image to sensor 'v1'
      img2 = img2.ravel()
      sim.simxSetVisionSensorImage(clientID, v1, img2, 0, sim.simx_opmode_oneshot)
      
      beta = 0.39956/26
      
      center_pix_x = (ret2[0]+ret3[0])/2
      center_pix_y = (ret2[1]+ret3[1])/2
      #print(center_pix_y)
      
      theta_cp = math.atan2(ret3[0]-ret2[0],ret3[1]-ret2[0])
      x_wf = ret[0]-center_pix_x
      y_wf = ret[1]-center_pix_y
      r = numpy.sqrt(x_wf**2+y_wf**2)
      y_ob_cf = numpy.cos(theta_cp)*(y_wf+x_wf*numpy.tan(theta_cp))
      x_ob_cf = numpy.sqrt(r**2-y_ob_cf**2)
      
      x_realdis = x_ob_cf*beta
      y_realdis = y_ob_cf*beta
      r_realdis = np.sqrt(x_realdis**2+y_realdis**2)
      #decide the sign, might not be correct by now
      if(ret[0]>ret2[0] and ret[0]>ret3[0]):
          x_realdis = -x_realdis
      if(ret[1]>ret2[1] and ret[1]>ret3[1]):
          y_realdis = -y_realdis
      print(x_realdis)
      print(y_realdis)
      
#      T = np.array([[0.,-1.,0.,x_realdis],[0.,0.,-1.,y_realdis],[1.,0.,0.,0], [0.,0.,0.,1.]])
#      
#      #not used, numerical method
#      
#      thetalist0 = np.array([[math.pi/9],[-math.pi/6],[-math.pi/3],[0],[math.pi/2],[0]])
#      theta_op = thetalist0
#      success=False
#      while(success==False):
#          theta_op,success = core.IKinSpace(S, M, T, thetalist0, 1, 0.001)
#          #print(theta_op)
#          print(success)
#          thetalist0[0] = thetalist0[0]+math.pi/10 
      theta_joint1 = math.atan2(x_realdis,-y_realdis)
      fix = 0.38125
      L3 = 0.24365
      L4 = 0.21328
      L6 = 0.0853
      dz = 0.01
      
      theta1 = math.pi-(numpy.arccos(fix/r_realdis)+math.atan2(x_realdis,-y_realdis))-math.pi/10
      thetalist0 = np.array([[theta1],[math.pi/18],[7*math.pi/9],[-7*math.pi/18],[0],[0]])
      

      d = np.sqrt(dz**2+(np.sqrt(r_realdis**2-fix**2)-L6)**2)
      print("d:",d)
      
    
      
      Left_vel = 0
      Right_vel = 0
      sim.simxSetJointTargetVelocity(clientID,MotorHandle_Left,Left_vel,sim.simx_opmode_oneshot  )
      sim.simxSetJointTargetVelocity(clientID,MotorHandle_Right,Right_vel,sim.simx_opmode_oneshot  )

      
      if(initial_flag!=0):
          sim.simxSetJointTargetPosition(clientID, jointHandle[0], thetalist0[0], sim.simx_opmode_oneshot)
          sim.simxSetJointTargetPosition(clientID, jointHandle[1], thetalist0[1], sim.simx_opmode_oneshot)
          sim.simxSetJointTargetPosition(clientID, jointHandle[2], thetalist0[2], sim.simx_opmode_oneshot)
          sim.simxSetJointTargetPosition(clientID, jointHandle[3], thetalist0[3], sim.simx_opmode_oneshot)
          sim.simxSetJointTargetPosition(clientID, jointHandle[4], thetalist0[4], sim.simx_opmode_oneshot)
          sim.simxSetJointTargetPosition(clientID, jointHandle[5], thetalist0[5], sim.simx_opmode_oneshot)
      
      time.sleep(1)
      
      #detect again
      image_byte_array = numpy.array(image,dtype=numpy.uint8)
      image_buffer = I.frombuffer("RGB", (resolution[0],resolution[1]), image_byte_array, "raw", "RGB", 0, 1)
      img2 = numpy.asarray(image_buffer)
      
      ret = track_green_object(img2)
      ret2 = track_red_object(img2)
      ret3 = track_blue_object(img2)

      # overlay rectangle marker if something is found by OpenCV
      if ret:
        cv2.rectangle(img2,(ret[0]-5,ret[1]-5), (ret[0]+5,ret[1]+5), (0xff,0xf4,0x0d), 1)
        
      if ret2:
        cv2.rectangle(img2,(ret2[0]-5,ret2[1]-5), (ret2[0]+5,ret2[1]+5), (0xff,0xf4,0x0d), 1)  
    
      if ret3:
        cv2.rectangle(img2,(ret3[0]-5,ret3[1]-5), (ret3[0]+5,ret3[1]+5), (0xff,0xf4,0x0d), 1)
        
      #print(ret2)
      #print(ret3)
      # return image to sensor 'v1'
      img2 = img2.ravel()
      sim.simxSetVisionSensorImage(clientID, v1, img2, 0, sim.simx_opmode_oneshot)
      
      beta = 0.39956/26
      
      center_pix_x = (ret2[0]+ret3[0])/2
      center_pix_y = (ret2[1]+ret3[1])/2
      #print(center_pix_y)
      
      theta_cp = math.atan2(ret3[0]-ret2[0],ret3[1]-ret2[0])
      x_wf = ret[0]-center_pix_x
      y_wf = ret[1]-center_pix_y
      r = numpy.sqrt(x_wf**2+y_wf**2)
      y_ob_cf = numpy.cos(theta_cp)*(y_wf+x_wf*numpy.tan(theta_cp))
      x_ob_cf = numpy.sqrt(r**2-y_ob_cf**2)
      
      x_realdis = x_ob_cf*beta
      y_realdis = y_ob_cf*beta
      r_realdis = np.sqrt(x_realdis**2+y_realdis**2)
      #decide the sign, might not be correct by now
      if(ret[0]>ret2[0] and ret[0]>ret3[0]):
          x_realdis = -x_realdis
      if(ret[1]>ret2[1] and ret[1]>ret3[1]):
          y_realdis = -y_realdis
      
      
      
      
      theta1 = math.pi-(numpy.arccos(fix/r_realdis)+math.atan2(x_realdis,-y_realdis))
      theta3 = math.pi-np.arccos((L3**2+L4**2-d**2)/(2*L3*L4))
      theta2 = math.pi-math.atan2(d,dz)-np.arccos((L3**2+d**2-L4**2)/(2*L3*d))
      theta4 = math.pi/2-theta2-theta3
      theta5 = 0
      theta6 = 0
      initial_flag = 0
      print(theta1)
      print(theta2)
      print(theta3)
      print(theta4)
      
      
      sim.simxSetJointTargetPosition(clientID, jointHandle[0], theta1, sim.simx_opmode_oneshot)
      time.sleep(1)
      sim.simxSetJointTargetPosition(clientID, jointHandle[1], theta2, sim.simx_opmode_oneshot)
      sim.simxSetJointTargetPosition(clientID, jointHandle[2], theta3, sim.simx_opmode_oneshot)
      sim.simxSetJointTargetPosition(clientID, jointHandle[3], theta4, sim.simx_opmode_oneshot)
      time.sleep(1)
      sim.simxSetJointTargetPosition(clientID, jointHandle[4], theta5, sim.simx_opmode_oneshot)
      sim.simxSetJointTargetPosition(clientID, jointHandle[5], theta6, sim.simx_opmode_oneshot)
      
      
      
      
    
    elif err == sim.simx_return_novalue_flag:
      print ("no image yet")
      pass
    else:
      print (err)
else:
  print ("Failed to connect to remote API Server")
  sim.simxFinish(clientID)