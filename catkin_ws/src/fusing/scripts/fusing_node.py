#!/usr/bin/env python2

# Imports
import sys
import select
import os
from threading import Thread
from threading import Lock
import collections
import numpy as np
import random
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import rospy
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import uwb.msg
from geometry_msgs.msg import PointStamped
#from tf2_msgs import TFMessage
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.msg
import cv2

## Fusing class: Fuses the information from UWB and for the vision tracker with an Extenden Kalmanfilter (EKF)
class Fusing:

  ## Class represents the flag for new values
  class NewValue:
    newValue = False
    newUWB = False
    newVision = False

  ## The Constructor of Fusing class
  def __init__(self):

    # ROS init
    rospy.init_node('fusing', anonymous=True)

    # Coordinates received from the UWB
    self.uwb_x_wc = 0.0
    self.uwb_y_wc = 0.0
    self.uwb_z_wc = 0.0
    # Velocities recived from the UWB
    self.uwb_vx_wc = 0.0
    self.uwb_vy_wc = 0.0
    self.uwb_vz_wc = 0.0

    # Coordinates received from the vision tracker
    self.vision_x_wc = 0.0
    self.vision_y_wc = 0.0
    self.vision_z_wc = 0.0

    self.vision_x_uv = 0.0
    self.vision_y_uv = 0.0
    self.vision_z_uv = 0.0

    # Fused coordinates from the states of the EKF
    #self.state = np.array([[320.0], [160.0], [1.0], [0.0], [0.0], [0.0]])
    self.state = np.array([[0.01], [-0.04], [1.0], [0.0], [0.0], [0.0]])
    #self.state = np.empty((6, 1))

    # EKF
    self.newValue = self.NewValue()
    self.lastTimeStamp = rospy.Time.now()
    self.deltaT = 0.0035
    # Initial values
    self.vecXm = [0, 0]
    self.matPm = np.zeros((6,6))
    #self.matPm = 5*np.identity(6)
    #self.matQ = np.vstack((np.zeros((3,6)), np.hstack((np.zeros((3,3)), 100*np.identity(3))))) # Covariance matrix of model with only variances for the velocity
    self.matQ = np.vstack((np.hstack((40*np.identity(3), np.zeros((3,3)))), np.hstack((np.zeros((3,3)), 100*np.identity(3))))) # Covariance matrix of the model with variances for position and velocity
    self.matR1 = np.identity(6)
    self.matR2 = 10**(-7)*np.identity(2)
    # Constant matrices used by the EKF
    #self.matA = np.vstack((np.hstack((np.identity(3), self.deltaT * np.identity(3))), np.hstack((np.zeros((3,3)), np.identity(3)))))

    # ROS message
    self.ekf_coordinates_msg = geometry_msgs.msg.PointStamped()
    self.header_seq = 0

    self.object_detected = True


    # Prototyping
    self.mutex_newValue = Lock()
    self.mutex_vision = Lock()
    self.mutex_uwb = Lock()
    self.mutex_image = Lock()
    self.mutex_state = Lock()
    self.uwb_x_wc_list = collections.deque(maxlen=200)
    self.uwb_y_wc_list = collections.deque(maxlen=200)
    self.uwb_z_wc_list = collections.deque(maxlen=200)
    self.vision_x_uv_list = collections.deque(maxlen=200)
    self.vision_y_uv_list = collections.deque(maxlen=200)
    self.x_state_list = collections.deque(maxlen=200)
    self.y_state_list = collections.deque(maxlen=200)
    self.lv = 1
    self.realState = []
    self.allStatesX = np.zeros(1)
    self.allStatesY = np.zeros(1)
    self.allStatesZ = np.zeros(1)
    self.allUWBX = np.zeros(1)
    self.allUWBY = np.zeros(1)
    self.allUWBZ = np.zeros(1)
    self.allVisionX = np.zeros(1)
    self.allVisionY = np.zeros(1)
    self.allVisionZ = np.zeros(1)

    self.bridge = CvBridge()

    self.cv_image = None

    #plt.ion()
    #plt.axis([-1.5, 1.5, -1.5, 1.5])
    #plt.axis('equal')
    fig = plt.figure()
    self.ax = fig.gca(projection='3d')
    #self.jet = plt.get_cmap('jet')

  ## Callback function to receive the UWB messages from ROS.
  def uwb_callback(self, data):
    # uwb_1: video_uwb_1: lrms=0.1093
    #uwb_x = data.state[0] + 0.1151
    #uwb_y = data.state[1] - 0.0139
    #uwb_z = data.state[2] + 0.0304

    # uwb_7: video_uwb_7: lrms=0.0782
    #uwb_x = data.state[0] + 0.2049
    #uwb_y = data.state[1] - 0.0568
    #uwb_z = data.state[2] - 0.0149

    # uwb_8: video_uwb_8: lrms=0.1088
    #uwb_x = data.state[0] + 0.2586
    #uwb_y = data.state[1] - 0.1066
    #uwb_z = data.state[2] - 0.0077

    # uwb_11: video_uwb_11: lrms=0.0703
    uwb_x = data.state[0] - 0.0327
    uwb_y = data.state[1] - 0.0143
    uwb_z = data.state[2] + 0.0038

    uwb_vx = data.state[3]
    uwb_vy = data.state[4]
    uwb_vz = data.state[5]
    #print("UWB raw: x = {0}, y = {1}, z = {2}".format(uwb_x, uwb_y, uwb_z))

    self.mutex_uwb.acquire(1)
    self.matR1 = 100*np.array([[data.covariance[0], data.covariance[1], data.covariance[2], \
                            data.covariance[3], data.covariance[4], data.covariance[5]], \
                           [data.covariance[6], data.covariance[7], data.covariance[8], \
                            data.covariance[9], data.covariance[10], data.covariance[11]], \
                           [data.covariance[12], data.covariance[13], data.covariance[14], \
                            data.covariance[15], data.covariance[16], data.covariance[17]], \
                           [data.covariance[18], data.covariance[19], data.covariance[20], \
                            data.covariance[21], data.covariance[22], data.covariance[23]], \
                           [data.covariance[24], data.covariance[25], data.covariance[26], \
                            data.covariance[27], data.covariance[28], data.covariance[29]], \
                           #[10**(-5), 10**(-5), 10**(-5), \
                           # 10**(-5), 10**(-5), 10**(-5)]])
                           [data.covariance[30], data.covariance[31], data.covariance[32], \
                            data.covariance[33], data.covariance[34], data.covariance[35]]])
    if self.object_detected==True:
      self.matR1 =  50**(2) * self.matR1
    #print("UWB cov: {0}".format((self.matR1)))

    # uwb_1: video_uwb_1: lrms=0.1093
    #self.uwb_x_wc = 0.9819 * ( 0.3072 * uwb_x - 0.9216 * uwb_y - 0.0086 * uwb_z)
    #self.uwb_y_wc = 0.9819 * ( 0.1335 * uwb_x + 0.0521 * uwb_y - 0.9897 * uwb_z)
    #self.uwb_z_wc = 0.9819 * ( 0.9422 * uwb_x + 0.3029 * uwb_y + 0.1430 * uwb_z)

    #self.uwb_vx_wc = 0.9819 * ( 0.3072 * uwb_vx - 0.9216 * uwb_vy - 0.0086 * uwb_vz)
    #self.uwb_vy_wc = 0.9819 * ( 0.1335 * uwb_vx + 0.0521 * uwb_vy - 0.9897 * uwb_vz)
    #self.uwb_vz_wc = 0.9819 * ( 0.9422 * uwb_vx + 0.3029 * uwb_vy + 0.1430 * uwb_vz)

    # uwb_7: video_uwb_7: lrms=0.0782
    #self.uwb_x_wc = 0.9356 * ( 0.3046 * uwb_x - 0.9441 * uwb_y + 0.1257 * uwb_z)
    #self.uwb_y_wc = 0.9356 * ( 0.2016 * uwb_x - 0.0650 * uwb_y - 0.9773 * uwb_z)
    #self.uwb_z_wc = 0.9356 * ( 0.9309 * uwb_x + 0.3231 * uwb_y + 0.1705 * uwb_z)

    #self.uwb_vx_wc = 0.9356 * ( 0.3046 * uwb_vx - 0.9441 * uwb_vy + 0.1257 * uwb_vz)
    #self.uwb_vy_wc = 0.9356 * ( 0.2016 * uwb_vx - 0.0650 * uwb_vy - 0.9773 * uwb_vz)
    #self.uwb_vz_wc = 0.9356 * ( 0.9309 * uwb_vx + 0.3231 * uwb_vy + 0.1705 * uwb_vz)

    # uwb_8: video_uwb_8: lrms=0.1088
    #self.uwb_x_wc = 0.9388*( 0.3169*uwb_x - 0.9389*uwb_y + 0.1342*uwb_z)
    #self.uwb_y_wc = 0.9388*( 0.2742*uwb_x - 0.0447*uwb_y - 0.9606*uwb_z)
    #self.uwb_z_wc = 0.9388*( 0.9080*uwb_x + 0.3412*uwb_y + 0.2433*uwb_z)

    #self.uwb_vx_wc = 0.9388*( 0.3169*uwb_vx - 0.9389*uwb_vy + 0.1342*uwb_vz)
    #self.uwb_vy_wc = 0.9388*( 0.2742*uwb_vx - 0.0447*uwb_vy - 0.9606*uwb_vz)
    #self.uwb_vz_wc = 0.9388*( 0.9080*uwb_vx + 0.3412*uwb_vy + 0.2433*uwb_vz)

    # uwb_11: video_uwb_11: lrms=0.0703
    self.uwb_x_wc = 1.0339*( 0.1227*uwb_x - 0.9923*uwb_y + 0.0193*uwb_z)
    self.uwb_y_wc = 1.0339*(-0.0729*uwb_x - 0.0284*uwb_y - 0.9969*uwb_z)
    self.uwb_z_wc = 1.0339*( 0.9898*uwb_x + 0.1209*uwb_y - 0.0758*uwb_z)

    self.uwb_vx_wc = 1.0339*( 0.1227*uwb_vx - 0.9923*uwb_vy + 0.0193*uwb_vz)
    self.uwb_vy_wc = 1.0339*(-0.0729*uwb_vx - 0.0284*uwb_vy - 0.9969*uwb_vz)
    self.uwb_vz_wc = 1.0339*( 0.9898*uwb_vx + 0.1209*uwb_vy - 0.0758*uwb_vz)

    self.uwb_x_uv = 593.16120354*self.uwb_x_wc/self.uwb_z_wc + 308.67164248
    self.uwb_y_uv = 589.605859*self.uwb_y_wc/self.uwb_z_wc + 245.3659398

    #self.uwb_x_uv = 593.16120354 * self.uwb_x_wc + 308.67164248 * self.uwb_z_wc
    #self.uwb_y_uv = 589.605859 * self.uwb_y_wc + 245.3659398 * self.uwb_z_wc
    self.uwb_z_uv = self.uwb_z_wc

    #self.uwb_vx_uv = 593.16120354 * self.uwb_vx_wc + 308.67164248 * self.uwb_vz_wc
    #self.uwb_vy_uv = 589.605859 * self.uwb_vy_wc + 245.3659398 * self.uwb_vz_wc
    #self.uwb_vz_uv = self.uwb_vz_wc

    self.uwb_x_wc_list.append(self.uwb_x_wc)
    self.uwb_y_wc_list.append(self.uwb_y_wc)
    self.uwb_z_wc_list.append(self.uwb_z_wc)
    self.mutex_uwb.release()

    self.mutex_newValue.acquire(1)
    self.newValue.newValue = True
    self.newValue.newUWB = True
    self.mutex_newValue.release()

  ## Callback function to receive the vision tracker coordinates messages from ROS.
  def vision_tracker_coordinates_callback(self, data):
    self.mutex_vision.acquire(1)
    self.vision_x_uv = data.point.x
    self.vision_y_uv = data.point.y

    self.vision_x_wc = 0.0017 * self.vision_x_uv - 0.5204
    self.vision_y_wc = 0.0017 * self.vision_y_uv - 0.4162
    self.vision_z_wc = 1

    self.vision_x_uv_list.append(self.vision_x_uv)
    self.vision_y_uv_list.append(self.vision_y_uv)
    self.mutex_vision.release()

    self.mutex_newValue.acquire(1)
    self.newValue.newValue = True
    self.newValue.newVision = True
    self.mutex_newValue.release()

  ## Callback function to receive the vision tracker object detected messages from ROS.
  def vision_tracker_object_detected_callback(self, data):
    self.object_detected = data.data

  ## Callback function to receive the image messages from ROS
  def image_callback(self, data):
    self.mutex_image.acquire(1)
    # For uncompressed images
    """
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    """
    # For compressed images
    np_arr = np.fromstring(data.data, np.uint8)
    self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    self.mutex_image.release()

  ## Performs one iteration of the EKF.
  def ekf_iteration(self):
    # Step 1
    #vecState_p = np.dot(self.matB, self.vecX)
    # Get the time difference
    deltaT = rospy.Time.now() - self.lastTimeStamp
    self.lastTimeStamp = rospy.Time.now()

    # Compute the matrix B, the vector state_p and the matrix P_p
    matB = np.vstack((np.hstack((np.identity(3), deltaT.to_sec() * np.identity(3))), np.hstack((np.zeros((3,3)), np.identity(3)))))
    #print("B = {0}".format(matB))
    #print("deltaT = {0}".format(deltaT.to_sec()))
    vecState_p = np.dot(matB, self.state)
    #print("vecState_p = {0}".format(vecState_p))
    matP_p = np.dot(matB, np.dot(self.matPm, matB.transpose())) + self.matQ # B*Pm*B^T + Q
    #print("matP_p = {0}".format(matP_p))

    # Step 2
    #matH = np.array([[np.identity(6)], [1/vecState_p[2], 0, -vecState_p[0]/vecState_p[3]**2, 0, 0, 0],\
    #                 [0, 1/vecState_p[2], -vecState_p[1]/vecState_p[3]**2, 0, 0, 0]])
    # Compute the matrix H, R and K
    matH_top = np.identity(6)
    matH_bottom = np.array([[1/vecState_p[2][0], 0, -vecState_p[0][0]/(vecState_p[2][0]**2), 0, 0, 0], \
                     [0, 1/vecState_p[2][0], -vecState_p[1][0]/(vecState_p[2][0]**2), 0, 0, 0]])
    matH = np.vstack((matH_top,matH_bottom))
    #matK_t1 = np.dot(self.matQ, matH.transpose()) # Q * H^T
    #matK_t2 = np.dot(matH, matK_t1) # H*Q*H^T
    matK_t1 = np.dot(matP_p, matH.transpose()) # P_p * H^T
    matK_t2 = np.dot(matH, matK_t1) # H*P_p*H^T
    matR = np.vstack((np.hstack((self.matR1, np.zeros((6,2)))), np.hstack((np.zeros((2,6)), self.matR2))))

    matK = np.dot(matK_t1, np.linalg.inv(matK_t2 + matR)) # (P_p * H^T) * (H*P_p*H^T + [R_1, 0; 0, R_2])^(-1)
    #print("Q = {0}".format(self.matQ))
    #print("R = {0}".format(matR))
    #print("K = {0}".format(matK))

    #vecStatem_t1 = np.array([[vecState_p[0][0]/vecState_p[2][0]], [vecState_p[1][0]/vecState_p[2][0]]])
    vecStatem_t1 = np.array([[vecState_p[0][0]/vecState_p[2][0]], [vecState_p[1][0]/vecState_p[2][0]]])

    vecZ = np.zeros((8,1))
    # Checks wheter new UWB data and/or new vision data is available.
    if (self.newValue.newUWB==True and self.newValue.newVision==True and self.object_detected==True):
      self.mutex_uwb.acquire(1)
      vecZ[0][0] = self.uwb_x_wc
      vecZ[1][0] = self.uwb_y_wc
      vecZ[2][0] = self.uwb_z_wc
      vecZ[3][0] = self.uwb_vx_wc
      vecZ[4][0] = self.uwb_vy_wc
      vecZ[5][0] = self.uwb_vz_wc
      self.mutex_uwb.release()
      self.mutex_vision.acquire(1)
      vecZ[6][0] = self.vision_x_wc
      vecZ[7][0] = self.vision_y_wc
      self.mutex_vision.release()
      #self.matR1[0][0] =
      vecStatem_t2 = vecZ - np.vstack(((vecState_p, vecStatem_t1))) # z - [H1*x_p; H2(x_p)]
      #print("h = {0}".format(np.vstack(((vecState_p, vecStatem_t1)))))
    elif (self.newValue.newUWB==True and (self.newValue.newVision==False or (self.newValue.newVision==True and self.object_detected==False))):
      self.mutex_uwb.acquire(1)
      vecZ[0][0] = self.uwb_x_wc
      vecZ[1][0] = self.uwb_y_wc
      vecZ[2][0] = self.uwb_z_wc
      vecZ[3][0] = self.uwb_vx_wc
      vecZ[4][0] = self.uwb_vy_wc
      vecZ[5][0] = self.uwb_vz_wc
      self.mutex_uwb.release()
      vecZ[6][0] = 0
      vecZ[7][0] = 0
      vecStatem_t2 = vecZ - np.vstack(((vecState_p, np.zeros((2, 1))))) # z - [H1*x; H2(x)]
      #print("h = {0}".format(np.vstack(((vecState_p, np.zeros((2, 1)))))))
    elif (self.newValue.newUWB==False and self.newValue.newVision==True and self.object_detected==True):
      vecZ[0][0] = 0
      vecZ[1][0] = 0
      vecZ[2][0] = 0
      vecZ[3][0] = 0
      vecZ[4][0] = 0
      vecZ[5][0] = 0
      self.mutex_vision.acquire(1)
      vecZ[6][0] = self.vision_x_wc
      vecZ[7][0] = self.vision_y_wc
      self.mutex_vision.release()
      vecStatem_t2 = vecZ - np.vstack(((np.zeros((6, 1)), vecStatem_t1))) # z - [H1*x; H2(x)]
      #print("h = {0}".format(np.vstack(((np.zeros((6, 1)), vecStatem_t1)))))


    # Estimation of the new state
    #vecStatem_t2 = vecZ - np.vstack(((vecState_p, vecStatem_t1))) # z - [H1*x; H2(x)]
    #print("z = {0}".format(vecZ))
    #print("h = {0}".format(np.vstack(((vecState_p, vecStatem_t1)))))
    #print("x_p = {0}".format(vecState_p))
    #print("K = {0}".format(matK))
    #print("(z - h) = {0}".format(vecStatem_t2))
    vecStatem_t3 = np.dot(matK, vecStatem_t2) # K*(z - [H1*x; H2(x)])
    #print(" + {0}".format(vecStatem_t3[5]))
    self.mutex_state.acquire(1)
    self.state = vecState_p + vecStatem_t3
    self.mutex_state.release()


    #self.state_x_uv = 593.16120354 * self.state[0] + 308.67164248 * self.state[2]
    #self.state_y_uv = 589.605859 * self.state[1] + 245.3659398 * self.state[2]

    self.state_x_uv = 593.16120354*self.state[0]/self.state[2] + 308.67164248
    self.state_y_uv = 589.605859*self.state[1]/self.state[2] + 245.3659398
    #self.state_x_uv = self.state[0]
    #self.state_y_uv = self.state[1]
    #self.state_z_uv = self.state[2]
    self.x_state_list.append(self.state[0])
    self.y_state_list.append(self.state[1])

    # Estimation of the new coovariance matrix
    matPm_t = np.dot(matK, matH) # K*H
    self.mutex_state.acquire(1)
    self.matPm = np.dot(np.identity(6) - matPm_t, matP_p) # (I - K*H)*Pp
    #print("matPm = {0}".format(self.matPm))
    self.mutex_state.release()

    # Reset flags
    self.newValue.newValue = False
    self.newValue.newVision = False
    self.newValue.newUWB = False

  ## Simulatin input data
  def simulateInput(self):
    self.lv = self.lv + 0.6
    self.realState.append(np.array([[self.lv], [self.lv], [1], [5*random.random()], [5*random.random()], [5*random.random()]]))
    matQ = np.vstack((np.zeros((3,6)), np.hstack((np.zeros((3,3)), 10*np.identity(3)))))
    matR1 = np.identity(6)
    matR2 = np.identity(2)
    #vecZ = np.array([[self.state[0][0] + 10*random.random()], [self.state[1][0] + 10*random.random()], [self.state[2][0] + 10*random.random()],\
    #        [self.state[3][0] + 10*random.random()], [self.state[4][0] + 10*random.random()], [self.state[5][0] + 10*random.random()],\
    #        [self.state[0][0] + 10*random.random()], [self.state[1][0] + 10*random.random()]])
    vecZ = np.array([[self.realState[-1][0][0] + 1*random.random()], [self.realState[-1][1][0] + 1*random.random()], [self.realState[-1][2][0] + 1*random.random()], \
                     [self.realState[-1][3][0] + 1*random.random()], [self.realState[-1][4][0] + 1*random.random()], [self.realState[-1][5][0] + 1*random.random()], \
                     [self.realState[-1][0][0] + 1*random.random()], [self.realState[-1][1][0] + 1*random.random()]])

    self.uwb_x_wc = vecZ[0][0]
    self.uwb_y_wc = vecZ[1][0]
    self.uwb_z_wc = vecZ[2][0]
    self.uwb_vx_wc = vecZ[3][0]
    self.uwb_vy_wc = vecZ[4][0]
    self.uwb_vz_wc = vecZ[5][0]
    self.vision_x_uv = vecZ[6][0]
    self.vision_y_uv = vecZ[7][0]

    self.newValue.newValue = True
    self.newValue.newUWB = True
    self.newValue.newVision = True
    return(matQ, matR1, matR2, vecZ)

  ## Initialize ROS
  def initROS(self):
    self.sub_uwb = rospy.Subscriber('/uwb/tracker', uwb.msg.UWBTracker, self.uwb_callback)
    self.sub_vision_coordinates = rospy.Subscriber('/vision_tracker/vision_coordinates', geometry_msgs.msg.PointStamped, self.vision_tracker_coordinates_callback)
    self.sub_vision_object_detected = rospy.Subscriber('/vision_tracker/object_detected', std_msgs.msg.Bool, self.vision_tracker_object_detected_callback)
    #self.sub_img = rospy.Subscriber('/vision_tracker/video', sensor_msgs.msg.Image, self.image_callback)
    self.sub_img = rospy.Subscriber('/camera/video/compressed', sensor_msgs.msg.CompressedImage, self.image_callback)

    self.pub = rospy.Publisher('/fusing/ekf_coordinates', geometry_msgs.msg.PointStamped, queue_size=1)

  def start(self):

    while not rospy.is_shutdown():
      os.system('cls' if os.name == 'nt' else 'clear')
      if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = raw_input()
        rospy.signal_shutdown("terminate")
        #break

      rospy.sleep(0.1)

      """
      try:
        trans = tfBuffer.lookup_transform_full(
        target_frame='vision',
        target_time=rospy.Time.now(),
        source_frame='world',
        source_time=rospy.Time.now(),
        timeout=rospy.Duration(0.0))
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.sleep(0.1)
        continue
      """

      #[matQ, matR1, matR2, vecZ] = self.simulateInput()

      #self.mutex_newValue.acquire(1)
      if self.newValue.newValue==True:
        self.ekf_iteration()

        # Display of state trajectory
        self.allStatesX = np.append(self.allStatesX, self.state[0])
        self.allStatesY = np.append(self.allStatesY, self.state[1])
        self.allStatesZ = np.append(self.allStatesZ, self.state[2])
        self.allUWBX = np.append(self.allUWBX, self.uwb_x_wc)
        self.allUWBY = np.append(self.allUWBY, self.uwb_y_wc)
        self.allUWBZ = np.append(self.allUWBZ, self.uwb_z_wc)
        self.allVisionX = np.append(self.allVisionX, self.vision_x_wc)
        self.allVisionY = np.append(self.allVisionY, self.vision_y_wc)
        self.allVisionZ = np.append(self.allVisionZ, self.vision_z_wc)
        if self.object_detected==True:
          self.ax.plot(self.allStatesX, self.allStatesZ, -self.allStatesY, label='trajectory', color='green')
        else:
          self.ax.plot(self.allStatesX, self.allStatesZ, -self.allStatesY, label='trajectory', color='yellow')
        self.ax.plot(self.allUWBX, self.allUWBZ, -self.allUWBY, label='trajectory', color='red')
        self.ax.plot(self.allVisionX, self.allVisionY, -self.allVisionY, label='trajectory', color='blue')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Z')
        self.ax.set_zlabel('Y')
        plt.draw()
        plt.pause(0.01)

        # Display of vision and the projection of uwb
        """
        plt.clf()
        plt.ion()
        plt.axis([-1.5, 1.5, -1.5, 1.5])
        self.mutex_vision.acquire(1)
        vision_plt = plt.scatter(self.vision_x_uv_list, self.vision_y_uv_list, c='b', marker='o')
        #print("x vision mean: ",format(np.mean(self.vision_x_uv_list)))
        #print("y vision mean: ",format(np.mean(self.vision_y_uv_list)))
        self.mutex_vision.release()
        self.mutex_uwb.acquire(1)
        #uwbPlt = plt.scatter(self.uwb_x_wc_list, self.uwb_y_wc_list, c='r', marker='o')
        uwb_x = np.array(self.uwb_x_wc_list)
        uwb_y = np.array(self.uwb_y_wc_list)
        uwb_z = np.array(self.uwb_z_wc_list)
        print("x div: ",format(abs(self.uwb_x_wc_list[-1] - self.vision_x_uv_list[-1])))
        print("y div: ",format(abs(self.uwb_y_wc_list[-1] - self.vision_y_uv_list[-1])))
        uwb_plt = plt.scatter(self.uwb_x_wc_list, self.uwb_y_wc_list, c='r', marker='o')
        self.mutex_uwb.release()
        ekf_plt = plt.scatter(self.x_state_list, self.y_state_list, c='g', marker='x')
        #plt.legend(handles=[vision_plt, uwb_plt, ekf_plt])#, aruco_transf_plt, uwb_plt])
        plt.pause(0.05)  # Display of vision and the projection of uwb
        """

        # Display of state trajectory  OLD
        """
        #self.hl.set_xdata(np.append(self.hl.get_xdata(), self.state[0][0]))
        #self.hl.set_ydata(np.append(self.hl.get_ydata(), self.state[1][0]))
        #plt.scatter(self.state[0][0], self.state[1][0], s=100, c="r", cmap=self.jet)
        self.ax.scatter(self.state[0][0], self.state[2][0], self.state[1][0], c='b', marker='x')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Z')
        self.ax.set_zlabel('Y')
        plt.axis('equal')
        plt.pause(0.05)
        #plt.show()
        """

        """
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "world"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "fusing"
        t.transform.translation.x = self.state[0][0]
        t.transform.translation.y = self.state[1][0]
        t.transform.translation.z = self.state[2][0]

        self.mutex_newValue.release()

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub.publish(tfm)
        """

        # Publish 2D position of the ekf state
        if (self.state_x_uv >= (0 + 10) and self.state_x_uv <= (640 - 10)) and (self.state_y_uv >= (0 + 10) and self.state_y_uv <= (480 - 10)):
          print("State: x = {0}, y = {1}".format(self.state_x_uv, self.state_y_uv))
          self.ekf_coordinates_msg.header.seq = self.header_seq
          self.ekf_coordinates_msg.header.stamp = rospy.Time.now()
          self.ekf_coordinates_msg.header.frame_id = str("ekf")
          self.ekf_coordinates_msg.point.x = self.state_x_uv
          self.ekf_coordinates_msg.point.y = self.state_y_uv
          self.pub.publish(self.ekf_coordinates_msg)

        # Check im image exists
        # Display image if it exists, the vision tracker position and the projection of the UWB
        """
        if self.cv_image is not None:
          self.mutex_image.acquire(1)
          self.mutex_uwb.acquire(1)
          #print("uwb: x = {0}, y = {1}, z = {2}".format(self.uwb_x_wc, self.uwb_y_wc, self.uwb_z_wc))
          cv2.circle(self.cv_image, (int(self.uwb_x_uv), int(self.uwb_y_uv)), 10, (0, 0, 255), -1)
          self.mutex_uwb.release()
          self.mutex_vision.acquire(1)
          #print("vision: x= {0}, y= {1}".format(self.vision_x_wc, self.vision_y_wc))
          cv2.circle(self.cv_image, (int(self.vision_x_uv), int(self.vision_y_uv)), 10, (255, 0, 0), -1)
          #print("Vision: x = {0}, y = {1}".format(self.vision_x_uv, self.vision_y_uv))
          self.mutex_vision.release()
          self.mutex_state.acquire(1)
          if self.object_detected==True:
            cv2.circle(self.cv_image, (int(self.state_x_uv), int(self.state_y_uv)), 10, (0, 255, 0), -1)
          else:
            cv2.circle(self.cv_image, (int(self.state_x_uv), int(self.state_y_uv)), 10, (0, 255, 255), -1)
          self.mutex_state.release()
          #print("State: x = {0}, y = {1}, z = {2}".format(self.state[0], self.state[1], self.state[2]))
          print("State: x = {0}, y = {1}".format(self.state_x_uv, self.state_y_uv))
          cv2.imshow("frame", self.cv_image)
          self.mutex_image.release()
          cv2.waitKey(1)
        """

    plt.show()
    plt.pause(0.01)


if __name__ == '__main__':
    try:
        fusing = Fusing()
        fusing.initROS()
        fusing.start()
    except rospy.ROSInterruptException:
        pass