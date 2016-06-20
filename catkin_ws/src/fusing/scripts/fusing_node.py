#!/usr/bin/env python2

# Imports
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
    self.x_uwb = 0.0
    self.y_uwb = 0.0
    self.z_uwb = 0.0
    # Velocities recived from the UWB
    self.vx_uwb = 0.0
    self.vy_uwb = 0.0
    self.vz_uwb = 0.0

    # Coordinates received from the vision tracker
    self.x_visionTracker = 0.0
    self.y_visionTracker = 0.0
    self.z_visionTracker = 0.0

    # Fused coordinates from the states of the EKF
    self.state = np.array([[1.0], [1.0], [1.0], [0.0], [0.0], [0.0]])

    # EKF
    self.newValue = self.NewValue()
    self.lastTimeStamp = rospy.Time.now()
    self.deltaT = 0.0035
    # Initial values
    self.vecXm = [0, 0]
    #self.matPm = np.zeros((6,6))
    self.matPm = 10*np.identity(6)
    self.matQ = np.vstack((np.zeros((3,6)), np.hstack((np.zeros((3,3)), 10*np.identity(3)))))
    self.matR1 = np.identity(6)
    self.matR2 = np.identity(2)
    # Constant matrices used by the EKF
    #self.matA = np.vstack((np.hstack((np.identity(3), self.deltaT * np.identity(3))), np.hstack((np.zeros((3,3)), np.identity(3)))))


    # Prototyping
    self.mutex_newValue = Lock()
    self.mutex_vision = Lock()
    self.mutex_uwb = Lock()
    self.mutex_image = Lock()
    self.x_uwb_list = collections.deque(maxlen=200)
    self.y_uwb_list = collections.deque(maxlen=200)
    self.z_uwb_list = collections.deque(maxlen=200)
    self.x_vision_list = collections.deque(maxlen=200)
    self.y_vision_list = collections.deque(maxlen=200)
    self.x_state_list = collections.deque(maxlen=200)
    self.y_state_list = collections.deque(maxlen=200)
    self.lv = 1
    self.realState = []
    self.allX = np.zeros(1)
    self.allY = np.zeros(1)
    self.allZ = np.zeros(1)

    self.bridge = CvBridge()

    #plt.ion()
    #plt.axis([-1.5, 1.5, -1.5, 1.5])
    #plt.axis('equal')
    fig = plt.figure()
    self.ax = fig.gca(projection='3d')
    #self.jet = plt.get_cmap('jet')

  ## Callback function to receive the UWB messages from ROS.
  def uwb_callback(self, data):
    #uwb_x = -data.state[1] + 0.0570
    #uwb_x = data.state[1] + 0.0570
    uwb_x = data.state[1] + 0.0595
    uwb_y = data.state[2] + 0.0388
    uwb_z = data.state[0] + 0.0359

    self.mutex_uwb.acquire(1)
    self.vx_uwb = -data.state[4]
    self.vy_uwb = data.state[5]
    self.vz_uwb = data.state[3]

    self.x_uwb = 0.9834*( 0.9956*uwb_x + 0.0861*uwb_y + 0.0380*uwb_z)
    self.y_uwb = 0.9834*(-0.0885*uwb_x + 0.9938*uwb_y + 0.0674*uwb_z)
    self.z_uwb = 0.9834*(-0.0320*uwb_x - 0.0704*uwb_y + 0.9970*uwb_z)

    self.x_uwb_transf = 593.16120354*self.x_uwb/self.z_uwb + 308.67164248
    self.y_uwb_transf = 589.605859*self.y_uwb/self.z_uwb + 245.3659398

    self.x_uwb_list.append(self.x_uwb)
    self.y_uwb_list.append(self.y_uwb)
    self.z_uwb_list.append(self.z_uwb)
    self.mutex_uwb.release()

    self.mutex_newValue.acquire(1)
    self.newValue.newValue = True
    self.newValue.newUWB = True
    self.mutex_newValue.release()

  ## Callback function to receive the vision tracker messages from ROS.
  def vision_tracker_callback(self, data):
    self.mutex_vision.acquire(1)
    self.x_visionTracker = data.point.x
    self.y_visionTracker = data.point.y
    #print("vision: x = {0}, y = {1}".format(data.point.x, data.point.y))

    #self.x_vision_transf = 593.16120354*self.x_visionTracker/1 + 308.67164248
    #self.y_vision_transf = 589.605859*self.y_visionTracker/1 + 245.3659398

    self.x_vision_list.append(self.x_visionTracker)
    self.y_vision_list.append(self.y_visionTracker)
    self.mutex_vision.release()

    self.mutex_newValue.acquire(1)
    self.newValue.newValue = True
    self.newValue.newVision = True
    self.mutex_newValue.release()

  ## Callback function to receive the image messages from ROS
  def image_callback(self, data):
    self.mutex_image.acquire(1)
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    self.mutex_image.release()

  ## Performs one iteration of the EKF.
  def ekf_iteration(self):
    # Step 1
    #vecState_p = np.dot(self.matA, self.vecX)
    # Get the time difference
    deltaT = rospy.Time.now() - self.lastTimeStamp
    self.lastTimeStamp = rospy.Time.now()

    # Compute the matrix A, the vector state_p and the matrix P_p
    matA = np.vstack((np.hstack((np.identity(3), deltaT.to_sec() * np.identity(3))), np.hstack((np.zeros((3,3)), np.identity(3)))))
    vecState_p = np.dot(matA, self.state)
    matP_p = np.dot(matA, np.dot(self.matPm, matA.transpose())) + self.matQ # A*Pm*A^T + Q

    # Step 2
    #matH = np.array([[np.identity(6)], [1/vecState_p[2], 0, -vecState_p[0]/vecState_p[3]**2, 0, 0, 0],\
    #                 [0, 1/vecState_p[2], -vecState_p[1]/vecState_p[3]**2, 0, 0, 0]])
    # Compute the matrix H, R and K
    matH_top = np.identity(6)
    matH_bottom = np.array([[1/vecState_p[2][0], 0, -vecState_p[0][0]/vecState_p[2][0]**2, 0, 0, 0], \
                     [0, 1/vecState_p[2][0], -vecState_p[1][0]/vecState_p[2][0]**2, 0, 0, 0]])
    matH = np.vstack((matH_top,matH_bottom))
    matK_t1 = np.dot(self.matQ, matH.transpose()) # Q * H^T
    matK_t2 = np.dot(matH, matK_t1) # H*Q*H^T
    matR = np.vstack((np.hstack((self.matR1, np.zeros((6,2)))), np.hstack((np.zeros((2,6)), self.matR2))))

    matK = np.dot(matK_t1, np.linalg.inv(matK_t2 + matR))

    vecStatem_t1 = np.array([[vecState_p[0][0]/vecState_p[2][0]], [vecState_p[1][0]/vecState_p[2][0]]])

    vecZ = np.zeros((8,1))
    # Checks wheter new UWB data and/or new vision data is available.
    if (self.newValue.newUWB==True and self.newValue.newVision==True):
      vecZ[0][0] = self.x_uwb
      vecZ[1][0] = self.y_uwb
      vecZ[2][0] = self.z_uwb
      vecZ[3][0] = self.vx_uwb
      vecZ[4][0] = self.vy_uwb
      vecZ[5][0] = self.vz_uwb
      vecZ[6][0] = self.x_visionTracker
      vecZ[7][0] = self.y_visionTracker
      vecStatem_t2 = vecZ - np.vstack(((vecState_p, vecStatem_t1))) # z - [H1*x; H2(x)]
    elif (self.newValue.newUWB==True and self.newValue.newVision==False):
      vecZ[0][0] = self.x_uwb
      vecZ[1][0] = self.y_uwb
      vecZ[2][0] = self.z_uwb
      vecZ[3][0] = self.vx_uwb
      vecZ[4][0] = self.vy_uwb
      vecZ[5][0] = self.vz_uwb
      vecStatem_t2 = vecZ - np.vstack(((vecState_p, np.zeros((2, 1))))) # z - [H1*x; H2(x)]
    elif (self.newValue.newUWB==False and self.newValue.newVision==True):
      vecZ[6][0] = self.x_visionTracker
      vecZ[7][0] = self.y_visionTracker
      vecStatem_t2 = vecZ - np.vstack(((np.zeros((6, 1)), vecStatem_t1))) # z - [H1*x; H2(x)]


    # Estimation of the new state
    vecStatem_t2 = vecZ - np.vstack(((vecState_p, vecStatem_t1))) # z - [H1*x; H2(x)]
    vecStatem_t3 = np.dot(matK, vecStatem_t2) # K*(z - [H1*x; H2(x)])
    self.state = vecState_p + vecStatem_t3
    self.x_state_list.append(self.state[0])
    self.y_state_list.append(self.state[1])

    # Estimation of the new coovariance matrix
    matPm_t = np.dot(matK, matH) # K*H
    self.matPm = np.dot(np.identity(6) - matPm_t, matP_p) # (I - K*H)*Pp

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

    self.x_uwb = vecZ[0][0]
    self.y_uwb = vecZ[1][0]
    self.z_uwb = vecZ[2][0]
    self.vx_uwb = vecZ[3][0]
    self.vy_uwb = vecZ[4][0]
    self.vz_uwb = vecZ[5][0]
    self.x_visionTracker = vecZ[6][0]
    self.y_visionTracker = vecZ[7][0]

    self.newValue.newValue = True
    self.newValue.newUWB = True
    self.newValue.newVision = True
    return(matQ, matR1, matR2, vecZ)

  ## Initialize ROS
  def initROS(self):
    self.sub_uwb = rospy.Subscriber('/uwb/tracker', uwb.msg.UWBTracker, self.uwb_callback)
    self.sub_vision = rospy.Subscriber('/vision_tracker/vision_coordinates', geometry_msgs.msg.PointStamped, self.vision_tracker_callback)
    self.sub_img = rospy.Subscriber('/vision_tracker/video', sensor_msgs.msg.Image, self.image_callback)

    self.pub = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)

  def start(self):
    #tfBuffer = tf2_ros.Buffer()
    #list1 = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
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

        """
        # Display of state trajectory
        self.allX = np.append(self.allX, self.state[0][0])
        self.allY = np.append(self.allY, self.state[1][0])
        self.allZ = np.append(self.allZ, self.state[2][0])
        self.ax.plot(self.allX, -self.allZ, self.allY, label='trajectory')
        plt.pause(0.05)
        """

        # Display of vision and the projection of uwb
        """
        plt.clf()
        plt.ion()
        plt.axis([-1.5, 1.5, -1.5, 1.5])
        self.mutex_vision.acquire(1)
        vision_plt = plt.scatter(self.x_vision_list, self.y_vision_list, c='b', marker='o')
        #print("x vision mean: ",format(np.mean(self.x_vision_list)))
        #print("y vision mean: ",format(np.mean(self.y_vision_list)))
        self.mutex_vision.release()
        self.mutex_uwb.acquire(1)
        #uwbPlt = plt.scatter(self.x_uwb_list, self.y_uwb_list, c='r', marker='o')
        uwb_x = np.array(self.x_uwb_list)
        uwb_y = np.array(self.y_uwb_list)
        uwb_z = np.array(self.z_uwb_list)
        print("x div: ",format(abs(self.x_uwb_list[-1] - self.x_vision_list[-1])))
        print("y div: ",format(abs(self.y_uwb_list[-1] - self.y_vision_list[-1])))
        uwb_plt = plt.scatter(self.x_uwb_list, self.y_uwb_list, c='r', marker='o')
        self.mutex_uwb.release()
        ekf_plt = plt.scatter(self.x_state_list, self.y_state_list, c='g', marker='x')
        #plt.legend(handles=[vision_plt, uwb_plt, ekf_plt])#, aruco_transf_plt, uwb_plt])
        plt.pause(0.05)  # Display of vision and the projection of uwb
        """

        # Display of state trajectory
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

        # Check im image exists
        try:
          self.cv_image
        except NameError:
          x_exists = False
        else:
          x_exists = True

        # Display image if it exists
        if x_exists == True:
          if self.cv_image is not None:
            self.mutex_image.acquire(1)
            self.mutex_uwb.acquire(1)
            print("uwb: x= {0}, y= {1}".format(int(self.x_uwb_transf) + 120, int(self.y_uwb_transf) - 100))
            cv2.circle(self.cv_image, (int(self.x_uwb_transf) + 120, int(self.y_uwb_transf) - 100), 10, (0, 0, 255), -1)
            self.mutex_uwb.release()
            self.mutex_vision.acquire(1)
            print("vision: x= {0}, y= {1}".format(int(self.x_visionTracker), int(self.y_visionTracker)))
            cv2.circle(self.cv_image, (int(self.x_visionTracker), int(self.y_visionTracker)), 10, (255, 0, 0), -1)
            self.mutex_vision.release()
            cv2.imshow("frame", self.cv_image)
            self.mutex_image.release()
            cv2.waitKey(1)

if __name__ == '__main__':
    try:
        fusing = Fusing()
        fusing.initROS()
        fusing.start()
    except rospy.ROSInterruptException:
        pass