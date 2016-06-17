#!/usr/bin/env python2

import numpy as np
import random
import matplotlib.pyplot as plt
import rospy
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from uwb.msg import UWBTracker
#from tf2_msgs import TFMessage

class Fusing:

  class NewValue:
    newValue = False
    newUWB = False
    newVision = False

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
    # Constant matrices used by the EKF
    #self.matA = np.vstack((np.hstack((np.identity(3), self.deltaT * np.identity(3))), np.hstack((np.zeros((3,3)), np.identity(3)))))


    # Prototyping
    self.lv = 1
    self.realState = []
    plt.ion()
    self.jet = plt.get_cmap('jet')

  def uwb_callback(self, data):
    self.x_uwb = data.state[0]
    self.y_uwb = data.state[1]
    self.z_uwb = data.state[2]
    self.vx_uwb = data.state[3]
    self.vy_uwb = data.state[4]
    self.vz_uwb = data.state[5]

    self.newValue.newValue = True
    self.newValue.newUWB = True

  def ekf_iteration(self, matQ, matR1, matR2, vecZ):
    # Step 1
    #vecState_p = np.dot(self.matA, self.vecX)
    deltaT = rospy.Time.now() - self.lastTimeStamp
    self.lastTimeStamp = rospy.Time.now()
    matA = np.vstack((np.hstack((np.identity(3), deltaT.to_sec() * np.identity(3))), np.hstack((np.zeros((3,3)), np.identity(3)))))
    vecState_p = np.dot(matA, self.state)
    matP_p = np.dot(matA, np.dot(self.matPm, matA.transpose())) + matQ # A*Pm*A^T + Q

    # Step 2
    #matH = np.array([[np.identity(6)], [1/vecState_p[2], 0, -vecState_p[0]/vecState_p[3]**2, 0, 0, 0],\
    #                 [0, 1/vecState_p[2], -vecState_p[1]/vecState_p[3]**2, 0, 0, 0]])
    matH_top = np.identity(6)
    matH_bottom = np.array([[1/vecState_p[2][0], 0, -vecState_p[0][0]/vecState_p[2][0]**2, 0, 0, 0], \
                     [0, 1/vecState_p[2][0], -vecState_p[1][0]/vecState_p[2][0]**2, 0, 0, 0]])
    matH = np.vstack((matH_top,matH_bottom))
    matK_t1 = np.dot(matQ, matH.transpose()) # Q * H^T
    matK_t2 = np.dot(matH, matK_t1) # H*Q*H^T
    matR = np.vstack((np.hstack((matR1, np.zeros((6,2)))), np.hstack((np.zeros((2,6)), matR2))))

    matK = np.dot(matK_t1, np.linalg.inv(matK_t2 + matR))

    vecStatem_t1 = np.array([[vecState_p[0][0]/vecState_p[2][0]], [vecState_p[1][0]/vecState_p[2][0]]])

    vecZ = np.zeros((8,1))
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
    elif(self.newValue.newUWB==False and self.newValue.newVision==True)
      vecZ[6][0] = self.x_visionTracker
      vecZ[7][0] = self.y_visionTracker
      vecStatem_t2 = vecZ - np.vstack(((np.zeros((6, 1)), vecStatem_t1))) # z - [H1*x; H2(x)]


    vecStatem_t2 = vecZ - np.vstack(((vecState_p, vecStatem_t1))) # z - [H1*x; H2(x)]
    vecStatem_t3 = np.dot(matK, vecStatem_t2) # K*(z - [H1*x; H2(x)])
    self.state = vecState_p + vecStatem_t3

    matPm_t = np.dot(matK, matH) # K*H
    self.matPm = np.dot(np.identity(6) - matPm_t, matP_p) # (I - K*H)*Pp

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

  def fuse(self):
    tfBuffer = tf2_ros.Buffer()
    list1 = tf2_ros.TransformListener(tfBuffer)
    sub = rospy.Subscriber('UWB_Tracker', UWBTracker, self.uwb_callback)

    pub = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)

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

      [matQ, matR1, matR2, vecZ] = self.simulateInput()

      if self.newValue.newValue==True:
        vecZ = np.zeros((8,1))
        """
        if self.newValue.newUWB==True:
          vecZ[0][0] = self.x_uwb
          vecZ[1][0] = self.y_uwb
          vecZ[2][0] = self.z_uwb
          vecZ[3][0] = self.vx_uwb
          vecZ[4][0] = self.vy_uwb
          vecZ[5][0] = self.vz_uwb
        if self.newValue.newVision==True:
          vecZ[6][0] = self.x_visionTracker
          vecZ[7][0] = self.y_visionTracker
        """

        self.ekf_iteration(matQ, matR1, matR2, vecZ)

      #self.allX.append(self.state[0][0])
      #self.allY.append(self.state[1][0])

      #self.hl.set_xdata(np.append(self.hl.get_xdata(), self.state[0][0]))
      #self.hl.set_ydata(np.append(self.hl.get_ydata(), self.state[1][0]))
      plt.scatter(self.realState[-1][0][0], self.realState[-1][1][0], s=100, c="b", cmap=self.jet)
      plt.scatter(self.state[0][0], self.state[1][0], s=100, c="r", cmap=self.jet)
      plt.pause(0.05)


      t = geometry_msgs.msg.TransformStamped()
      t.header.frame_id = "world"
      t.header.stamp = rospy.Time.now()
      t.child_frame_id = "fusing"
      t.transform.translation.x = 0.0
      t.transform.translation.y = 0.0
      t.transform.translation.z = 0.0

      t.transform.rotation.x = 0.0
      t.transform.rotation.y = 0.0
      t.transform.rotation.z = 0.0
      t.transform.rotation.w = 0.0

      tfm = tf2_msgs.msg.TFMessage([t])
      pub.publish(tfm)


if __name__ == '__main__':
    try:
        fusing = Fusing()
        fusing.fuse()
    except rospy.ROSInterruptException:
        pass