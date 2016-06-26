#!/usr/bin/env python2

# Imports
import os
import sys
import select
import rospy
import uwb.msg
import geometry_msgs.msg
import threading
import numpy as np
import matplotlib.pyplot as plt

vicon_list = np.empty((4,1))

uwb_list = np.empty((4,1))

ekf_list = np.empty((4,1))

vicon_mutex = threading.Lock()
uwb_mutex = threading.Lock()
ekf_mutex = threading.Lock()

def vicon_callback(self, data):
  global vicon_list
  global vicon_mutex

  vicon_array = np.array([[data.transforms.header.stamp], [data.transforms.transform.translation.x], [data.transforms.transform.translation.y], [data.transforms.transform.translation.z]])

  vicon_mutex.acquire(1)
  tmp = vicon_list
  vicon_list = np.zeros((tmp.shape[0], tmp.shape[1]+1))
  vicon_list[:,:-1] = tmp
  vicon_list[:,-1] = vicon_array.transpose()
  vicon_mutex.release()

def uwb_callback(self, data):
  global uwb_list
  global uwb_mutex

  # uwb_11: video_uwb_11: lrms=0.0703
  uwb_x = data.state[0] - 0.0327
  uwb_y = data.state[1] - 0.0143
  uwb_z = data.state[2] + 0.0038

  uwb_vx = data.state[3]
  uwb_vy = data.state[4]
  uwb_vz = data.state[5]

  uwb_x_wc = 1.0339*( 0.1227*uwb_x - 0.9923*uwb_y + 0.0193*uwb_z)
  uwb_y_wc = 1.0339*(-0.0729*uwb_x - 0.0284*uwb_y - 0.9969*uwb_z)
  uwb_z_wc = 1.0339*( 0.9898*uwb_x + 0.1209*uwb_y - 0.0758*uwb_z)

  uwb_array = np.array([[data.header.stamp], [uwb_x_wc], [uwb_y_wc], [uwb_z_wc]])

  uwb_mutex.acquire(1)
  tmp = uwb_list
  vicon_list = np.zeros((tmp.shape[0], tmp.shape[1]+1))
  vicon_list[:,:-1] = tmp
  vicon_list[:,-1] = uwb_array.transpose()
  uwb_mutex.release()

def ekf_callback(self, data):
  global ekf_list
  global ekf_mutex

  ekf_array = np.array([[data.header.stamp], [data.point.x], [data.point.y], [data.point.z]])

  ekf_mutex.acquire(1)
  tmp = ekf_list
  ekf_list = np.zeros((tmp.shape[0], tmp.shape[1]+1))
  ekf_list[:,:-1] = tmp
  ekf_list[:,-1] = ekf_array.transpose()
  ekf_mutex.release()


def initROS(self):
  rospy.init_node('validation', anonymous=True)

  sub_vicon = rospy.Subscriber('/vicon/tf', )
  sub_uwb = rospy.Subscriber('/uwb/tracker', uwb.msg.UWBTracker, uwb_callback)
  sub_ekf = rospy.Subscriber('/fusing/ekf_coordinates', geometry_msgs.msg.PointStamped, ekf_callback)

def start(self):
  while not rospy.is_shutdown():
    os.system('cls' if os.name == 'nt' else 'clear')
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
      line = raw_input()
      rospy.signal_shutdown("terminate")

  vicon = []
  uwb = []
  pos = 0
  offset = 2

  for i in range(0, len(vicon_list)):
    for j in range(offset, len(uwb_list)):
      # value where uwb time > vicon time
      if vicon_list[0,i] < uwb_list[0,j]:
        # check oth one before
        if abs(vicon_list[0,i] - uwb_list[0,j-1] < 5):
          vicon[:,pos] = vicon_list[:,i]
          uwb[:,pos] = uwb_list[:,j-1]
          pos = pos + 1
          offset = j
          continue

  rmse_uwb = []
  for i in range(0, len(vicon[0,:])):
    rmse_uwb[i] = np.sqrt((vicon[1,i] - uwb[1,i])**2 + (vicon[2,i] - uwb[2,i])**2 + (vicon[3,i] - uwb[3,i])**2)

  plt.plot(vicon[0,:], rmse_uwb)
  print("UWB rmse = {0}".format(sum(rmse_uwb)))


  vicon = []
  ekf = []
  pos = 0
  offset = 2

  for i in range(0, len(vicon_list)):
    for j in range(offset, len(ekf_list)):
      # value where ekf time > vicon time
      if vicon_list[0,i] < ekf_list[0,j]:
        # check oth one before
        if abs(vicon_list[0,i] - ekf_list[0,j-1] < 5):
          vicon[:,pos] = vicon_list[:,i]
          ekf[:,pos] = ekf_list[:,j-1]
          pos = pos + 1
          offset = j
          continue

  rmse_ekf = []
  for i in range(0, len(vicon[0,:])):
    rmse_ekf[i] = np.sqrt((vicon[1,i] - ekf[1,i])**2 + (vicon[2,i] - ekf[2,i])**2 + (vicon[3,i] - ekf[3,i])**2)

  plt.plot(vicon[0,:], rmse_ekf)
  print("EKF rmse = {0}".format(sum(rmse_ekf)))


if __name__ == '__main__':
  try:
    initROS()
    start()
  except rospy.ROSInterruptException:
    pass
