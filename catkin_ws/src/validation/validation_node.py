#!/usr/bin/env python2

# Imports
import os
import sys
import select
import datetime
import h5py
import rospy
import uwb.msg
import geometry_msgs.msg
import threading
import numpy as np
import matplotlib.pyplot as plt

#vicon_list = np.empty((4,1))
vicon_list = []

#uwb_list = np.empty((4,1))
uwb_list = []

#ekf_list = np.empty((4,1))
ekf_list = []

vicon_mutex = threading.Lock()
uwb_mutex = threading.Lock()
ekf_mutex = threading.Lock()


def timedelta():
  #delta =  time.mktime((datetime.datetime.now().timetuple())) - time.mktime((start.timetuple()))
  delta =  (datetime.datetime.now() - start_time).total_seconds() * 1000.0
  return delta

def vicon_callback(data):
  global vicon_list
  global vicon_mutex

  """
  time = rospy.Time.now()
  vicon_x = data.transform.translation.x
  vicon_y = data.transform.translation.y
  vicon_z = data.transform.translation.z

  # vicon_30: video_uwb_30: lrms=0.0336
  vicon_x = vicon_x - 1.4680
  vicon_y = vicon_y + 1.2642
  vicon_z = vicon_z - 1.2334

  # vicon_30: video_uwb_30: lrms=0.0336
  vicon_transf_x = 1.0639*(-0.0349*vicon_x + 0.9993*vicon_y - 0.0109*vicon_z)
  vicon_transf_y = 1.0639*( 0.0581*vicon_x - 0.0089*vicon_y - 0.9983*vicon_z)
  vicon_transf_z = 1.0639*(-0.9977*vicon_x - 0.0355*vicon_y - 0.0587*vicon_z)
  """

  #vicon_array = np.array([[data.header.stamp.secs], [data.transform.translation.x], [data.transform.translation.y], [data.transform.translation.z]])

  vicon_mutex.acquire(1)
  """
  tmp = vicon_list
  vicon_list = np.zeros((tmp.shape[0], tmp.shape[1]+1))
  vicon_list[:,:-1] = tmp
  vicon_list[:,-1] = vicon_array.transpose()
  """
  #vicon_list.append([data.header.stamp.secs, data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
  #vicon_list.append([rospy.Time.now().to_nsec(), data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
  vicon_list.append([timedelta(), data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
  vicon_mutex.release()

def uwb_callback(data):
  global uwb_list
  global uwb_mutex

  """
  # uwb_30: video_uwb_30: lrms=0.0632
  uwb_x = data.state[0] - 0.0239
  uwb_y = data.state[1] + 0.0119
  uwb_z = data.state[2] - 0.0563

  uwb_x_wc = 1.0281*( 0.1233*uwb_x - 0.9904*uwb_y + 0.0629*uwb_z)
  uwb_y_wc = 1.0281*(-0.0348*uwb_x - 0.0677*uwb_y - 0.9971*uwb_z)
  uwb_z_wc = 1.0281*( 0.9918*uwb_x + 0.1207*uwb_y - 0.0428*uwb_z)
  """

  #uwb_array = np.array([[data.header.stamp.secs], [uwb_x_wc], [uwb_y_wc], [uwb_z_wc]])

  uwb_mutex.acquire(1)
  """
  tmp = uwb_list
  uwb_list = np.zeros((tmp.shape[0], tmp.shape[1]+1))
  uwb_list[:,:-1] = tmp
  uwb_list[:,-1] = uwb_array.transpose()
  """
  #uwb_list.append([data.header.stamp.secs, data.state[0], data.state[1], data.state[2]])
  #uwb_list.append([rospy.Time.now().to_nsec(), data.state[0], data.state[1], data.state[2]])
  uwb_list.append([timedelta(), data.state[0], data.state[1], data.state[2]])
  uwb_mutex.release()

def ekf_callback(data):
  global ekf_list
  global ekf_mutex

  #ekf_array = np.array([[data.header.stamp.secs], [data.point.x], [data.point.y], [data.point.z]])

  ekf_mutex.acquire(1)
  """
  tmp = ekf_list
  ekf_list = np.zeros((tmp.shape[0], tmp.shape[1]+1))
  ekf_list[:,:-1] = tmp
  ekf_list[:,-1] = ekf_array.transpose()
  """
  #ekf_list.append([data.header.stamp.secs, data.point.x, data.point.y, data.point.z])
  #ekf_list.append([rospy.Time.now().to_nsec(), data.point.x, data.point.y, data.point.z])
  ekf_list.append([timedelta(), data.point.x, data.point.y, data.point.z])
  ekf_mutex.release()


def initROS():
  rospy.init_node('validation', anonymous=True)

  sub_vicon = rospy.Subscriber('/vicon/UWB_Box/UWB_Box', geometry_msgs.msg.TransformStamped, vicon_callback)
  sub_uwb = rospy.Subscriber('/uwb/tracker', uwb.msg.UWBTracker, uwb_callback)
  sub_ekf = rospy.Subscriber('/fusing/ekf_wc_coordinates', geometry_msgs.msg.PointStamped, ekf_callback)

def start():
  global vicon_list
  global uwb_list
  global ekf_list

  while not rospy.is_shutdown():
    os.system('cls' if os.name == 'nt' else 'clear')
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
      line = raw_input()
      rospy.signal_shutdown("terminate")

  # Open file and write header
  file = h5py.File("/data/SP1/catkin_ws/src/validation/validation_data.hdf5", "w")

  # Wirte date sets to file
  dset = file.create_dataset("vicon", data = vicon_list, compression = "gzip", compression_opts = 9)
  dset = file.create_dataset("uwb", data = uwb_list, compression = "gzip", compression_opts = 9)
  dset = file.create_dataset("ekf", data = ekf_list, compression = "gzip", compression_opts = 9)
  file.flush()
  file.close()

if __name__ == '__main__':
  try:
    initROS()

    # Measure start time
    start_time = datetime.datetime.now()

    start()
  except rospy.ROSInterruptException:
    pass
