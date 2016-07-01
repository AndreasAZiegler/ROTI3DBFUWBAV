#!/usr/bin/env python2

# Imports
import rospy
import numpy as np
from threading import Lock
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import uwb.msg
from cv_bridge import CvBridge, CvBridgeError
import cv2

object_detected = True
bridge = CvBridge()
newValue = False

vision_x_uv = 0
vision_y_uv = 0
uwb_x_uv = 0
uwb_y_uv = 0
state_x_uv = 0
state_y_uv = 0

cv_image = np.zeros(1)

matCovarianceRotation = np.array([[ 0.1203, -0.9910,  0.0595,       0,       0,       0],\
                                  [-0.0356,  0.0642, -0.9973,       0,       0,       0],\
                                  [ 0.9921,  0.1178,  0.0758,       0,       0,       0],\
                                  [      0,       0,       0,  0.1203, -0.9910,  0.0595],\
                                  [      0,       0,       0, -0.0356,  0.0642, -0.9973],\
                                  [      0,       0,       0,  0.9921,  0.1178,  0.0758]])
covRadiusUWB = 0
matR1 = np.identity(6)

covRadiusEKF = 0

mutex_image = Lock()
mutex_vision_detected = Lock()
mutex_vision = Lock()
mutex_uwb = Lock()
mutex_newValue = Lock()
mutex_state = Lock()
mutex_covar_ekf = Lock()

## Callback function to receive the UWB messages from ROS.
def uwb_callback(data):
  global mutex_uwb
  global mutex_newValue
  global matCovarianceRotation
  global covRadiusUWB
  global matR1
  global newValue
  global uwb_x_uv
  global uwb_y_uv

  # uwb_30: video_uwb_30: lrms=0.0646
  uwb_x = data.state[0] - 0.0297
  uwb_y = data.state[1] + 0.0083
  uwb_z = data.state[2] - 0.0568

  mutex_uwb.acquire(1)
  matR1_t1 = np.array([[data.covariance[0], data.covariance[1], data.covariance[2], \
                        data.covariance[3], data.covariance[4], data.covariance[5]], \
                       [data.covariance[6], data.covariance[7], data.covariance[8], \
                        data.covariance[9], data.covariance[10], data.covariance[11]], \
                       [data.covariance[12], data.covariance[13], data.covariance[14], \
                        data.covariance[15], data.covariance[16], data.covariance[17]], \
                       [data.covariance[18], data.covariance[19], data.covariance[20], \
                        data.covariance[21], data.covariance[22], data.covariance[23]], \
                       [data.covariance[24], data.covariance[25], data.covariance[26], \
                        data.covariance[27], data.covariance[28], data.covariance[29]], \
                       [data.covariance[30], data.covariance[31], data.covariance[32], \
                        data.covariance[33], data.covariance[34], data.covariance[35]]])

  matR1_t2 = np.dot(matR1_t1, matCovarianceRotation.transpose())
  matR1 = 1.0340**2 * np.dot(matCovarianceRotation, matR1_t2)

  covRadiusUWB = 25**4*(matR1[0, 0] ** 2 + matR1[1, 1] ** 2 + matR1[2, 2] ** 2)

  # uwb_30: video_uwb_30: lrms=0.0646
  uwb_x_wc = 1.0340*( 0.1203*uwb_x - 0.9910*uwb_y + 0.0595*uwb_z)
  uwb_y_wc = 1.0340*(-0.0356*uwb_x - 0.0642*uwb_y - 0.9973*uwb_z)
  uwb_z_wc = 1.0340*( 0.9921*uwb_x + 0.1178*uwb_y - 0.0758*uwb_z)

  uwb_x_uv = 593.16120354*uwb_x_wc/uwb_z_wc + 308.67164248
  uwb_y_uv = 589.605859*uwb_y_wc/uwb_z_wc + 245.3659398

  mutex_uwb.release()

  mutex_newValue.acquire(1)
  newValue = True
  mutex_newValue.release()

## Callback function to receive the vision tracker coordinates messages from ROS.
def vision_tracker_coordinates_callback(data):
  global mutex_vision
  global mutex_newValue
  global vision_x_uv
  global vision_y_uv
  global newValue

  mutex_vision.acquire(1)
  vision_x_uv = data.point.x
  vision_y_uv = data.point.y
  mutex_vision.release()

  mutex_newValue.acquire(1)
  newValue = True
  mutex_newValue.release()

## Callback function to receive the vision tracker object detected messages from ROS.
def vision_tracker_object_detected_callback(data):
  global mutex_vision_detected
  global mutex_vision
  global object_detected

  mutex_vision_detected.acquire(1)
  object_detected = data.data
  mutex_vision.acquire(1)

  if object_detected==True:
    matR2 = 10**(-7)*np.identity(2)
  else:
    matR2 = np.identity(2)

  mutex_vision.release()
  mutex_vision_detected.release()

## Callback function to receive the image messages from ROS
def image_callback(data):
  global mutex_image
  global bridge
  global cv_image
  global newValue

  mutex_image.acquire(1)
  # For uncompressed images
  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)
  """
  # For compressed images
  np_arr = np.fromstring(data.data, np.uint8)
  self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
  """
  mutex_image.release()

  mutex_newValue.acquire(1)
  newValue = True
  mutex_newValue.release()

## Callback function to receive the fused coordinate points from the EKF
def ekf_coordinates_callback(data):
  global state_x_uv
  global state_y_uv
  global mutex_state
  global newValue

  mutex_state.acquire(1)
  state_x_uv = data.point.x
  state_y_uv = data.point.y
  mutex_state.release()

  mutex_newValue.acquire(1)
  newValue = True
  mutex_newValue.release()

## Callback function to receive the covariances from the EKF
def ekf_covariances_callback(data):
  global covRadiusEKF
  global mutex_covar_ekf

  mutex_covar_ekf.acquire(1)
  covRadiusEKF = 25**4*(data.data[0]**2 + data.data[1]**2 + data.data[2]**2)
  mutex_covar_ekf.release()

## Initialize ROS
def initROS():
  # ROS init
  rospy.init_node('visualization', anonymous=True)

  sub_uwb = rospy.Subscriber('/uwb/tracker', uwb.msg.UWBTracker, uwb_callback)
  sub_vision_coordinates = rospy.Subscriber('/vision_tracker/vision_coordinates', geometry_msgs.msg.PointStamped, vision_tracker_coordinates_callback)
  sub_vision_object_detected = rospy.Subscriber('/vision_tracker/object_detected', std_msgs.msg.Bool, vision_tracker_object_detected_callback)
  sub_img = rospy.Subscriber('/camera/video', sensor_msgs.msg.Image, image_callback)
  sub_ekf_coordinates = rospy.Subscriber('/fusing/ekf_uv_coordinates', geometry_msgs.msg.PointStamped, ekf_coordinates_callback)
  sub_ekf_covariances = rospy.Subscriber('/fusing/ekf_covariance', std_msgs.msg.Float64MultiArray, ekf_covariances_callback)

def start():
  global mutex_image
  global mutex_vision
  global mutex_vision_detected
  global mutex_uwb
  global mutex_state
  global mutex_newValue
  global cv_image
  global uwb_x_uv
  global uwb_y_uv
  global state_x_uv
  global state_y_uv
  global newValue
  global covRadiusUWB

  while not rospy.is_shutdown():

    rospy.sleep(0.1)


    mutex_newValue.acquire(1)
    if newValue==True:
      # Check im image exists
      # Display image if it exists, the vision tracker position and the projection of the UWB
      if cv_image is not None:
        mutex_image.acquire(1)
        mutex_uwb.acquire(1)
        #print("uwb: x = {0}, y = {1}, z = {2}".format(self.uwb_x_wc, self.uwb_y_wc, self.uwb_z_wc))
        print("covRadiusUWB = {0}".format(covRadiusUWB))
        cv2.circle(cv_image, (int(uwb_x_uv), int(uwb_y_uv)), int(covRadiusUWB), (0, 0, 255), 5)
        mutex_uwb.release()
        mutex_vision.acquire(1)
        #print("vision: x= {0}, y= {1}".format(self.vision_x_wc, self.vision_y_wc))
        if object_detected==True:
          cv2.circle(cv_image, (int(vision_x_uv), int(vision_y_uv)), 10, (255, 0, 0), 5)
          cv2.putText(cv_image, "Vision", (20, 70), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (255, 0, 0))
        else:
          cv2.circle(cv_image, (int(vision_x_uv), int(vision_y_uv)), 10, (255, 255, 0), 5)
          #cv2.putText(cv_image, "Vision", (20, 70), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (255, 255, 0))
        #print("Vision: x = {0}, y = {1}".format(self.vision_x_uv, self.vision_y_uv))
        mutex_vision.release()
        mutex_state.acquire(1)
        mutex_covar_ekf.acquire(1)
        cv2.circle(cv_image, (int(state_x_uv), int(state_y_uv)), int(covRadiusEKF), (0, 200, 0), 5)
        print("covRadiusEKF = {0}".format(covRadiusEKF))
        mutex_covar_ekf.release()
        mutex_state.release()
        #print("State: x = {0}, y = {1}, z = {2}".format(self.state[0], self.state[1], self.state[2]))
        #print("State: x = {0}, y = {1}".format(self.state_x_uv, self.state_y_uv))
        cv2.putText(cv_image, "UWB", (20, 40), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0, 0, 255))
        cv2.putText(cv_image, "EKF", (20, 100), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0, 207, 0))
        cv2.imwrite("./output.jpg", cv_image)
        cv2.imshow("frame", cv_image)
        mutex_image.release()
        cv2.waitKey(1)

        newValue=False

    mutex_newValue.release()


if __name__ == '__main__':
  try:
    initROS()
    start()
  except rospy.ROSInterruptException:
    pass
