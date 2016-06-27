#!/usr/bin/env python2
'''
Created on 20.06.2016

@author: Andreas Ziegler
'''
import cv2
import numpy as np
import aruco
import rospy
import datetime
import sensor_msgs.msg
import geometry_msgs.msg
from uwb.msg import UWBTracker
import threading
import h5py

mutex_image = threading.Lock()
mutex_vicon = threading.Lock()


camparam = aruco.CameraParameters()
camparam.readFromXMLFile("../config/camera.yml")
intrinsics = camparam.CameraMatrix

R = np.array([[ 0.03674776, 0.99930489, -0.00627444], [ 0.99402457, -0.03590636, 0.10308167], [ 0.10278472, -0.01002497, -0.99465311]])
t = np.array([[ 0.02915348], [-0.05801044], [ 0.66861612]])

def timedelta():
    #delta =  time.mktime((datetime.datetime.now().timetuple())) - time.mktime((start.timetuple()))
    delta =  (datetime.datetime.now() - start).total_seconds() * 1000.0
    return delta

def writeToFile(string):
    # Thread block at this line until it can obtain lock
    lock.acquire()

    file.write(string)

def image_callback(data):
  global cv_image
  global mutex_image
  np_arr = np.fromstring(data.data, np.uint8)
  mutex_image.acquire(1)
  cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
  mutex_image.release()

def vicon_callback(data):
  global cv_image
  global mutex_image
  global x_vicon_transf
  global y_vicon_transf
  global x_vicon
  global y_vicon
  global z_vicon
  global mutex_vicon
  global intrinsics
  global t
  global R

   # Write world coordinates with time stamp to file
  viconCoordinates.append([timedelta(), data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])

  """
  vicon_x_tmp = data.state[0] + 0.0907
  vicon_y_tmp = data.state[1] - 0.0377
  vicon_z_tmp = data.state[2] - 0.0185

  x_vicon = 1.0136*( 0.3184*vicon_x_tmp - 0.9388*vicon_y_tmp - 0.0964*vicon_z_tmp)
  y_vicon = 1.0136*( 0.0785*vicon_x_tmp + 0.1255*vicon_y_tmp - 0.9707*vicon_z_tmp)
  z_vicon = 1.0136*( 0.9447*vicon_x_tmp + 0.1222*vicon_y_tmp + 0.2203*vicon_z_tmp)
  """

  """
  mutex_vicon.acquire(1)
  #uwb = np.matmul(R, np.array([[x_vicon], [y_vicon], [z_vicon]]))
  #uwb = np.matmul(intrinsics, uwb)
  uwb = np.matmul(intrinsics, np.array([[x_vicon], [y_vicon], [z_vicon]]))
  uwb = uwb + t
  x_vicon_transf = uwb[0]
  y_vicon_transf = uwb[1]
  x_vicon_transf = 593.16120354*x_vicon/z_vicon + 308.67164248
  y_vicon_transf = 589.605859*y_vicon/z_vicon + 245.3659398
  #x_vicon_transf = 593.16120354*x_vicon + 308.67164248*z_vicon + t[0]
  #y_vicon_transf = 589.605859*y_vicon + 245.3659398*z_vicon + t[1]
  mutex_vicon.release()
  """

def rosInit():
  rospy.init_node('uwb_aruco', anonymous=True)
  rospy.Subscriber('/camera/video/compressed', sensor_msgs.msg.CompressedImage, image_callback)
  rospy.Subscriber('/vicon/UWB_Box/UWB_Box', geometry_msgs.msg.TransformStamped, vicon_callback)

def getCoordinatesFromMarker():
    global camparam
    global cv_image
    global mutex_image
    global x_vicon_transf
    global y_vicon_transf
    global x_vicon
    global y_vicon
    global z_vicon
    global mutex_vicon
    # load board and camera parameters
    camparam = aruco.CameraParameters()
    #camparam.readFromXMLFile("/data/SP1/catkin_ws/src/record_worldcoordinates/camera.yml")
    camparam.readFromXMLFile("../config/camera.yml")
    #camparam.readFromXMLFile("../config/logitech.yml")

    # create detector and set parameters
    markerdetector = aruco.MarkerDetector()
    # set minimum marker size for detection
    #markerdetector.setMinMaxSize(0.01)

    # load video
    #cap = cv2.VideoCapture("example.mp4")
    """
    cap = cv2.VideoCapture(0)
    [ret, frame] = cap.read()

    if not ret:
        print("can't open video!")
        exit(-1)
    """

    #frame = cv2.imread("./2016-04-15-062737.jpg")

    while not rospy.is_shutdown():
        # Detect marker
        mutex_image.acquire(1)
        try:
          markers = markerdetector.detect(cv_image, camparam, 0.142)
        except NameError:
          print("continue")
          mutex_image.release()
          continue

        mutex_image.release()

        #if cv_image is not None:
          #markers = markerdetector.detect(frame, camparam, 400)

        #print(str(len(markers)) + " markers detected")

        for m in markers:
          if m.isValid():
            # get board and draw it
            #board = detector.getDetectedBoard()
            #board.draw(frame, np.array([255, 255, 255]), 2)

            # Commpose 4x4 transform matrix
            """"
            rotation_matrix = np.empty([3, 3])
            transform_matrix = np.empty([4, 4])
            cv2.Rodrigues(board.Rvec, rotation_matrix)
            transform_matrix = rotation_matrix.copy()
            transform_matrix.resize([4, 4])
            transform_matrix[3, 0] = 0
            transform_matrix[3, 1] = 0
            transform_matrix[3, 2] = 0
            transform_matrix[3, 3] = 1
            """

            """
            print("detected ids: ", ", ".join(str(m.id) for m in board) + '\n')
            print("Marker coordinates: (" + str(len(markers)) + " markers)", "\n".join(str(m.getCenter()) for m in markers) + '\n')
            """

            #print("Marker id: " + str(markers[i].id) + " Coordinates: " + str(markers[i].getCenter()))
            #print("Marker Coordinates: " + str(m.getCenter()) + " Tvec: " + str(m.Tvec))

            # Get rvec, tvec, pixel coordinates
            rvec = m.Rvec
            tvec = m.Tvec
            coordinates = m.getCenter()
            uvPoint = np.ones([3, 1])
            uvPoint[0] = coordinates[0]
            uvPoint[1] = coordinates[1]

            # Get rotations matrix and camera intrinsics matrix
            [rotationsMatrix, jacobian] = cv2.Rodrigues(rvec)
            #intrinsics = np.array([camparam.CameraMatrix[0], camparam.CameraMatrix[1], camparam.CameraMatrix[2]])
            intrinsics = camparam.CameraMatrix

            #print("R = {0}".format(rotationsMatrix))
            #print("t = {0}".format(tvec))
            #print("u = {0}, v = {1}". format(uvPoint[0], uvPoint[1]))

            """
            tvec_proj = np.array([tvec[0] / tvec[2], tvec[1] / tvec[2], 1])
            uv_proj = np.dot(intrinsics, tvec_proj)
            print("uv_proj = {}".format(uv_proj))
            #print("intrinsics = {0}".format(intrinsics))
            """

            """
            zConst = 1
            s = 0
            #tempMat = rotationsMatrix.inv() * intrinsics.inv() * coordinates
            tempMat = np.linalg.inv(rotationsMatrix) * np.linalg.inv(intrinsics) * uvPoint
            #tempMat2 = rotationsMatrix.inv() * tvec
            tempMat2 = np.linalg.inv(rotationsMatrix) * tvec
            s = zConst + tempMat2[2,0]
            s = s / tempMat[2,0]
            wcPoint = np.linalg.inv(rotationsMatrix) * (s * np.linalg.inv(intrinsics) * uvPoint - tvec)

            realPoint = [wcPoint[0,0], wcPoint[1,0], wcPoint[2,0]]
            """

            # Calculate world coordinates
            #wcPoint = np.matmul(np.linalg.inv(intrinsics), (uvPoint - tvec)) # intrinsics^-1 * (uvPoint - tvec)
            #wcPoint = np.matmul(np.linalg.inv(rotationsMatrix), wcPoint) # rotationsMatrix^-1 * (intrinsics^-1 * (uvPoint - tvec))

            wcPoint = m.Tvec

            # Write world coordinates with time stamp to file
            arucoCoordinates.append([timedelta(), wcPoint[0], wcPoint[1], wcPoint[2]])

            #print("World coordinates: " + str(wcPoint))
            #print('\n')

            # Draw marker frame on input image
            coordinates = m.getCenter()
            uvPoint = np.ones([2, 1])
            uvPoint[0] = coordinates[0]
            uvPoint[1] = coordinates[1]
            mutex_image.acquire(1)
            m.draw(cv_image, np.array([255, 255, 255]), 2)
            #cv2.circle(cv_image, (uvPoint[0], uvPoint[1]), 10, (255, 0, 0), -1)
            #print("ArUco: x = {0}, y = {1}, z = {2}".format(wcPoint[0], wcPoint[1], wcPoint[2]))
            #print("ArUco: u = {0}, v = {1}".format(uvPoint[0], uvPoint[1]))
            #print("world coordinates - uv: x = {0}, y = {1}, z = {2}".format(abs()))
            #mutex_vicon.acquire(1)
            #cv2.circle(cv_image, (int(x_vicon_transf), int(y_vicon_transf)), 10, (0, 0, 255), -1)
            #print("UWB: x = {0}, y = {1}, z = {2}".format(x_vicon, y_vicon, z_vicon))
            #print("UWB: u = {0}, v = {1}".format(x_vicon_transf, y_vicon_transf))
            #mutex_vicon.release()
            mutex_image.release()

        # show frame
        mutex_image.acquire(1)
        cv2.imshow("frame", cv_image)
        mutex_image.release()
        # Terminate ROS when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("terminate")
            break

        # read next frame
        #ret, frame = cap.read()

if __name__ == '__main__':

    # Create arrays for the recording
    viconCoordinates = []
    arucoCoordinates = []

    # Measure start time
    start = datetime.datetime.now()

    # Start aruco marker detection in separate thread
    arucoThread = threading.Thread(target=getCoordinatesFromMarker)
    arucoThread.daemon = True
    arucoThread.start()

    # Initialize ROS
    rosInit()

    # ROS run
    rospy.spin()

    # Wait for aruco thread
    arucoThread.join()

    # Open file and write header
    file = h5py.File("/data/SP1/catkin_ws/src/record_worldcoordinates/aruco_vicon_output.hdf5", "w")

    # Wirte date sets to file
    dset = file.create_dataset("vicon_coordinates", data = viconCoordinates, compression ="gzip", compression_opts = 9)
    dset = file.create_dataset("aruco_coordinates", data = arucoCoordinates, compression = "gzip", compression_opts = 9)
    file.flush()
    file.close()
