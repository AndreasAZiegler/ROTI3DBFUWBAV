#!/usr/bin/env python2
'''
Created on 05.04.2016

@author: fehlfarbe
'''
import cv2
import numpy as np
import aruco
import time
import datetime
import rospy
from uwb.msg import UWBTracker
import threading
import h5py
import numpy as np

lock = threading.Lock()

def timedelta():
    #delta =  time.mktime((datetime.datetime.now().timetuple())) - time.mktime((start.timetuple()))
    delta =  (datetime.datetime.now() - start).total_seconds() * 1000.0
    return delta

def writeToFile(string):
    # Thread block at this line until it can obtain lock
    lock.acquire()

    file.write(string)

def rosInit():
    rospy.init_node('record_worldcoordinates', anonymous=True)
    rospy.Subscriber("UWB_Tracker", UWBTracker, callback)

# Callback function used when UWVTracker messages was received. Adds the x, y, z coordinates and the according variances with a time stamp
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.state[1])

    # Write world coordinates with time stamp to file
    #string = (datetime.datetime.now().strftime("%I:%M:%S") + ',' + str(data.state[0])[1:-1] + ',' + str(data.state[1])[1:-1] + ',' + str(data.state[2])[1:-1] + ',' + str(data.covariance[0])[1:-1] + ',' + str(data.covariance[7])[1:-1] + ',' + str(data.covariance[14])[1:-1] + '\n')
    #writeToFile(string)
    uwbCoordinates.append([timedelta(), data.state[0], data.state[1], data.state[2]])
    uwbVariances.append([timedelta(), data.covariance[0], data.covariance[7], data.covariance[14]])

def getCoordinatesFromMarker():
    # load board and camera parameters
    camparam = aruco.CameraParameters()
    camparam.readFromXMLFile("/data/SP1/catkin_ws/src/record_worldcoordinates/camera.yml")
    #camparam.readFromXMLFile("logitech.yml")

    # create detector and set parameters
    markerdetector = aruco.MarkerDetector()
    # set minimum marker size for detection
    #markerdetector.setMinMaxSize(0.01)

    # load video
    #cap = cv2.VideoCapture("example.mp4")
    cap = cv2.VideoCapture(0)
    [ret, frame] = cap.read()

    if not ret:
        print("can't open video!")
        exit(-1)

    #frame = cv2.imread("./2016-04-15-062737.jpg")

    while ret:
        # Detect marker
        markers = markerdetector.detect(frame, camparam, 200)

        print(str(len(markers)) + " markers detected")

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
            print("Marker Coordinates: " + str(m.getCenter()) + " Tvec: " + str(m.Tvec))

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

            """
            zConst = 5
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
            wcPoint = np.matmul(np.linalg.inv(intrinsics), (uvPoint - tvec)) # intrinsics^-1 * (uvPoint - tvec)
            wcPoint = np.matmul(np.linalg.inv(rotationsMatrix), wcPoint) # rotationsMatrix^-1 * (intrinsics^-1 * (uvPoint - tvec))

            # Write world coordinates with time stamp to file
            #string = (datetime.datetime.now().strftime("%I:%M:%S") + ',' + str(wcPoint[0])[1:-1] + ',' + str(wcPoint[1])[1:-1] + ',' + str(wcPoint[2])[1:-1] + ',' + str(0) + ',' + str(0) + ',' + str(0) + '\n')
            #writeToFile(string)
            arucoCoordinates.append([timedelta(), wcPoint[0], wcPoint[1], wcPoint[2]])

            print("World coordinates: " + str(wcPoint))
            print('\n')

            # Draw marker frame on input image
            m.draw(frame, np.array([255, 255, 255]), 2)

        # show frame
        cv2.imshow("frame", frame)
        # Terminate ROS when 'q' is pressed
        if cv2.waitKey(70) & 0xFF == ord('q'):
            rospy.signal_shutdown("terminate")
            break

        # read next frame
        ret, frame = cap.read()

if __name__ == '__main__':

    # Create arrays for the recording
    uwbCoordinates = []
    uwbVariances = []
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
    file = h5py.File("/data/SP1/catkin_ws/src/record_worldcoordinates/aruco_output.hdf5", "w")

    # Wirte date sets to file
    #dset = file.create_dataset("mydataset", (100,), dtype='i')
    dset = file.create_dataset("uwb_coordinates", data = uwbCoordinates, compression = "gzip", compression_opts = 9)
    dset = file.create_dataset("uwb_variances", data = uwbVariances, compression = "gzip", compression_opts = 9)
    dset = file.create_dataset("aruco_coordinates", data = arucoCoordinates, compression = "gzip", compression_opts = 9)
    file.flush()
    file.close()
