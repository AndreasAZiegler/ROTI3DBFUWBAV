'''
Created on 05.04.2016

@author: fehlfarbe
'''
import cv2
import numpy as np
import aruco
import datetime

if __name__ == '__main__':
    
    # load board and camera parameters
    #boardconfig = aruco.BoardConfiguration("chessboardinfo_small_meters.yml")
    camparam = aruco.CameraParameters()
    #camparam.readFromXMLFile("camera.yml")
    camparam.readFromXMLFile("logitech.yml")

    # create detector and set parameters
    #detector = aruco.BoardDetector()
    #detector.setParams(boardconfig, camparam)
    # set minimum marker size for detection
    #markerdetector = detector.getMarkerDetector()
    markerdetector = aruco.MarkerDetector()
    #markerdetector.setMinMaxSize(0.01)
    
    # load video
    #cap = cv2.VideoCapture("example.mp4")
    cap = cv2.VideoCapture(0)
    [ret, frame] = cap.read()

    # Open file and write header
    file = open('aruco_output.csv', 'w+')
    file.write("Time stamp, X, Y, Z\n")

    if not ret:
        print("can't open video!")
        exit(-1)


    #frame = cv2.imread("./2016-04-15-062737.jpg")

    while ret:
        #likelihood = markerdetector.detect(frame)

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
            string = (datetime.datetime.now().strftime("%I:%M:%S") + ',' + str(wcPoint[0])[1:-1] + ',' + str(wcPoint[1])[1:-1] + ',' + str(wcPoint[2])[1:-1] + '\n')
            file.write(string)

            print("World coordinates: " + str(wcPoint))
            print('\n')

            # Draw marker frame on input image
            m.draw(frame, np.array([255, 255, 255]), 2)
            
        # show frame
        cv2.imshow("frame", frame)
        cv2.waitKey(100)
        
        # read next frame
        ret, frame = cap.read()

    file.close()
