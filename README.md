# ROTI3DBFUWBAV 
Robus	object tracking	in 3D	by fusing ultra-wideband and vision

# Contained scripts/ROS nodes #

##Scripts##

###aruco/main.py##
ArUco sample, used to test ArUco python functionality.

###calibration/calibration.py###
Python script for the camera calibration with OpenCV.

###kabsch/Calibration.m, kabsch/CalibrationVicon.m###
Matlab scripts to match the coordinate systems with the Kabsch algorithm.

###tracker###
KCF tracker base implementation without ROS.

###validation_plot/validation_plot.py###
Python script for the offline evaluation of the ekf. Measures the rmse of the uwb and the ekf with respect to the VICON data.

##ROS nodes##

###publish_image, publish_image_node###
Publishes images, recorded with a camera, as ROS messages.

###uwb, uwb_tracker_node.py###
Publishes the information provided by the UWB system as ROS messages.

###uwb_aruco, uwb_aruco_node.py###
Receives ROS messages from the uwb and publish_image node, detects ArUco markers and saves the locations provided by
both systems into a hdf5 file.

###vicon_aruco, vicon_aruco_node.py###
Receives ROS messages from the VICON system and publish_image node, detects ArUco markers and saves the locations provided by both systems into a hdf5 file.

###vision_tracker, vision_tracker_node###
The KCF tracker publishes the 2D pixel coordinates as ROS message as well as boolean ROS message which indicate, if the object is lost or not. 

###fusing, fusing_node.py###
The EKF receives ROS messages from the vision_tracker and uwb node and publishes the fused positions as ROS messages. 

###visualization, visualization_node###
Displays the positions provided by the UWB, the vision tracker and by the EKF on top of the picture. 

###validation, validation_node.py###
Records the required information to perform the evaluation.
