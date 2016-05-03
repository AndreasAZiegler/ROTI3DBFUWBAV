/*****************************************************************************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

ROS bits and integration added by Florian Lier flier at techfak dot uni-bielefeld dot de

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <unistd.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
using namespace cv;
using namespace aruco;
string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=-1;
int ThePyrDownLevel;
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos,void*);
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CP,Size size);
// Determines the average time required for detection
pair<double,double> AvrgTime(0,0) ;
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=0;
bool update_images;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  Mat img;

public:
  ImageConverter()
    : it_(nh_)
  {
    // subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ar_follow/image", 1, &ImageConverter::imageCb, this);
  }

  ~ImageConverter()
  {
   printf("Stopped Image Import");
  }

  cv::Mat getCurrentImage() {
        return img;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    img = cv_ptr->image;
  }

};

bool readArguments ( int argc,char **argv )
{
    if (argc<2) {
        cerr<< "Invalid number of arguments" <<endl;
        cerr<< "Usage: (in.avi|live|copy) [intrinsics.yml] [size]" <<endl;
        return false;
    }
    TheInputVideo=argv[1];
    if (argc>=3)
        TheIntrinsicFile=argv[2];
    if (argc>=4)
        TheMarkerSize=atof(argv[3]);
    if (argc==3)
        cerr<< "NOTE: You need makersize to see 3d info!" <<endl;
    return true;
}

int main(int argc,char **argv){
	// Show images, press "SPACE" to diable image
        // rendering to save CPU time
        update_images = true;

	if (readArguments(argc,argv)==false) {
		return 0;
	}

	TheVideoCapturer.open(0);
	// Check video is open
	if (!TheVideoCapturer.isOpened()) {
		cerr<<"Could not open video"<<endl;
		return 0;
	}

	// Read first image to get the dimensions
	TheVideoCapturer>>TheInputImage;

	// Read camera parameters if passed
	if (TheIntrinsicFile!="") {
		TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
		TheCameraParameters.resize(TheInputImage.size());
	}

	// Configure other parameters
	if (ThePyrDownLevel>0)
		MDetector.pyrDown(ThePyrDownLevel);

	// Create gui
	cv::namedWindow("THRESHOLD IMAGE",1);
	cv::namedWindow("INPUT IMAGE",1);

	MDetector.getThresholdParams( ThresParam1,ThresParam2);
	MDetector.setCornerRefinementMethod(MarkerDetector::LINES);

	iThresParam1=ThresParam1;
	iThresParam2=ThresParam2;
	cv::createTrackbar("ThresParam1", "INPUT IMAGE",&iThresParam1, 13, cvTackBarEvents);
	cv::createTrackbar("ThresParam2", "INPUT IMAGE",&iThresParam2, 13, cvTackBarEvents);
	char key=0;
	int index=0;

	// ROS messaging init
	ros::init(argc, argv, "aruco_tf_publisher");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	// Publish pose message and buffer up to 100 messages
	ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("aruco_pose", 100);
	tf::TransformBroadcaster broadcaster;

	// Capture until press ESC or until the end of the video
	while ((key != 'x') && (key!=27) && TheVideoCapturer.grab() && ros::ok()){

                ros::spinOnce();

		if (TheVideoCapturer.retrieve(TheInputImage)){
			// Copy image
			index++; // Number of images captured
			double tick = (double)getTickCount();// For checking the speed

                        // Detection of markers in the image passed
			MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,TheMarkerSize);

			// Check the speed by calculating the mean speed of all iterations
			AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
			AvrgTime.second++;

			// Show the detection time
			// cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;

                        TheInputImage.copyTo(TheInputImageCopy);

			geometry_msgs::Pose msg;

			float x_t, y_t, z_t;
			float roll,yaw,pitch;
			bool found = (TheMarkers.size()>0)?true:false;

			if (found)  {
				x_t = -TheMarkers[0].Tvec.at<Vec3f>(0,0)[0];
				y_t = TheMarkers[0].Tvec.at<Vec3f>(0,0)[1];
				z_t = TheMarkers[0].Tvec.at<Vec3f>(0,0)[2];
				// printf("%4.2f %4.2f %4.2f\n",x_t,y_t,z_t);

				cv::Mat rot_mat(3,3,cv::DataType<float>::type);
				// You need to apply cv::Rodrigues() in order to obatain angles wrt to camera coords
				cv::Rodrigues(TheMarkers[0].Rvec,rot_mat);

				pitch   = -atan2(rot_mat.at<float>(2,0), rot_mat.at<float>(2,1));
				yaw     = acos(rot_mat.at<float>(2,2));
				roll    = -atan2(rot_mat.at<float>(0,2), rot_mat.at<float>(1,2));
			}else {
				// x_t = 0.0;
				// y_t = 0.0;
				// z_t = -10.0;
				// yaw  = 0.0;
				// roll = 0.0;
				// pitch = 0.0;
                                printf("Marker _NOT_ found\n");
			}

			// Marker rotation should be initially zero (just for convenience)
			float p_off = CV_PI;
			float r_off = CV_PI/2;
			float y_off = CV_PI/2;

			// See: http://en.wikipedia.org/wiki/Flight_dynamics
			if (found){
				printf( "Angles (deg) wrt Flight Dynamics: roll:%5.2f pitch:%5.2f yaw:%5.2f \n", (roll-r_off)*(180.0/CV_PI), (pitch-p_off)*(180.0/CV_PI), (yaw-y_off)*(180.0/CV_PI));
				printf( "       Marker distance in metres:  x_d:%5.2f   y_d:%5.2f z_d:%5.2f \n", x_t, y_t, z_t);
			}

			if (ros::ok()){
				// Publish TF message including the offsets
				tf::Quaternion quat = tf::createQuaternionFromRPY(roll-p_off, pitch+p_off, yaw-y_off);
				broadcaster.sendTransform(tf::StampedTransform(tf::Transform(quat, tf::Vector3(x_t, y_t, z_t)), ros::Time::now(),"camera", "marker"));

				// Now publish the pose message, remember the offsets
				msg.position.x = x_t;
				msg.position.y = y_t;
				msg.position.z = z_t;
				geometry_msgs::Quaternion p_quat = tf::createQuaternionMsgFromRollPitchYaw(roll-r_off, pitch+p_off, yaw-y_off);
				msg.orientation = p_quat;
				pose_pub.publish(msg);
				ros::spinOnce();
			}

			// Print other rectangles that contains no valid markers
			/** for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
				aruco::Marker m( MDetector.getCandidates()[i],999);
				m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
			}*/
			// Draw a 3d cube in each marker if there is 3d info

                       if (TheCameraParameters.isValid()){
				for (unsigned int i=0;i<TheMarkers.size();i++) {
					CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
					CvDrawingUtils::draw3dAxis(TheInputImageCopy,TheMarkers[i],TheCameraParameters);
				}
			}

			// Show input with augmented information and the thresholded image
			if (update_images) {
				cv::imshow("INPUT IMAGE",TheInputImageCopy);
				cv::imshow("THRESHOLD IMAGE",MDetector.getThresholdedImage());
			}
		}else {
		      printf("retrieve failed\n");
		}

		key=cv::waitKey(1);

                // If space is hit, don't render the image.
		if (key == ' '){
			update_images = !update_images;
		}
	}
}

void checkbox_callback(bool value){
	update_images = value;
}

void cvTackBarEvents(int pos,void*)
{
    if (iThresParam1<3) iThresParam1=3;
    if (iThresParam1%2!=1) iThresParam1++;
    if (ThresParam2<1) ThresParam2=1;
    ThresParam1=iThresParam1;
    ThresParam2=iThresParam2;
    MDetector.setThresholdParams(ThresParam1,ThresParam2);

    // Recompute
    MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters);
    TheInputImage.copyTo(TheInputImageCopy);

    for (unsigned int i=0;i<TheMarkers.size();i++) TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);

    // Print other rectangles that contains no valid markers
    /* for (unsigned int i=0;i<MDetector.getCandidates().size();i++) {
        aruco::Marker m( MDetector.getCandidates()[i],999);
        m.draw(TheInputImageCopy,cv::Scalar(255,0,0));
    } */

    // Draw a 3d cube in each marker if there is 3d info
    if (TheCameraParameters.isValid())
        for (unsigned int i=0;i<TheMarkers.size();i++)
            CvDrawingUtils::draw3dCube(TheInputImageCopy,TheMarkers[i],TheCameraParameters);

    cv::imshow("INPUT IMAGE",TheInputImageCopy);
    cv::imshow("THRESHOLD IMAGE",MDetector.getThresholdedImage());
}
