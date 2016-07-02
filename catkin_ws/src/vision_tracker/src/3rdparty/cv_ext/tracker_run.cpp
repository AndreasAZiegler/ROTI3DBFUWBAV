/*M///////////////////////////////////////////////////////////////////////////////////////
//
// IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
// By downloading, copying, installing or using the software you agree to this license.
// If you do not agree to this license, do not download, install,
// copy or use the software.
//
// License Agreement
// For Open Source Computer Vision Library
// (3-clause BSD License)
//
// Copyright (C) 2000-2015, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
// Copyright (C) 2009-2015, NVIDIA Corporation, all rights reserved.
// Copyright (C) 2010-2013, Advanced Micro Devices, Inc., all rights reserved.
// Copyright (C) 2015, OpenCV Foundation, all rights reserved.
// Copyright (C) 2015, Itseez Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the names of the copyright holders nor the names of the contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall copyright holders or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//M*/

/*
// Original file: https://github.com/Itseez/opencv_contrib/blob/292b8fa6aa403fb7ad6d2afadf4484e39d8ca2f1/modules/tracking/samples/tracker.cpp
// + Author: Klaus Haag
// * Refactor file: Split into files/classes
// * Replace command line arguments
// * Add a variety of additional features
*/

#include "tracker_run.hpp"

#include <ros/ros.h>
#include <iostream>
#include <ctype.h>

#include "init_box_selector.hpp"
#include "cf_tracker.hpp"
#include "mat_consts.hpp"
#include <sstream>

using namespace cv;
using namespace std;
using namespace TCLAP;
using namespace cf_tracking;

const bool ROS_RECORD_OR_PLAY = true;              // false = For recording with rosbag, true = For play with rosbag

TrackerRun::TrackerRun(string windowTitle, ros::NodeHandle *node)
 : _windowTitle(windowTitle),
   _cmd(_windowTitle.c_str(), ' ', "0.1"),
   _debug(0),
   _nh(node),
   _headerSeq(0),
   _it(*_nh)
{
    _tracker = 0;
}

TrackerRun::TrackerRun(string windowTitle)
 : _windowTitle(windowTitle),
   _cmd(_windowTitle.c_str(), ' ', "0.1"),
   _debug(0),
  _it(*_nh)
{
    _tracker = 0;
}

TrackerRun::~TrackerRun()
{
    if (_resultsFile.is_open())
        _resultsFile.close();

    if (_tracker)
    {
        delete _tracker;
        _tracker = 0;
    }
}

void TrackerRun::imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        _rosimage = cv_ptr->image;
    }catch(cv_bridge::Exception& e) {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
    }
}

void TrackerRun::ekfCallback(const geometry_msgs::PointStamped& msg) {
    std::lock_guard<std::mutex> guard(ekfCoordinatesMutex);
    ekfCoordinates[0] = msg.point.x;
    ekfCoordinates[1] = msg.point.y;
}

Parameters TrackerRun::parseCmdArgs(int argc, char** argv)
{
    Parameters paras;

    try{
        ValueArg<int> deviceIdArg("c", "cam", "Camera device id", false, 0, "integer", _cmd);
        ValueArg<string> seqPathArg("s", "seq", "Path to sequence", false, "", "path", _cmd);
        ValueArg<string> expansionArg("i", "image_name_expansion", "image name expansion (only necessary for image sequences) ie. /%.05d.jpg", false, "", "string", _cmd);
        ValueArg<string> initBbArg("b", "box", "Init Bounding Box", false, "-1,-1,-1,-1", "x,y,w,h", _cmd);
        SwitchArg noShowOutputSw("n", "no-show", "Don't show video", _cmd, false);
        ValueArg<string> outputPathArg("o", "out", "Path to output file", false, "", "file path", _cmd);
        ValueArg<string> imgExportPathArg("e", "export", "Path to output folder where the images will be saved with BB", false, "", "folder path", _cmd);
        SwitchArg pausedSw("p", "paused", "Start paused", _cmd, false);
        SwitchArg repeatSw("r", "repeat", "endless loop the same sequence", _cmd, false);
        ValueArg<int> startFrameArg("", "start_frame", "starting frame idx (starting at 1 for the first frame)", false, 1, "integer", _cmd);
        SwitchArg dummySequenceSw("", "mock_sequence", "Instead of processing a regular sequence, a dummy sequence is used to evaluate run time performance.", _cmd, false);
        _tracker = parseTrackerParas(_cmd, argc, argv);

        paras.device = deviceIdArg.getValue();
        paras.sequencePath = seqPathArg.getValue();
        string expansion = expansionArg.getValue();

        size_t foundExpansion = expansion.find_first_of('.');

        if (foundExpansion != string::npos)
            expansion.erase(foundExpansion, 1);

        paras.expansion = expansion;

        paras.outputFilePath = outputPathArg.getValue();
        paras.imgExportPath = imgExportPathArg.getValue();
        paras.showOutput = !noShowOutputSw.getValue();
        paras.paused = pausedSw.getValue();
        paras.repeat = repeatSw.getValue();
        paras.startFrame = startFrameArg.getValue();

        stringstream initBbSs(initBbArg.getValue());

        double initBb[4];

        for (int i = 0; i < 4; ++i)
        {
            string singleValueStr;
            getline(initBbSs, singleValueStr, ',');
            initBb[i] = static_cast<double>(stod(singleValueStr.c_str()));
        }

        paras.initBb = Rect_<double>(initBb[0], initBb[1], initBb[2], initBb[3]);

        if (_debug != 0)
            _debug->init(paras.outputFilePath + "_debug");

        paras.isMockSequence = dummySequenceSw.getValue();
    }
    catch (ArgException &argException)
    {
        cerr << "Command Line Argument Exception: " << argException.what() << endl;
        exit(-1);
    }
    // TODO: properly check every argument and throw exceptions accordingly
    catch (...)
    {
        cerr << "Command Line Argument Exception!" << endl;
        exit(-1);
    }

    return paras;
}

bool TrackerRun::start(int argc, char** argv, bool* stop_flag)
{

    _stop_flag = stop_flag;

    _paras = parseCmdArgs(argc, argv);

    while (true)
    {
        if (init() == false)
            return false;
        if (run() == false)
            return false;

        if (!_paras.repeat || _exit)
            break;

        _boundingBox = _paras.initBb;
        _isTrackerInitialzed = false;
    }

    return true;
}

bool TrackerRun::init()
{
    // ROS init stuff
    //_pub_coordinates = _nh->advertise<tf2_msgs::TFMessage>("vision_coordinates", 10);
    //_pub_coordinates = _nh->advertise<tf::tfMessage>("vision_coordinates", 10);
    _sub_ekfCoordinates = _nh->subscribe("/fusing/ekf_uv_coordinates", 1, &TrackerRun::ekfCallback, this);
    _pub_coordinates = _nh->advertise<geometry_msgs::PointStamped>("/vision_tracker/vision_coordinates", 1);
    _pub_objectDetected = _nh->advertise<std_msgs::Bool>("/vision_tracker/object_detected", 1);

    if(false == ROS_RECORD_OR_PLAY){
      _image_pub = _it.advertise("/vision_tracker/video", 1);
    }else{
      //_image_sub = _it.subscribe("/vision_tracker/video", 1, &TrackerRun::imageCb, this);
      _image_sub = _it.subscribe("/camera/video", 1, &TrackerRun::imageCb, this);
    }
    ros::Rate loop_rate(10);

    ImgAcqParas imgAcqParas;
    imgAcqParas.device = _paras.device;
    imgAcqParas.expansionStr = _paras.expansion;
    imgAcqParas.isMock = _paras.isMockSequence;
    imgAcqParas.sequencePath = _paras.sequencePath;
    _cap.open(imgAcqParas);

    if (!_cap.isOpened())
    {
        cerr << "Could not open device/sequence/video!" << endl;
        exit(-1);
    }
    _cap.init();

    int startIdx = _paras.startFrame - 1;

    // HACKFIX:
    _cap.set(CV_CAP_PROP_POS_FRAMES, startIdx);
    // OpenCV's _cap.set in combination with image sequences is
    // currently bugged on Linux
    // TODO: review when OpenCV 3.0 is stable
    //cv::Mat temp;

    //for (int i = 0; i < startIdx; ++i)
    //    _cap >> temp;
    // HACKFIX END

    if (_paras.showOutput)
        namedWindow(_windowTitle.c_str());

    if (!_paras.outputFilePath.empty())
    {
        _resultsFile.open(_paras.outputFilePath.c_str());

        if (!_resultsFile.is_open())
        {
            std::cerr << "Error: Unable to create results file: "
                << _paras.outputFilePath.c_str()
                << std::endl;

            return false;
        }

        _resultsFile.precision(std::numeric_limits<double>::digits10 - 4);
    }

    if (_paras.initBb.width > 0 || _paras.initBb.height > 0)
    {
        _boundingBox = _paras.initBb;
        _hasInitBox = true;
    }

    _isPaused = _paras.paused;
    _frameIdx = 0;
    return true;
}

bool TrackerRun::run()
{
    bool success = true;

    std::cout << "Switch pause with 'p'" << std::endl;
    std::cout << "Step frame with 'c'" << std::endl;
    std::cout << "Select new target with 'r'" << std::endl;
    std::cout << "Update current tracker model at new location  't'" << std::endl;
    std::cout << "Quit with 'ESC'" << std::endl;

    //while (true)
    while(ros::ok())
    {
        success = update();

        if (!success)
            break;
    }

    _cap.release();

    return true;
}

bool TrackerRun::update()
{
  int64 tStart = 0;
  int64 tDuration = 0;

  if (!_isPaused || _frameIdx == 0 || _isStep) {
    if(false == ROS_RECORD_OR_PLAY){
      _cap >> _image;
    }else {
      _image = _rosimage;
    }

    if (_image.empty()) {
      return false;
    }

    if(false == ROS_RECORD_OR_PLAY) {
      _image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", _image).toImageMsg());
    }

    ++_frameIdx;
  }

  if (!_isTrackerInitialzed) {
    if (!_hasInitBox) {
      Rect box;

      if (!InitBoxSelector::selectBox(_image, box)) {
        return false;
      }

      _boundingBox = Rect_<double>(static_cast<double>(box.x),
                                   static_cast<double>(box.y),
                                   static_cast<double>(box.width),
                                   static_cast<double>(box.height));

      _hasInitBox = true;
    }

    tStart = getTickCount();
    _targetOnFrame = _tracker->reinit(_image, _boundingBox);
    tDuration = getTickCount() - tStart;

    if (_targetOnFrame) {
      _isTrackerInitialzed = true;
    }
  }
  else if (_isTrackerInitialzed && (!_isPaused || _isStep)) {

    if (!_targetOnFrame) {
      // Set new bounding box based on ekf 2D coordinates
      {
        std::lock_guard<std::mutex> guard(ekfCoordinatesMutex);
        //cv::Point newPos<double>(ekfCoordinates[0] - _boundingBox.width * 0.5, ekfCoordinates[1] - _boundingBox.width * 0.5);
        if((ekfCoordinates[0] > 0) && (ekfCoordinates[1] > 0)) {
          //_tracker->updatePosition(cv::Point(ekfCoordinates[0] - _boundingBox.width * 0.5, ekfCoordinates[1] - _boundingBox.width * 0.5));
          _tracker->updatePosition(cv::Point(ekfCoordinates[0], ekfCoordinates[1]));
        }
      }
      //_targetOnFrame = _tracker->reinit(_image, _boundingBox);
    }

    _isStep = false;

    if (_updateAtPos) {
      Rect box;

      if (!InitBoxSelector::selectBox(_image, box)) {
        return false;
      }

      _boundingBox = Rect_<double>(static_cast<double>(box.x),
                                   static_cast<double>(box.y),
                                   static_cast<double>(box.width),
                                   static_cast<double>(box.height));

      _updateAtPos = false;

      std::cout << "UpdateAt_: " << _boundingBox << std::endl;
      tStart = getTickCount();
      _targetOnFrame = _tracker->updateAt(_image, _boundingBox);

      if (!_targetOnFrame) {
        std::cout << "Target not found!" << std::endl;
      }
    }
    else {
      tStart = getTickCount();
      _targetOnFrame = _tracker->update(_image, _boundingBox);
      tDuration = getTickCount() - tStart;
    }

    if (!_targetOnFrame) {
      // Publish ROS message to indicate, that vision tracker can't detect the object
      _detectedObjectMsg.data = false;
      _pub_objectDetected.publish(_detectedObjectMsg);
    } else {
      // Publish ROS message to indicate, that vision tracker can detect the object
      _detectedObjectMsg.data = true;
      _pub_objectDetected.publish(_detectedObjectMsg);
    }


    // Publish positions
    _trans.header.seq = _headerSeq;
    _headerSeq++;
    _trans.header.stamp = ros::Time::now();
    _trans.header.frame_id = string("vision");
    _trans.child_frame_id = string("vision_tracker");
    _trans.transform.translation.x = _boundingBox.x + _boundingBox.width * 0.5;
    _trans.transform.translation.y = _boundingBox.y + _boundingBox.width * 0.5;
    _trans.transform.translation.z = 1;
    _trans.transform.rotation.x = 0;
    _trans.transform.rotation.y = 0;
    _trans.transform.rotation.z = 0;
    _trans.transform.rotation.w = 0;

    _point.header.seq = _headerSeq;
    _point.header.stamp = ros::Time::now();
    _point.header.frame_id = string("vision");
    //_point.point.x = _boundingBox.x + _boundingBox.width * 0.5;
    //_point.point.y = _boundingBox.y + _boundingBox.width * 0.5;
    //_point.point.x = (_boundingBox.x + _boundingBox.width * 0.5) / 1000.0;
    //_point.point.y = (_boundingBox.y + _boundingBox.width * 0.5) / 1000.0;
    _point.point.x = _boundingBox.x + _boundingBox.width / 2;
    _point.point.y = _boundingBox.y + _boundingBox.width / 2;
    std::cout << "Point x: " << _point.point.x << " y: " << _point.point.y << std::endl;
    _point.point.z = 1;

    _msg.transforms.clear();
    _msg.transforms.push_back(_trans);
    ROS_INFO("%f", _point);
    _pub_coordinates.publish(_point);
  }

  double fps = static_cast<double>(getTickFrequency() / tDuration);
  printResults(_boundingBox, _targetOnFrame, fps);


  if (_paras.showOutput) {
    Mat hudImage;
    _image.copyTo(hudImage);
    rectangle(hudImage, _boundingBox, Scalar(0, 0, 255), 2);
    Point_<double> center;
    center.x = _boundingBox.x + _boundingBox.width / 2;
    center.y = _boundingBox.y + _boundingBox.height / 2;
    circle(hudImage, center, 3, Scalar(0, 0, 255), 2);

    stringstream ss;
    ss << "FPS: " << fps;
    putText(hudImage, ss.str(), Point(20, 20), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 0, 0));

    ss.str("");
    ss.clear();
    ss << "#" << _frameIdx;
    putText(hudImage, ss.str(), Point(hudImage.cols - 60, 20), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 0, 0));

    if (_debug != 0) {
      _debug->printOnImage(hudImage);
    }

    if (!_targetOnFrame) {
      cv::Point_<double> tl = _boundingBox.tl();
      cv::Point_<double> br = _boundingBox.br();

      line(hudImage, tl, br, Scalar(0, 0, 255));
      line(hudImage, cv::Point_<double>(tl.x, br.y),
      cv::Point_<double>(br.x, tl.y), Scalar(0, 0, 255));
    }

    imshow(_windowTitle.c_str(), hudImage);

    if (!_paras.imgExportPath.empty()) {
      stringstream ssi;
      ssi << setfill('0') << setw(5) << _frameIdx << ".png";
      std::string imgPath = _paras.imgExportPath + ssi.str();

      try {
        imwrite(imgPath, hudImage);
      }catch (runtime_error& runtimeError) {
        cerr << "Could not write output images: " << runtimeError.what() << endl;
      }
    }

    char c = (char)waitKey(10);

    if (c == 27) {
      _exit = true;
      *_stop_flag = true;
      return false;
    }

    switch (c) {
      case 'p':
        _isPaused = !_isPaused;
        break;
      case 'c':
        _isStep = true;
        break;
      case 'r':
        _hasInitBox = false;
        _isTrackerInitialzed = false;
        break;
      case 't':
        _updateAtPos = true;
        break;
      default:
            ;
    }
  }

  return true;
}

void TrackerRun::printResults(const cv::Rect_<double>& boundingBox, bool isConfident, double fps) {
  if (_resultsFile.is_open()) {
    if (boundingBox.width > 0 && boundingBox.height > 0 && isConfident) {
      _resultsFile << boundingBox.x << ","
      << boundingBox.y << ","
      << boundingBox.width << ","
      << boundingBox.height << ","
      << fps << std::endl;
    }
    else {
      _resultsFile << "NaN, NaN, NaN, NaN, " << fps << std::endl;
    }

    if (_debug != 0) {
      _debug->printToFile();
    }
  }
}

void TrackerRun::setTrackerDebug(cf_tracking::TrackerDebug* debug) {
  _debug = debug;
}
