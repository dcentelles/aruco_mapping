/*********************************************************************************************/ /**
 * @file aruco_mapping.cpp
 *
 * Copyright (c)
 * Smart Robotic Systems
 * March 2015
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/* Author: Jan Bacik */

#ifndef ARUCO_MAPPING_CPP
#define ARUCO_MAPPING_CPP

#include <aruco_mapping/aruco_mapping_static.h>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
namespace aruco_mapping {

ArucoMappingStatic::ArucoMappingStatic(ros::NodeHandle &nh)
    : marker_size_(0.1),        // Marker size in m
      calib_filename_("empty"), // Calibration filepath
      space_type_("plane"),     // Space type - 2D plane
      gui_(true), debug_image_(true), debug_image_topic_("debug_image"),
      image_topic_("/image_raw"), nh_("~")

{
  double temp_marker_size;

  // Parse params from launch file
  nh_.getParam("calibration_file", calib_filename_);
  nh_.getParam("marker_size", temp_marker_size);
  nh_.getParam("pace_type", space_type_);
  nh_.getParam("gui", gui_);
  nh_.getParam("debug_image", debug_image_);
  nh_.getParam("debug_image_topic", debug_image_topic_);
  nh_.getParam("image_topic", image_topic_);
  nh_.getParam("camera_info", camera_info_);
  // Double to float conversion
  marker_size_ = float(temp_marker_size);

  if (calib_filename_ == "empty")
    ROS_WARN("Calibration filename empty! Check the launch file paths");
  else {
    ROS_INFO_STREAM("Calibration file path: " << calib_filename_);
    ROS_INFO_STREAM("Marker Size: " << marker_size_);
    ROS_INFO_STREAM("Type of space: " << space_type_);
    ROS_INFO_STREAM("debug_image: " << debug_image_);
    ROS_INFO_STREAM("debug_image_topic: " << debug_image_topic_);
    ROS_INFO_STREAM("image_topic: " << image_topic_);
  }

  // ROS publishers
  marker_msg_pub_ = nh.advertise<aruco_mapping::ArucoMarker>("aruco_poses", 1);
  marker_visualization_pub_ =
      nh.advertise<visualization_msgs::Marker>("aruco_markers", 1);

  // ARUCO detector settings
  detector_.setDictionary(aruco::Dictionary::DICT_TYPES::ARUCO_MIP_36h12);
  //  detector_.setCornerRefinementMethod(
  //      aruco::MarkerDetector::CornerRefinementMethod::LINES);
  //  detector_.setMinMaxSize(0.03, 0.8);
  //  detector_.setThresholdParams(50,50);
  //  detector_.setThresholdParamRange(10,10); //With 10 10 is slow but more
  //  detections
  //  detector_.setCornerRefinementMethod(aruco::MarkerDetector::CornerRefinementMethod::NONE,20);

  if (camera_info_ != "") {
    sensor_msgs::CameraInfoConstPtr msg =
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_,
                                                            nh_); //, 10.0);
    rosCameraInfo2ArucoCamParams(*msg);
    ROS_INFO("Camera parameters from camera_info topic");
  } else {

    // Parse data from calibration file
    parseCalibrationFile(calib_filename_);
  }
  // Initialize OpenCV window
  if (gui_)
    cv::namedWindow("BGR8", CV_WINDOW_AUTOSIZE);

  if (debug_image_) {
    image_transport::ImageTransport it(nh_);
    marker_debug_image_pub_ = it.advertise(debug_image_topic_, 1);
  }
  image_transport::ImageTransport it(nh);
  img_sub_ = it.subscribe(
      image_topic_, 1, &aruco_mapping::ArucoMappingStatic::imageCallback, this);
}

ArucoMappingStatic::~ArucoMappingStatic() {}

bool ArucoMappingStatic::rosCameraInfo2ArucoCamParams(
    const sensor_msgs::CameraInfo &msg) {
  cout << "Reading calibration from camera_info topic" << endl;
  cv::Mat cameraMatrix(3, 3, CV_64F);
  cv::Mat distortionCoeff(4, 1, CV_64F);

  for (int i = 0; i < 9; ++i)
    cameraMatrix.at<double>(i % 3, i - (i % 3) * 3) = msg.K[i];
  if (msg.D.size() == 4) {
    for (int i = 0; i < 4; ++i)
      distortionCoeff.at<double>(i, 0) = msg.D[i];
  } else {
    cout << "Length of camera_info D vector is not 4!!!" << endl;
    for (int i = 0; i < 4; ++i)
      distortionCoeff.at<double>(i, 0) = msg.D[i];
  }

  aruco_calib_params_.setParams(cameraMatrix, distortionCoeff,
                                cv::Size(msg.height, msg.width));

  cout << "Camera Matrix is: " << cameraMatrix << endl;
  cout << "Distortion coefficients are: " << distortionCoeff << endl;
  cout << "Height and width are: " << msg.height << " , " << msg.width << endl;
}

bool ArucoMappingStatic::parseCalibrationFile(std::string calib_filename) {
  sensor_msgs::CameraInfo camera_calibration_data;
  std::string camera_name = "camera";

  camera_calibration_parsers::readCalibrationIni(calib_filename, camera_name,
                                                 camera_calibration_data);

  // Alocation of memory for calibration data
  cv::Mat intrinsics(3, 3, CV_64F);
  cv::Mat distortion_coeff(5, 1, CV_64F);
  cv::Size image_size;

  image_size.width = camera_calibration_data.width;
  image_size.height = camera_calibration_data.height;

  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 3; j++)
      intrinsics.at<double>(i, j) = camera_calibration_data.K.at(3 * i + j);

  for (size_t i = 0; i < 5; i++)
    distortion_coeff.at<double>(i, 0) = camera_calibration_data.D.at(i);

  ROS_INFO_STREAM("Image width: " << image_size.width);
  ROS_INFO_STREAM("Image height: " << image_size.height);
  ROS_INFO_STREAM("Intrinsics:" << std::endl << intrinsics);
  ROS_INFO_STREAM("Distortion: " << distortion_coeff);

  // Load parameters to aruco_calib_param_ for aruco detection
  aruco_calib_params_.setParams(intrinsics, distortion_coeff, image_size);

  // Simple check if calibration data meets expected values
  if ((intrinsics.at<double>(2, 2) == 1) &&
      (distortion_coeff.at<double>(0, 4) == 0)) {
    ROS_INFO_STREAM("Calibration data loaded successfully");
    return true;
  } else {
    ROS_WARN("Wrong calibration data, check calibration file and filepath");
    return false;
  }
}

void ArucoMappingStatic::imageCallback(
    // const sensor_msgs::ImageConstPtr &original_image) {
    const sensor_msgs::ImageConstPtr &msg) {
  // Create cv_brigde instance

  if (true) {
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    processImage(cv_ptr->image);
  } else {
    cout << "Camera info not received" << endl;
  }
}

bool ArucoMappingStatic::processImage(cv::Mat input_image) {
  std::vector<aruco::Marker> temp_markers;

  // Detect markers
  temp_markers =
      detector_.detect(input_image, aruco_calib_params_, marker_size_);

  cv::Mat displayimg = input_image.clone();

  for (size_t i = 0; i < temp_markers.size(); i++) {

    auto detectedMarker = &temp_markers[i];
    detectedMarker->calculateExtrinsics(marker_size_, aruco_calib_params_,
                                        false);
    // Draw marker convex, ID, cube and axis
    detectedMarker->draw(displayimg, cv::Scalar(0, 0, 255), 2);
    aruco::CvDrawingUtils::draw3dCube(displayimg, *detectedMarker,
                                      aruco_calib_params_);
    aruco::CvDrawingUtils::draw3dAxis(displayimg, *detectedMarker,
                                      aruco_calib_params_);

    auto camera_tf = arucoMarker2Tf(*detectedMarker);
    auto camera_tf_inverse = camera_tf.inverse();

    std::stringstream marker_tf_id;
    marker_tf_id << "aruco_marker_" << detectedMarker->id;
    broadcaster_.sendTransform(tf::StampedTransform(
        camera_tf_inverse, ros::Time::now(), marker_tf_id.str(), "camera"));
  }

  if (gui_) {
    cv::imshow("BGR8", displayimg);
    cv::waitKey(20);
  }
  if (debug_image_) {
    debug_image_msg_ =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", displayimg).toImageMsg();
    marker_debug_image_pub_.publish(debug_image_msg_);
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////

tf::Transform ArucoMappingStatic::arucoMarker2Tf(const aruco::Marker &marker) {
  //  return transform;
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Mat Rvec64;
  marker.Rvec.convertTo(Rvec64, CV_64FC1);
  cv::Rodrigues(Rvec64, rot);
  cv::Mat tran64;
  marker.Tvec.convertTo(tran64, CV_64FC1);

  tf::Matrix3x3 tf_rot(
      rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
      rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
      rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));

  tf::Vector3 tf_orig(tran64.at<double>(0, 0), tran64.at<double>(1, 0),
                      tran64.at<double>(2, 0));

  return tf::Transform(tf_rot, tf_orig);
}

} // namespace aruco_mapping

#endif // ARUCO_MAPPING_CPP
