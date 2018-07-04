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

#include <aruco_mapping/aruco_mapping_fixed.h>
#include <sensor_msgs/CameraInfo.h>

namespace aruco_mapping {

ArucoMappingFixed::ArucoMappingFixed(ros::NodeHandle &nh)
    : listener_(new tf::TransformListener), // Initialize TF Listener
      num_of_markers_(10),                  // Number of used markers
      marker_size_(0.1),                    // Marker size in m
      calib_filename_("empty"),             // Calibration filepath
      space_type_("plane"),                 // Space type - 2D plane
      roi_allowed_(false),                  // ROI not allowed by default
      first_marker_detected_(false), // First marker not detected by defualt
      lowest_marker_id_(-1),         // Lowest marker ID
      marker_counter_(0),            // Reset marker counter
      closest_camera_id_(-1), // Reset closest camera id (camera_{marker's id})
      gui_(true), debug_image_(true), debug_image_topic_("debug_image"),
      image_topic_("/image_raw"), nh_("~"), useCamInfo_(false)

{
  double temp_marker_size;

  // Parse params from launch file
  nh_.getParam("calibration_file", calib_filename_);
  nh_.getParam("marker_size", temp_marker_size);
  nh_.getParam("num_of_markers", num_of_markers_);
  nh_.getParam("pace_type", space_type_);
  nh_.getParam("roi_allowed", roi_allowed_);
  nh_.getParam("roi_x", roi_x_);
  nh_.getParam("roi_y", roi_y_);
  nh_.getParam("roi_w", roi_w_);
  nh_.getParam("roi_h", roi_h_);
  nh_.getParam("gui", gui_);
  nh_.getParam("debug_image", debug_image_);
  nh_.getParam("debug_image_topic", debug_image_topic_);
  nh_.getParam("image_topic", image_topic_);
  nh_.param<bool>("use_camera_info", useCamInfo_, true);
  nh_.param<bool>("use_rectified_images", useRectifiedImages_, true);
  nh_.getParam("camera_info", camera_info_);
  // Double to float conversion
  marker_size_ = float(temp_marker_size);

  if (calib_filename_ == "empty")
    ROS_WARN("Calibration filename empty! Check the launch file paths");
  else {
    ROS_INFO_STREAM("Calibration file path: " << calib_filename_);
    ROS_INFO_STREAM("Number of markers: " << num_of_markers_);
    ROS_INFO_STREAM("Marker Size: " << marker_size_);
    ROS_INFO_STREAM("Type of space: " << space_type_);
    ROS_INFO_STREAM("ROI allowed: " << roi_allowed_);
    ROS_INFO_STREAM("ROI x-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI y-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI width: " << roi_w_);
    ROS_INFO_STREAM("ROI height: " << roi_h_);
    ROS_INFO_STREAM("debug_image: " << debug_image_);
    ROS_INFO_STREAM("debug_image_topic: " << debug_image_topic_);
    ROS_INFO_STREAM("image_topic: " << image_topic_);
  }

  // ROS publishers
  marker_msg_pub_ = nh.advertise<aruco_mapping::ArucoMarker>("aruco_poses", 1);
  marker_visualization_pub_ =
      nh.advertise<visualization_msgs::Marker>("aruco_markers", 1);

  if (useCamInfo_) {
    sensor_msgs::CameraInfoConstPtr msg =
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_,
                                                            nh_); //, 10.0);
    rosCameraInfo2ArucoCamParams(*msg, useRectifiedImages_);
    ROS_INFO("Camera parameters from camera_info topic");
  } else {

    // Parse data from calibration file
    parseCalibrationFile(calib_filename_);
  }
  // Initialize OpenCV window
  if (gui_)
    cv::namedWindow("Mono8", CV_WINDOW_AUTOSIZE);

  // Resize marker container
  markers_.resize(num_of_markers_);

  // Default markers_ initialization
  for (size_t i = 0; i < num_of_markers_; i++) {
    markers_[i].previous_marker_id = -1;
    markers_[i].visible = false;
    markers_[i].marker_id = -1;
  }

  if (debug_image_) {
    image_transport::ImageTransport it(nh_);
    marker_debug_image_pub_ = it.advertise(debug_image_topic_, 1);
  }
  image_transport::ImageTransport it(nh);
  img_sub_ = it.subscribe(
      image_topic_, 1, &aruco_mapping::ArucoMappingFixed::imageCallback, this);
  detector_.setDetectionMode(aruco::DetectionMode::DM_NORMAL);
  detector_.setDictionary("ARUCO_MIP_36h12");
}

ArucoMappingFixed::~ArucoMappingFixed() { delete listener_; }

bool ArucoMappingFixed::rosCameraInfo2ArucoCamParams(
    const sensor_msgs::CameraInfo &cam_info, bool useRectifiedParameters) {
  // Alocation of memory for calibration data
  cv::Mat intrinsics(3, 3, CV_64F);
  cv::Mat distortion_coeff(4, 1, CV_64F);
  cv::Size image_size;

  image_size.width = cam_info.width;
  image_size.height = cam_info.height;

  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 3; j++)
      intrinsics.at<double>(i, j) = cam_info.K.at(3 * i + j);

  for (size_t i = 0; i < 4; i++)
    distortion_coeff.at<double>(i, 0) = cam_info.D.at(i);

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

bool ArucoMappingFixed::parseCalibrationFile(std::string calib_filename) {
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

void ArucoMappingFixed::imageCallback(
    const sensor_msgs::ImageConstPtr &original_image) {
  // Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(original_image,
                                 sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s",
              e.what());
    return;
  }

  // sensor_msgs::Image to OpenCV Mat structure
  last_image_ = cv_ptr->image;

  // region of interest
  if (roi_allowed_ == true)
    last_image_ = cv_ptr->image(cv::Rect(roi_x_, roi_y_, roi_w_, roi_h_));

  // Marker detection
  processImage(last_image_, last_image_);

  // Show image
  if (gui_) {
    cv::imshow("Mono8", last_image_);
    cv::waitKey(10);
  }
  if (debug_image_) {
    debug_image_msg_ =
        cv_bridge::CvImage(std_msgs::Header(), "mono8", last_image_)
            .toImageMsg();
    marker_debug_image_pub_.publish(debug_image_msg_);
  }
}

bool ArucoMappingFixed::processImage(cv::Mat input_image,
                                     cv::Mat output_image) {
  std::vector<aruco::Marker> temp_markers;

  // Set visibility flag to false for all markers
  for (size_t i = 0; i < num_of_markers_; i++)
    markers_[i].visible = false;

  // Save previous marker count
  marker_counter_previous_ = marker_counter_;

  // Detect markers
  detector_.detect(input_image, temp_markers, aruco_calib_params_,
                   marker_size_);

  for (size_t i = 0; i < temp_markers.size(); i++) {
    // Draw marker convex, ID, cube and axis
    temp_markers[i].draw(output_image, cv::Scalar(0, 0, 255), 2);
    aruco::CvDrawingUtils::draw3dCube(output_image, temp_markers[i],
                                      aruco_calib_params_);
    aruco::CvDrawingUtils::draw3dAxis(output_image, temp_markers[i],
                                      aruco_calib_params_);

    auto detectedMarker = &temp_markers[i];

    auto camera_tf = arucoMarker2Tf(*detectedMarker);
    auto camera_tf_inverse = camera_tf.inverse();

    std::stringstream marker_tf_id;
     marker_tf_id << "aruco_marker_" << detectedMarker->id;

     // TF from marker to its camera
     broadcaster_.sendTransform(tf::StampedTransform(
         camera_tf_inverse, ros::Time::now(),
         marker_tf_id.str(), "camera"));
     break;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////

void ArucoMappingFixed::publishTfs() {
  if (first_marker_detected_) {
    // Actual Marker
    std::stringstream marker_tf_id;
    marker_tf_id << "marker_" << markers_[0].marker_id;
    broadcaster_.sendTransform(
        tf::StampedTransform(markers_[0].tf_to_previous, ros::Time::now(),
                             "base_marker", marker_tf_id.str()));

    std::stringstream camera_tf_id;
    camera_tf_id << "camera_" << markers_[0].marker_id;
    broadcaster_.sendTransform(tf::StampedTransform(
        markers_[0].current_camera_tf_inverse, ros::Time::now(),
        marker_tf_id.str(), camera_tf_id.str()));

    ros::Duration(BROADCAST_WAIT_INTERVAL).sleep();
  }

  for (int i = 1; i < marker_counter_; i++) {
    auto minfo = &markers_[i];
    //    // Actual Marker
    std::stringstream marker_tf_id;
    marker_tf_id << "marker_" << minfo->marker_id;
    // Older marker - or base_marker
    std::stringstream marker_tf_id_old;
    marker_tf_id_old << "marker_" << minfo->previous_marker_id;
    broadcaster_.sendTransform(
        tf::StampedTransform(minfo->tf_to_previous, ros::Time::now(),
                             marker_tf_id_old.str(), marker_tf_id.str()));

    // Position of camera to its marker
    std::stringstream camera_tf_id;
    camera_tf_id << "camera_" << minfo->marker_id;
    broadcaster_.sendTransform(
        tf::StampedTransform(minfo->current_camera_tf_inverse, ros::Time::now(),
                             marker_tf_id.str(), camera_tf_id.str()));

    ros::Duration(BROADCAST_WAIT_INTERVAL).sleep();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////

void ArucoMappingFixed::publishMarker(geometry_msgs::Pose marker_pose,
                                      int marker_id, int index) {
  visualization_msgs::Marker vis_marker;

  if (index == 0)
    vis_marker.header.frame_id = "base_marker";
  else {
    std::stringstream marker_tf_id_old;
    marker_tf_id_old << "marker_" << markers_[index].previous_marker_id;
    vis_marker.header.frame_id = marker_tf_id_old.str();
  }

  vis_marker.header.stamp = ros::Time::now();
  vis_marker.ns = "basic_shapes";
  vis_marker.id = marker_id;
  vis_marker.type = visualization_msgs::Marker::CUBE;
  vis_marker.action = visualization_msgs::Marker::ADD;

  vis_marker.pose = marker_pose;
  vis_marker.scale.x = marker_size_;
  vis_marker.scale.y = marker_size_;
  vis_marker.scale.z = RVIZ_MARKER_HEIGHT;

  vis_marker.color.r = RVIZ_MARKER_COLOR_R;
  vis_marker.color.g = RVIZ_MARKER_COLOR_G;
  vis_marker.color.b = RVIZ_MARKER_COLOR_B;
  vis_marker.color.a = RVIZ_MARKER_COLOR_A;

  vis_marker.lifetime = ros::Duration(RVIZ_MARKER_LIFETIME);

  marker_visualization_pub_.publish(vis_marker);
}

////////////////////////////////////////////////////////////////////////////////////////////////

tf::Transform ArucoMappingFixed::arucoMarker2Tf(const aruco::Marker &marker) {
  cv::Mat marker_rotation(3, 3, CV_32FC1);
  cv::Rodrigues(marker.Rvec, marker_rotation);
  cv::Mat marker_translation = marker.Tvec;

  cv::Mat rotate_to_ros(3, 3, CV_32FC1);
  rotate_to_ros.at<float>(0, 0) = -1.0;
  rotate_to_ros.at<float>(0, 1) = 0;
  rotate_to_ros.at<float>(0, 2) = 0;
  rotate_to_ros.at<float>(1, 0) = 0;
  rotate_to_ros.at<float>(1, 1) = 0;
  rotate_to_ros.at<float>(1, 2) = 1.0;
  rotate_to_ros.at<float>(2, 0) = 0.0;
  rotate_to_ros.at<float>(2, 1) = 1.0;
  rotate_to_ros.at<float>(2, 2) = 0.0;

  marker_rotation = marker_rotation; // * rotate_to_ros.t();

  // Origin solution
  tf::Matrix3x3 marker_tf_rot(
      marker_rotation.at<float>(0, 0), marker_rotation.at<float>(0, 1),
      marker_rotation.at<float>(0, 2), marker_rotation.at<float>(1, 0),
      marker_rotation.at<float>(1, 1), marker_rotation.at<float>(1, 2),
      marker_rotation.at<float>(2, 0), marker_rotation.at<float>(2, 1),
      marker_rotation.at<float>(2, 2));

  tf::Vector3 marker_tf_tran(marker_translation.at<float>(0, 0),
                             marker_translation.at<float>(1, 0),
                             marker_translation.at<float>(2, 0));

  ROS_INFO("(%d) :: [%.5f : %.5f : %.5f]", marker.id,
           marker_translation.at<float>(0, 0),
           marker_translation.at<float>(1, 0),
           marker_translation.at<float>(2, 0));
  return tf::Transform(marker_tf_rot, marker_tf_tran);
  //  cv::Mat rot(3, 3, CV_64FC1);
  //  cv::Mat Rvec64;
  //  marker.Rvec.convertTo(Rvec64, CV_64FC1);
  //  cv::Rodrigues(Rvec64, rot);
  //  cv::Mat tran64;
  //  marker.Tvec.convertTo(tran64, CV_64FC1);

  //  tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1),
  //  rot.at<double>(0,2),
  //                       rot.at<double>(1,0), rot.at<double>(1,1),
  //                       rot.at<double>(1,2),
  //                       rot.at<double>(2,0), rot.at<double>(2,1),
  //                       rot.at<double>(2,2));

  //  tf::Vector3 tf_orig(tran64.at<double>(0,0), tran64.at<double>(1,0),
  //  tran64.at<double>(2,0));

  //  return tf::Transform(tf_rot, tf_orig);
}

} // aruco_mapping

#endif // ARUCO_MAPPING_CPP
