/*!
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

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Jan Bacik */

#ifndef ARUCO_MAPPING_CPP
#define ARUCO_MAPPING_CPP

#include <aruco_mapping/aruco_mapping.h>
#include <sensor_msgs/CameraInfo.h>

using std::cout;
using std::endl;

namespace aruco_mapping {

void cvTackBarEvents(int pos, void *userData) {
  (void)(pos);
  ArucoMapping *ammd = static_cast<ArucoMapping *>(userData);
  //  if (ammd->get_iThresParam1() < 3)
  //    ammd->set_iThresParam1(3);
  //  if (ammd->get_iThresParam1() % 2 != 1) {
  //    int p = ammd->get_iThresParam1();
  //    ammd->set_iThresParam1(p++);
  //  }
  //  if (ammd->get_iThresParam2() < 1)
  //    ammd->set_iThresParam2(1);
  ammd->set_thresParam1(ammd->get_iThresParam1());
  ammd->set_thresParam2(ammd->get_iThresParam2());
  ammd->setMarkerDetectorThreshold();
}

ArucoMapping::ArucoMapping(ros::NodeHandle &nh)
    : listener_(new tf::TransformListener),   // Initialize TF Listener
      num_of_markers_(10),                    // Number of used markers
      marker_size_(0.1),                      // Marker size in m
      calib_filename_("empty"),               // Calibration filepath
      space_type_("plane"),                   // Space type - 2D plane
      first_marker_detected_(false),  // First marker not detected by defualt
      base_marker_id_(-1),            // Lowest marker ID
      marker_counter_(0),             // Reset marker counter
      closest_camera_id_(-1),  // Reset closest camera id (camera_{marker's id})
      gui_(true), debug_image_(true), debug_image_topic_("debug_image"),
      image_topic_("/image_raw"), base_marker_name_("base_marker"),
      marker_prefix_("aruco_marker_"), nh_("~"), desired_base_marker_id_(-1),
      _thres_param_1(10), _thres_param_2(3) {
  double temp_marker_size;

  // Parse params from launch file
  nh_.getParam("base_marker", desired_base_marker_id_);
  nh_.getParam("calibration_file", calib_filename_);
  nh_.getParam("camera_info", camera_info_);
  nh_.getParam("debug_image", debug_image_);
  nh_.getParam("debug_image_topic", debug_image_topic_);
  nh_.getParam("gui", gui_);
  nh_.getParam("image_topic", image_topic_);
  nh_.getParam("marker_prefix", marker_prefix_);
  nh_.getParam("marker_size", temp_marker_size);
  nh_.getParam("num_of_markers", num_of_markers_);
  nh_.getParam("space_type", space_type_);
  nh_.getParam("threshold_1", _thres_param_1);
  nh_.getParam("threshold_2", _thres_param_2);
  // Double to float conversion
  marker_size_ = static_cast<float>(temp_marker_size);

  if (calib_filename_ == "empty") {
    ROS_WARN("Calibration filename empty! Check the launch file paths");
  } else {
    ROS_INFO_STREAM("Calibration file path: " << calib_filename_);
    ROS_INFO_STREAM("Number of markers: " << num_of_markers_);
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

  // detector_.setDetectionMode(aruco::DetectionMode::DM_NORMAL);
  //  detector_.setThresholdMethod(
  //      aruco::MarkerDetector::ThresholdMethods::ADPT_THRES);
  detector_.setDictionary(aruco::Dictionary::DICT_TYPES::ARUCO_MIP_36h12);
  detector_.setDetectionMode(aruco::DetectionMode::DM_NORMAL);
  aruco::MarkerDetector::Params params;
  params.setDetectionMode(aruco::DetectionMode::DM_NORMAL, 0.1);
  params.setCornerRefinementMethod(aruco::CORNER_LINES);
  params.detectEnclosedMarkers(false);

  //  detector_.setCornerRefinementMethod(
  //      aruco::MarkerDetector::CornerRefinementMethod::LINES);
  //  detector_.setMinMaxSize(0.03, 0.8);
  //  detector_.setThresholdParams(_thres_param_1, _thres_param_2);

  if (camera_info_ != "") {
    sensor_msgs::CameraInfoConstPtr msg =
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_, nh_);
                                                                     //, 10.0);
    rosCameraInfo2ArucoCamParams(*msg);
    ROS_INFO("Camera parameters from camera_info topic");
  } else {
    // Parse data from calibration file
    parseCalibrationFile(calib_filename_);
  }

  // Initialize OpenCV window
  if (gui_) {
    cv::namedWindow("Display", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    // detector_.getThresholdParams(_thres_param_1, _thres_param_2);
    cv::createTrackbar("ThresParam1", "Display", &_i_thres_param_1, 20,
                       cvTackBarEvents, this);
    cv::createTrackbar("ThresParam2", "Display", &_i_thres_param_2, 20,
                       cvTackBarEvents, this);
  }

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
  img_sub_ = it.subscribe(image_topic_, 1,
                          &aruco_mapping::ArucoMapping::imageCallback, this);
}

ArucoMapping::~ArucoMapping() { delete listener_; }

bool ArucoMapping::rosCameraInfo2ArucoCamParams(
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

bool ArucoMapping::parseCalibrationFile(std::string calib_filename) {
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

void ArucoMapping::imageCallback(
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

bool ArucoMapping::processImage(cv::Mat input_image) {
  std::vector<aruco::Marker> temp_markers;
  cv::Mat displayimg = input_image.clone();

  // Set visibility flag to false for all markers
  for (size_t i = 0; i < num_of_markers_; i++)
    markers_[i].visible = false;

  // Save previous marker count
  marker_counter_previous_ = marker_counter_;

  // Detect markers
  temp_markers =
      detector_.detect(input_image, aruco_calib_params_, marker_size_);

  // If no marker found, print statement
  if (temp_markers.size() == 0)
    ROS_DEBUG("No marker found!");

  for (size_t i = 0; i < temp_markers.size(); i++) {
    auto detectedMarker = &temp_markers[i];
    detectedMarker->calculateExtrinsics(marker_size_, aruco_calib_params_,
                                        false);

    // Draw marker convex, ID, cube and axis
    temp_markers[i].draw(displayimg, cv::Scalar(0, 0, 255), 2);
    aruco::CvDrawingUtils::draw3dCube(displayimg, temp_markers[i],
                                      aruco_calib_params_);
    aruco::CvDrawingUtils::draw3dAxis(displayimg, temp_markers[i],
                                      aruco_calib_params_);
  }

  if (gui_) {
    cv::Mat img_display(input_image.rows, 3. / 2. * input_image.cols, CV_8UC3);
    cv::Mat left(img_display,
                 cv::Rect(0, 0, input_image.cols, input_image.rows));
    displayimg.copyTo(left);
    cv::Rect trr(input_image.cols, 0, input_image.cols / 2,
                 input_image.rows / 2);
    cv::Mat topRight(img_display, trr);
    cv::Rect lrr(input_image.cols, input_image.rows / 2, input_image.cols / 2,
                 input_image.rows / 2);
    cv::Mat lowRight(img_display, lrr);
    cv::Mat aux;
    cv::cvtColor(detector_.getThresholdedImage(), aux, cv::COLOR_GRAY2RGB);
    cv::resize(aux, lowRight,
               cv::Size(input_image.cols / 2, input_image.rows / 2));
    cv::resize(input_image, topRight,
               cv::Size(input_image.cols / 2, input_image.rows / 2));

    // Add text for identifying images
    cv::putText(lowRight, "thresholded image",
                cv::Point2f(20, lowRight.rows - 20), cv::FONT_HERSHEY_SIMPLEX,
                1, cv::Scalar(255, 255, 255), 2);
    cv::putText(topRight, "original image", cv::Point2f(20, topRight.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
    cv::putText(left, "Detections", cv::Point2f(20, input_image.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

    // Display images
    cv::imshow("Display", img_display);
    cv::waitKey(20);
  }
  if (debug_image_) {
    debug_image_msg_ =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", displayimg).toImageMsg();
    marker_debug_image_pub_.publish(debug_image_msg_);
  }
  //------------------------------------------------------
  // FIRST MARKER DETECTED
  //------------------------------------------------------
  if ((temp_markers.size() > 0) && (first_marker_detected_ == false)) {
    if (desired_base_marker_id_ >= 0) {
      for (size_t i = 0; i < temp_markers.size(); i++) {
        if (temp_markers[i].id == desired_base_marker_id_) {
          base_marker_id_ = temp_markers[i].id;
          break;
        }
      }
      if (base_marker_id_ !=
          desired_base_marker_id_) {  // base marker not detected yet
        return false;
      }
    } else {
      // Detect lowest marker ID
      base_marker_id_ = temp_markers[0].id;
      for (size_t i = 1; i < temp_markers.size(); i++) {
        if (temp_markers[i].id < base_marker_id_) {
          base_marker_id_ = temp_markers[i].id;
        }
      }
    }

    // Set flag
    first_marker_detected_ = true;
    ROS_DEBUG_STREAM("Base marker id: " << base_marker_id_);

    // Identify lowest marker ID with base_marker's origin
    markers_[0].marker_id = base_marker_id_;

    markers_[0].geometry_msg_to_base_marker.position.x = 0;
    markers_[0].geometry_msg_to_base_marker.position.y = 0;
    markers_[0].geometry_msg_to_base_marker.position.z = 0;

    markers_[0].geometry_msg_to_base_marker.orientation.x = 0;
    markers_[0].geometry_msg_to_base_marker.orientation.y = 0;
    markers_[0].geometry_msg_to_base_marker.orientation.z = 0;
    markers_[0].geometry_msg_to_base_marker.orientation.w = 1;

    // Relative position and Global position
    markers_[0].geometry_msg_to_previous.position.x = 0;
    markers_[0].geometry_msg_to_previous.position.y = 0;
    markers_[0].geometry_msg_to_previous.position.z = 0;

    markers_[0].geometry_msg_to_previous.orientation.x = 0;
    markers_[0].geometry_msg_to_previous.orientation.y = 0;
    markers_[0].geometry_msg_to_previous.orientation.z = 0;
    markers_[0].geometry_msg_to_previous.orientation.w = 1;

    // Transformation Pose to TF
    tf::Vector3 position;
    position.setX(0);
    position.setY(0);
    position.setZ(0);

    tf::Quaternion rotation;
    rotation.setRPY(0, 0, 0);
    rotation.normalize();

    markers_[0].tf_to_previous.setOrigin(position);
    markers_[0].tf_to_previous.setRotation(rotation);

    // Relative position of first marker equals Global position
    markers_[0].tf_to_base_marker = markers_[0].tf_to_previous;

    // Increase count
    marker_counter_++;

    // Set sign of visibility of first marker
    markers_[0].visible = true;
    markers_[0].baseLinked = true;
    base_marker_ = &markers_[0];

    ROS_INFO_STREAM("First marker with ID: " << markers_[0].marker_id
                                             << " detected");

    // First marker does not have any previous marker
    markers_[0].previous_marker_id = THIS_IS_FIRST_MARKER;

    base_marker_name_ = marker_prefix_ + std::to_string(base_marker_id_);
  }

  //------------------------------------------------------
  // FOR EVERY MARKER DO
  //------------------------------------------------------

  // Flag to keep info if any_known marker_visible in actual image
  bool any_known_marker_visible = false;

  for (size_t i = 0; i < temp_markers.size(); i++) {
    int index;
    int current_marker_id = temp_markers[i].id;

    // Existing marker ?
    bool existing = false;
    int temp_counter = 0;

    while ((existing == false) && (temp_counter < marker_counter_)) {
      if (markers_[temp_counter].marker_id == current_marker_id) {
        index = temp_counter;
        existing = true;
        ROS_DEBUG_STREAM("Existing marker with ID: " << current_marker_id
                                                     << "found");
      }
      temp_counter++;
    }

    // New marker ?
    if (existing == false) {
      index = marker_counter_;
      markers_[index].marker_id = current_marker_id;
      existing = true;
      ROS_DEBUG_STREAM("New marker with ID: " << current_marker_id << " found");
    }

    // Change visibility flag of new marker
    for (size_t j = 0; j < marker_counter_; j++) {
      for (size_t k = 0; k < temp_markers.size(); k++) {
        if (markers_[j].marker_id == temp_markers[k].id) {
          markers_[j].visible = true;
          if (base_marker_->visible) {
            markers_[j].baseLinked = true;
          }
        }
      }
    }

    auto marker_info = &markers_[index];

    if (!first_marker_detected_)
      break;

    // Naming - TFs
    std::stringstream camera_tf_id;
    std::stringstream marker_tf_id;

    camera_tf_id << "camera_" << marker_info->marker_id;
    marker_tf_id << "marker_" << marker_info->marker_id;

    marker_info->current_camera_tf = arucoMarker2Tf(temp_markers[i]);
    marker_info->current_camera_tf_inverse =
        marker_info->current_camera_tf.inverse();

    tf::Vector3 marker_origin = marker_info->current_camera_tf.getOrigin();
    marker_info->current_camera_pose.position.x = marker_origin.getX();
    marker_info->current_camera_pose.position.y = marker_origin.getY();
    marker_info->current_camera_pose.position.z = marker_origin.getZ();

    tf::Quaternion marker_quaternion =
        marker_info->current_camera_tf.getRotation();
    marker_info->current_camera_pose.orientation.x = marker_quaternion.getX();
    marker_info->current_camera_pose.orientation.y = marker_quaternion.getY();
    marker_info->current_camera_pose.orientation.z = marker_quaternion.getZ();
    marker_info->current_camera_pose.orientation.w = marker_quaternion.getW();

    // Array ID of markers, which position of new marker is calculated
    int last_marker_index;
    std::stringstream camera_tf_id_old;
    std::stringstream marker_tf_id_old;

    // TF from marker to its camera
    broadcaster_.sendTransform(tf::StampedTransform(
        marker_info->current_camera_tf_inverse, ros::Time::now(),
        marker_tf_id.str(), camera_tf_id.str()));

    any_known_marker_visible = true;
    // Testing, if is possible calculate position of a new marker to old known
    // marker
    for (int k = 0; k < index; k++) {
      auto minfo = &markers_[k];
      if ((minfo->visible == true)) {
        if (minfo->previous_marker_id != -1) {
          camera_tf_id_old << "camera_" << minfo->marker_id;
          marker_tf_id_old << "marker_" << minfo->marker_id;
          marker_info->previous_marker_id = minfo->marker_id;
          if (minfo->baseLinked)
            marker_info->baseLinked = true;

          last_marker_index = k;

          // TF from marker to its camera
          broadcaster_.sendTransform(tf::StampedTransform(
              marker_info->current_camera_tf_inverse, ros::Time::now(),
              marker_tf_id.str(), camera_tf_id.str()));
          // TF from old camera (should be new camera) to marker
          broadcaster_.sendTransform(tf::StampedTransform(
              marker_info->current_camera_tf, ros::Time::now(),
              camera_tf_id_old.str(), marker_tf_id.str()));
          // TF from old marker to old camera
          broadcaster_.sendTransform(tf::StampedTransform(
              minfo->current_camera_tf_inverse, ros::Time::now(),
              marker_tf_id_old.str(), camera_tf_id_old.str()));
          ros::Duration(BROADCAST_WAIT_INTERVAL).sleep();

          try {
            listener_->waitForTransform(
                marker_tf_id_old.str(), marker_tf_id.str(), ros::Time(0),
                ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
            listener_->lookupTransform(marker_tf_id_old.str(),
                                       marker_tf_id.str(), ros::Time(0),
                                       marker_info->tf_to_previous);

            // Save origin and quaternion of calculated TF
            marker_origin = marker_info->tf_to_previous.getOrigin();
            marker_quaternion = marker_info->tf_to_previous.getRotation();

            //            // If plane type selected roll, pitch and Z axis are
            //            zero
            //            if(space_type_ == "plane")
            //            {
            //              double roll, pitch, yaw;
            //              tf::Matrix3x3(marker_quaternion).getRPY(roll,pitch,yaw);
            //              roll = 0;
            //              pitch = 0;
            //              marker_origin.setZ(0);
            //              marker_quaternion.setRPY(pitch,roll,yaw);
            //            }

            marker_info->tf_to_previous.setRotation(marker_quaternion);
            marker_info->tf_to_previous.setOrigin(marker_origin);

            marker_origin = marker_info->tf_to_previous.getOrigin();
            marker_info->geometry_msg_to_previous.position.x =
                marker_origin.getX();
            marker_info->geometry_msg_to_previous.position.y =
                marker_origin.getY();
            marker_info->geometry_msg_to_previous.position.z =
                marker_origin.getZ();

            marker_quaternion = marker_info->tf_to_previous.getRotation();
            marker_info->geometry_msg_to_previous.orientation.x =
                marker_quaternion.getX();
            marker_info->geometry_msg_to_previous.orientation.y =
                marker_quaternion.getY();
            marker_info->geometry_msg_to_previous.orientation.z =
                marker_quaternion.getZ();
            marker_info->geometry_msg_to_previous.orientation.w =
                marker_quaternion.getW();

            marker_origin = marker_info->current_camera_tf_inverse.getOrigin();
            marker_info->current_camera_pose.position.x = marker_origin.getX();
            marker_info->current_camera_pose.position.y = marker_origin.getY();
            marker_info->current_camera_pose.position.z = marker_origin.getZ();

            marker_quaternion =
                marker_info->current_camera_tf_inverse.getRotation();
            marker_info->current_camera_pose.orientation.x =
                marker_quaternion.getX();
            marker_info->current_camera_pose.orientation.y =
                marker_quaternion.getY();
            marker_info->current_camera_pose.orientation.z =
                marker_quaternion.getZ();
            marker_info->current_camera_pose.orientation.w =
                marker_quaternion.getW();
          } catch (tf::TransformException &e) {
            ROS_ERROR("visible: Not able to lookup transform: %s", e.what());
          }
          break;
        }
      }
    }

    if (index == marker_counter_) {
      ROS_INFO("New marker detected: %d", marker_info->marker_id);
      marker_counter_++;
    }
  }
  publishTfs();
  //------------------------------------------------------
  // Compute which of visible markers is the closest to the camera
  //------------------------------------------------------
  if (any_known_marker_visible) {
    double minimal_distance = INIT_MIN_SIZE_VALUE;
    bool baseLinked = false;
    for (int k = 0; k < num_of_markers_; k++) {
      double a, b, c, size;
      auto minfo = &markers_[k];
      // If marker is visible, distance is calculated
      if (minfo->visible == true && minfo->baseLinked) {
        baseLinked = true;
        a = minfo->current_camera_pose.position.x;
        b = minfo->current_camera_pose.position.y;
        c = minfo->current_camera_pose.position.z;
        size = std::sqrt((a * a) + (b * b) + (c * c));
        if (size < minimal_distance) {
          minimal_distance = size;
          closest_camera_id_ = minfo->marker_id;
        }
      }
    }
    if (baseLinked) {
      std::stringstream closest_camera_tf_name;
      closest_camera_tf_name << "camera_" << closest_camera_id_;

      listener_->waitForTransform(base_marker_name_,
                                  closest_camera_tf_name.str(), ros::Time(0),
                                  ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
      try {
        listener_->lookupTransform(base_marker_name_,
                                   closest_camera_tf_name.str(), ros::Time(0),
                                   base_marker_position_transform_);
        broadcaster_.sendTransform(tf::StampedTransform(
            base_marker_position_transform_, ros::Time::now(),
            base_marker_name_, "camera"));
      } catch (tf::TransformException &ex) {
        ROS_ERROR("base: Not able to lookup transform");
      }
    }
  }
  publishTfs();
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////

void ArucoMapping::publishTfs() {
  if (first_marker_detected_) {
    // Actual Marker
    std::stringstream marker_tf_id;
    marker_tf_id << "marker_" << markers_[0].marker_id;
    broadcaster_.sendTransform(
        tf::StampedTransform(markers_[0].tf_to_previous, ros::Time::now(),
                             base_marker_name_, marker_tf_id.str()));

    std::stringstream camera_tf_id;
    camera_tf_id << "camera_" << markers_[0].marker_id;
    broadcaster_.sendTransform(tf::StampedTransform(
        markers_[0].current_camera_tf_inverse, ros::Time::now(),
        marker_tf_id.str(), camera_tf_id.str()));

    // Cubes for RVIZ - markers
    publishMarker(
      markers_[0].geometry_msg_to_previous, markers_[0].marker_id, 0);
    ros::Duration(BROADCAST_WAIT_INTERVAL).sleep();
  }

  for (int i = 1; i < marker_counter_; i++) {
    auto minfo = &markers_[i];
    //    // Actual Marker
    std::stringstream marker_tf_id;
    marker_tf_id << "marker_" << minfo->marker_id;
    // Older marker - or base_marker
    std::stringstream marker_tf_id_old;
    if (minfo->previous_marker_id != -1) {
      marker_tf_id_old << "marker_" << minfo->previous_marker_id;
      broadcaster_.sendTransform(
          tf::StampedTransform(minfo->tf_to_previous, ros::Time::now(),
                               marker_tf_id_old.str(), marker_tf_id.str()));
    }
    // Position of camera to its marker
    std::stringstream camera_tf_id;
    camera_tf_id << "camera_" << minfo->marker_id;
    broadcaster_.sendTransform(
        tf::StampedTransform(minfo->current_camera_tf_inverse, ros::Time::now(),
                             marker_tf_id.str(), camera_tf_id.str()));

    // Cubes for RVIZ - markers
    publishMarker(
      markers_[i].geometry_msg_to_previous, markers_[i].marker_id, i);
    ros::Duration(BROADCAST_WAIT_INTERVAL).sleep();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////

void ArucoMapping::publishMarker(geometry_msgs::Pose marker_pose, int marker_id,
                                 int index) {
  visualization_msgs::Marker vis_marker;

  if (index == 0) {
    vis_marker.header.frame_id = base_marker_name_;
  } else {
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

tf::Transform ArucoMapping::arucoMarker2Tf(const aruco::Marker &marker) {
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

}  // namespace aruco_mapping

#endif  // ARUCO_MAPPING_CPP
