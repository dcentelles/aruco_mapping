/*********************************************************************************************/ /**
 * @file aruco_mapping.h
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

#ifndef ARUCO_MAPPING_H
#define ARUCO_MAPPING_H

// Standard ROS libraries
#include <camera_calibration_parsers/parse_ini.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// Aruco libraries
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <aruco/cvdrawingutils.h>

// OpenCV libraries
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// Custom message
#include <aruco_mapping/ArucoMarker.h>

/** \brief Aruco mapping namespace */
namespace aruco_mapping {

/** \brief Client class for Aruco mapping */
class ArucoMapping {
public:
  /** \brief Struct to keep marker information */
  struct MarkerInfo {

    bool visible;           // Marker visibile in actual image?
    int marker_id;          // Marker ID
    int previous_marker_id; // Used for chaining markers
    geometry_msgs::Pose
        geometry_msg_to_previous; // Position with respect to previous marker
    geometry_msgs::Pose geometry_msg_to_base_marker; // Position with respect to
                                                     // base_marker's origin
    tf::StampedTransform tf_to_previous; // TF with respect to previous marker
    tf::StampedTransform
        tf_to_base_marker; // TF with respect to base_marker's origin
    geometry_msgs::Pose
        current_camera_pose; // Position of camera with respect to the marker
    tf::Transform current_camera_tf; // TF of camera with respect to the marker
    tf::Transform current_camera_tf_inverse;
    bool baseLinked = false;
  };

public:
  /** \brief Construct a client for EZN64 USB control*/
  ArucoMapping(ros::NodeHandle &nh);

  ~ArucoMapping();

  int get_iThresParam1() { return _i_thres_param_1; }
  int get_iThresParam2() { return _i_thres_param_2; }
  void set_iThresParam1(int p) { _i_thres_param_1 = p; }
  void set_iThresParam2(int p) { _i_thres_param_2 = p; }
  int get_thresParam1() { return _thres_param_1; }
  int get_thresParam2() { return _thres_param_2; }
  void set_thresParam1(int p) { _thres_param_1 = p; }
  void set_thresParam2(int p) { _thres_param_2 = p; }
  void setMarkerDetectorThreshold() {
    //detector_.setThresholdParams(_thres_param_1, _thres_param_2);
  }

  /** \brief Callback function to handle image processing*/
  void imageCallback(const sensor_msgs::ImageConstPtr &original_image);

private:
  /** \brief Function to parse data from calibration file*/
  bool parseCalibrationFile(std::string filename);

  /** \brief Function to publish all known TFs*/
  void publishTfs();

  /** \brief Function to publish all known markers for visualization purposes*/
  void publishMarker(geometry_msgs::Pose markerPose, int MarkerID, int rank);

  /** \brief Publisher of visualization_msgs::Marker message to "aruco_markers"
   * topic*/
  ros::Publisher marker_visualization_pub_;

  /** \brief Publisher of aruco_mapping::ArucoMarker custom message*/
  ros::Publisher marker_msg_pub_;

  /** \brief Compute TF from marker detector result*/
  tf::Transform arucoMarker2Tf(const aruco::Marker &marker);

  /** \brief Process actual image, detect markers and compute poses */
  bool processImage(cv::Mat input_image);

  /**
     * @brief rosCameraInfo2ArucoCamParams gets the camera intrinsics from a
   * CameraInfo message and copies them
     *                                     to aruco_ros own data structure
     * @param cam_info
     * @return
     */
  bool rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo &cam_info);

  // Launch file params
  std::string calib_filename_;
  std::string space_type_;
  float marker_size_;
  int num_of_markers_, desired_base_marker_id_;
  bool gui_, debug_image_;
  std::string debug_image_topic_, image_topic_, camera_info_, base_marker_name_, marker_prefix_;
  sensor_msgs::ImagePtr debug_image_msg_;
  ros::NodeHandle nh_;

  image_transport::Subscriber img_sub_;

  /** \brief Publisher of debug image (with marker frames)*/
  image_transport::Publisher marker_debug_image_pub_;

  /** \brief Container holding MarkerInfo data about all detected markers */
  std::vector<MarkerInfo> markers_;

  /** \brief Actual TF of camera with respect to base_marker's origin */
  tf::StampedTransform base_marker_position_transform_;

  /** \brief Actual Pose of camera with respect to base_marker's origin */
  geometry_msgs::Pose base_marker_position_geometry_msg_;

  aruco::CameraParameters aruco_calib_params_;

  aruco::MarkerDetector detector_;
  cv::Mat last_image_;
  MarkerInfo *base_marker_;

  int marker_counter_;
  int marker_counter_previous_;
  int closest_camera_id_;
  int base_marker_id_;
  bool first_marker_detected_;

  tf::TransformListener *listener_;
  tf::TransformBroadcaster broadcaster_;

  // Consts
  static const int CV_WAIT_KEY = 10;
  static const int CV_WINDOW_MARKER_LINE_WIDTH = 2;

  static constexpr double WAIT_FOR_TRANSFORM_INTERVAL = 2.0;
  static constexpr double BROADCAST_WAIT_INTERVAL = 0.0001;
  static constexpr double INIT_MIN_SIZE_VALUE = 1000000;

  static constexpr double RVIZ_MARKER_HEIGHT = 0.01;
  static constexpr double RVIZ_MARKER_LIFETIME = 0.2;
  static constexpr double RVIZ_MARKER_COLOR_R = 1.0;
  static constexpr double RVIZ_MARKER_COLOR_G = 1.0;
  static constexpr double RVIZ_MARKER_COLOR_B = 1.0;
  static constexpr double RVIZ_MARKER_COLOR_A = 1.0;

  static constexpr double THIS_IS_FIRST_MARKER = -2;

  double _thres_param_1, _thres_param_2;
  int _i_thres_param_1, _i_thres_param_2;

}; // ArucoMapping class
} // aruco_mapping namespace

#endif // ARUCO_MAPPING_H
