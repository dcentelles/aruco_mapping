/*********************************************************************************************//**
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

#include <aruco_mapping/aruco_mapping.h>

namespace aruco_mapping
{

ArucoMapping::ArucoMapping(ros::NodeHandle &nh) :
  listener_ (new tf::TransformListener),  // Initialize TF Listener  
  num_of_markers_ (10),                   // Number of used markers
  marker_size_(0.1),                      // Marker size in m
  calib_filename_("empty"),               // Calibration filepath
  space_type_ ("plane"),                  // Space type - 2D plane 
  roi_allowed_ (false),                   // ROI not allowed by default
  first_marker_detected_(false),          // First marker not detected by defualt
  lowest_marker_id_(-1),                  // Lowest marker ID
  marker_counter_(0),                     // Reset marker counter
  closest_camera_id_(-1),               // Reset closest camera id (camera_{marker's id})
  gui_(true),
  debug_image_(true),
  debug_image_topic_("debug_image"),
  image_topic_("/image_raw"),
  nh_("~")
  
{
  double temp_marker_size;  
  
  //Parse params from launch file 
  nh_.getParam("calibration_file", calib_filename_);
  nh_.getParam("marker_size", temp_marker_size);
  nh_.getParam("num_of_markers", num_of_markers_);
  nh_.getParam("pace_type",space_type_);
  nh_.getParam("roi_allowed",roi_allowed_);
  nh_.getParam("roi_x",roi_x_);
  nh_.getParam("roi_y",roi_y_);
  nh_.getParam("roi_w",roi_w_);
  nh_.getParam("roi_h",roi_h_);
  nh_.getParam("gui",gui_);
  nh_.getParam("debug_image", debug_image_);
  nh_.getParam("debug_image_topic", debug_image_topic_);
  nh_.getParam("image_topic", image_topic_);
  // Double to float conversion
  marker_size_ = float(temp_marker_size);
  
  if(calib_filename_ == "empty")
    ROS_WARN("Calibration filename empty! Check the launch file paths");
  else
  {
    ROS_INFO_STREAM("Calibration file path: " << calib_filename_ );
    ROS_INFO_STREAM("Number of markers: " << num_of_markers_);
    ROS_INFO_STREAM("Marker Size: " << marker_size_);
    ROS_INFO_STREAM("Type of space: " << space_type_);
    ROS_INFO_STREAM("ROI allowed: " << roi_allowed_);
    ROS_INFO_STREAM("ROI x-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI y-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI width: "  << roi_w_);
    ROS_INFO_STREAM("ROI height: " << roi_h_);
    ROS_INFO_STREAM("debug_image: "  << debug_image_);
    ROS_INFO_STREAM("debug_image_topic: " << debug_image_topic_);
    ROS_INFO_STREAM("image_topic: " << image_topic_);
  }
    
  //ROS publishers
  marker_msg_pub_           = nh.advertise<aruco_mapping::ArucoMarker>("aruco_poses",1);
  marker_visualization_pub_ = nh.advertise<visualization_msgs::Marker>("aruco_markers",1);
          
  //Parse data from calibration file
  parseCalibrationFile(calib_filename_);

  //Initialize OpenCV window
  if(gui_)
    cv::namedWindow("Mono8", CV_WINDOW_AUTOSIZE);       
      
  //Resize marker container
  markers_.resize(num_of_markers_);
  
  // Default markers_ initialization
  for(size_t i = 0; i < num_of_markers_;i++)
  {
    markers_[i].previous_marker_id = -1;
    markers_[i].visible = false;
    markers_[i].marker_id = -1;
  }

  if(debug_image_)
  {
      image_transport::ImageTransport it(nh_);
      marker_debug_image_pub_ = it.advertise(debug_image_topic_,1);
  }
  image_transport::ImageTransport it(nh);
  img_sub_ = it.subscribe(image_topic_, 1, &aruco_mapping::ArucoMapping::imageCallback, this);
  detector_.setDetectionMode (aruco::DetectionMode::DM_FAST);
  detector_.setDictionary("ARUCO_MIP_36h12");
}

ArucoMapping::~ArucoMapping()
{
 delete listener_;
}

bool
ArucoMapping::parseCalibrationFile(std::string calib_filename)
{
  sensor_msgs::CameraInfo camera_calibration_data;
  std::string camera_name = "camera";

  camera_calibration_parsers::readCalibrationIni(calib_filename, camera_name, camera_calibration_data);

  // Alocation of memory for calibration data
  cv::Mat  *intrinsics       = new(cv::Mat)(3, 3, CV_64F);
  cv::Mat  *distortion_coeff = new(cv::Mat)(5, 1, CV_64F);
  cv::Size *image_size       = new(cv::Size);

  image_size->width = camera_calibration_data.width;
  image_size->height = camera_calibration_data.height;

  for(size_t i = 0; i < 3; i++)
    for(size_t j = 0; j < 3; j++)
    intrinsics->at<double>(i,j) = camera_calibration_data.K.at(3*i+j);

  for(size_t i = 0; i < 5; i++)
    distortion_coeff->at<double>(i,0) = camera_calibration_data.D.at(i);

  ROS_DEBUG_STREAM("Image width: " << image_size->width);
  ROS_DEBUG_STREAM("Image height: " << image_size->height);
  ROS_DEBUG_STREAM("Intrinsics:" << std::endl << *intrinsics);
  ROS_DEBUG_STREAM("Distortion: " << *distortion_coeff);


  //Load parameters to aruco_calib_param_ for aruco detection
  aruco_calib_params_.setParams(*intrinsics, *distortion_coeff, *image_size);

  //Simple check if calibration data meets expected values
  if ((intrinsics->at<double>(2,2) == 1) && (distortion_coeff->at<double>(0,4) == 0))
  {
    ROS_INFO_STREAM("Calibration data loaded successfully");
    return true;
  }
  else
  {
    ROS_WARN("Wrong calibration data, check calibration file and filepath");
    return false;
  }
}

void
ArucoMapping::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
  //Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
    return;
  }
  
  // sensor_msgs::Image to OpenCV Mat structure
  last_image_ = cv_ptr->image;
  
  // region of interest
  if(roi_allowed_==true)
    last_image_ = cv_ptr->image(cv::Rect(roi_x_,roi_y_,roi_w_,roi_h_));

  //Marker detection
  processImage(last_image_,last_image_);
  
  // Show image
  if(gui_)
  {
    cv::imshow("Mono8", last_image_);
    cv::waitKey(10);
  } 
  if(debug_image_)
    {
      debug_image_msg_ = cv_bridge::CvImage(std_msgs::Header(), "mono8", last_image_).toImageMsg();
      marker_debug_image_pub_.publish (debug_image_msg_);
    }
}


bool
ArucoMapping::processImage(cv::Mat input_image,cv::Mat output_image)
{
  std::vector<aruco::Marker> temp_markers;

  //Set visibility flag to false for all markers
  for(size_t i = 0; i < num_of_markers_; i++)
      markers_[i].visible = false;

  // Save previous marker count
  marker_counter_previous_ = marker_counter_;

  // Detect markers
  detector_.detect(input_image,temp_markers,aruco_calib_params_,marker_size_);
    
  // If no marker found, print statement
  if(temp_markers.size() == 0)
    ROS_DEBUG("No marker found!");

  //------------------------------------------------------
  // FIRST MARKER DETECTED
  //------------------------------------------------------
  if((temp_markers.size() > 0) && (first_marker_detected_ == false))
  {
    //Set flag
    first_marker_detected_=true;

    // Detect lowest marker ID
    lowest_marker_id_ = temp_markers[0].id;
    for(size_t i = 0; i < temp_markers.size();i++)
    {
      if(temp_markers[i].id < lowest_marker_id_)
        lowest_marker_id_ = temp_markers[i].id;
    }


    ROS_DEBUG_STREAM("The lowest Id marker " << lowest_marker_id_ );

    // Identify lowest marker ID with base_marker's origin
    markers_[0].marker_id = lowest_marker_id_;

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
    rotation.setX(0);
    rotation.setY(0);
    rotation.setZ(0);
    rotation.setW(1);

    markers_[0].tf_to_previous.setOrigin(position);
    markers_[0].tf_to_previous.setRotation(rotation);

    // Relative position of first marker equals Global position
    markers_[0].tf_to_base_marker=markers_[0].tf_to_previous;

    // Increase count
    marker_counter_++;

    // Set sign of visibility of first marker
    markers_[0].visible=true;

    ROS_INFO_STREAM("First marker with ID: " << markers_[0].marker_id << " detected");

    //First marker does not have any previous marker
    markers_[0].previous_marker_id = THIS_IS_FIRST_MARKER;
  }

  //------------------------------------------------------
  // FOR EVERY MARKER DO
  //------------------------------------------------------
  for(size_t i = 0; i < temp_markers.size();i++)
  {
    int index;
    int current_marker_id = temp_markers[i].id;

    //Draw marker convex, ID, cube and axis
    temp_markers[i].draw(output_image, cv::Scalar(0,0,255),2);
    aruco::CvDrawingUtils::draw3dCube(output_image,temp_markers[i], aruco_calib_params_);
    aruco::CvDrawingUtils::draw3dAxis(output_image,temp_markers[i], aruco_calib_params_);

    // Existing marker ?
    bool existing = false;
    int temp_counter = 0;

    while((existing == false) && (temp_counter < marker_counter_))
    {
      if(markers_[temp_counter].marker_id == current_marker_id)
      {
        index = temp_counter;
        existing = true;
        ROS_DEBUG_STREAM("Existing marker with ID: " << current_marker_id << "found");
      }
        temp_counter++;
    }

    //New marker ?
    if(existing == false)
    {
      index = marker_counter_;
      markers_[index].marker_id = current_marker_id;
      existing = true;
      ROS_DEBUG_STREAM("New marker with ID: " << current_marker_id << " found");
    }

    // Change visibility flag of new marker
    for(size_t j = 0;j < marker_counter_; j++)
    {
      for(size_t k = 0;k < temp_markers.size(); k++)
      {
        if(markers_[j].marker_id == temp_markers[k].id)
          markers_[j].visible = true;
      }
    }

    auto marker_info = &markers_[index];

    if(!first_marker_detected_) break;

    // Naming - TFs
    std::stringstream camera_tf_id;
    std::stringstream marker_tf_id;

    camera_tf_id << "camera_" << marker_info->marker_id;
    marker_tf_id << "marker_" << marker_info->marker_id;

    broadcaster_.sendTransform(tf::StampedTransform(markers_[0].tf_to_previous,ros::Time::now(),"base_marker", marker_tf_id.str()));

    marker_info->current_camera_tf=arucoMarker2Tf(temp_markers[i]);

    tf::Vector3 marker_origin = marker_info->current_camera_tf.getOrigin();
    marker_info->current_camera_pose.position.x = marker_origin.getX();
    marker_info->current_camera_pose.position.y = marker_origin.getY();
    marker_info->current_camera_pose.position.z = marker_origin.getZ();

    tf::Quaternion marker_quaternion = marker_info->current_camera_tf.getRotation();
    marker_info->current_camera_pose.orientation.x = marker_quaternion.getX();
    marker_info->current_camera_pose.orientation.y = marker_quaternion.getY();
    marker_info->current_camera_pose.orientation.z = marker_quaternion.getZ();
    marker_info->current_camera_pose.orientation.w = marker_quaternion.getW();


    // Flag to keep info if any_known marker_visible in actual image
    bool any_known_marker_visible = false;

    // Array ID of markers, which position of new marker is calculated
    int last_marker_index;
    std::stringstream camera_tf_id_old;
    std::stringstream marker_tf_id_old;


    // TF from marker to its camera
    broadcaster_.sendTransform(tf::StampedTransform(marker_info->current_camera_tf.inverse(),ros::Time::now(),
                                                    marker_tf_id.str(), camera_tf_id.str()));


    // Testing, if is possible calculate position of a new marker to old known marker
    for(int k = 0; k < index; k++)
    {
      auto minfo = &markers_[k];
      if((minfo->visible == true))
      {
        if(minfo->previous_marker_id != -1)
        {
          any_known_marker_visible = true;
          camera_tf_id_old << "camera_" << minfo->marker_id;
          marker_tf_id_old << "marker_" << minfo->marker_id;
          marker_info->previous_marker_id = minfo->marker_id;
          last_marker_index = k;

          // TF from marker to its camera
          broadcaster_.sendTransform(tf::StampedTransform(marker_info->current_camera_tf.inverse(),ros::Time::now(),
                                                          marker_tf_id.str(), camera_tf_id.str()));
          // TF from old camera (should be new camera) to marker
          broadcaster_.sendTransform(tf::StampedTransform(marker_info->current_camera_tf,ros::Time::now(),
                                                          camera_tf_id_old.str(),marker_tf_id.str()));
          // TF from old marker to old camera
          broadcaster_.sendTransform(tf::StampedTransform(minfo->current_camera_tf.inverse(),ros::Time::now(),
                                                          marker_tf_id_old.str(),camera_tf_id_old.str()));
          ros::Duration(BROADCAST_WAIT_INTERVAL).sleep();

          try
          {
            listener_->waitForTransform(marker_tf_id_old.str(),marker_tf_id.str(),ros::Time(0),ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
            listener_->lookupTransform(marker_tf_id_old.str(),marker_tf_id.str(),ros::Time(0),
                                       marker_info->tf_to_previous);
          }
          catch(tf::TransformException &e)
          {
            ROS_ERROR("Not able to lookup transform: %s", e.what ());
          }
          break;
         }
       }
     }

    if(index == marker_counter_)
    {
        ROS_INFO("New marker detected: %d", marker_info->marker_id);
        marker_counter_++;
    }
   }
  //publishTfs(false);


/*


        // Save origin and quaternion of calculated TF
        marker_origin = marker_info->tf_to_previous.getOrigin();
        marker_quaternion = marker_info->tf_to_previous.getRotation();

        // If plane type selected roll, pitch and Z axis are zero
        if(space_type_ == "plane")
        {
          double roll, pitch, yaw;
          tf::Matrix3x3(marker_quaternion).getRPY(roll,pitch,yaw);
          roll = 0;
          pitch = 0;
          marker_origin.setZ(0);
          marker_quaternion.setRPY(pitch,roll,yaw);
        }

        marker_info->tf_to_previous.setRotation(marker_quaternion);
        marker_info->tf_to_previous.setOrigin(marker_origin);

        marker_origin = marker_info->tf_to_previous.getOrigin();
        marker_info->geometry_msg_to_previous.position.x = marker_origin.getX();
        marker_info->geometry_msg_to_previous.position.y = marker_origin.getY();
        marker_info->geometry_msg_to_previous.position.z = marker_origin.getZ();

        marker_quaternion = marker_info->tf_to_previous.getRotation();
        marker_info->geometry_msg_to_previous.orientation.x = marker_quaternion.getX();
        marker_info->geometry_msg_to_previous.orientation.y = marker_quaternion.getY();
        marker_info->geometry_msg_to_previous.orientation.z = marker_quaternion.getZ();
        marker_info->geometry_msg_to_previous.orientation.w = marker_quaternion.getW();

        // increase marker count
        marker_counter_++;

        // Invert and position of new marker to compute camera pose above it
        marker_info->current_camera_tf = marker_info->current_camera_tf.inverse();

        marker_origin = marker_info->current_camera_tf.getOrigin();
        marker_info->current_camera_pose.position.x = marker_origin.getX();
        marker_info->current_camera_pose.position.y = marker_origin.getY();
        marker_info->current_camera_pose.position.z = marker_origin.getZ();

        marker_quaternion = marker_info->current_camera_tf.getRotation();
        marker_info->current_camera_pose.orientation.x = marker_quaternion.getX();
        marker_info->current_camera_pose.orientation.y = marker_quaternion.getY();
        marker_info->current_camera_pose.orientation.z = marker_quaternion.getZ();
        marker_info->current_camera_pose.orientation.w = marker_quaternion.getW();

        // Publish all TFs and markers
        publishTfs(false);
      }
    }
     /*

    //------------------------------------------------------
    // Compute global position of new marker
    //------------------------------------------------------
    if((marker_counter_previous_ < marker_counter_) && (first_marker_detected_ == true))
    {
      // Publish all TF five times for listener
      for(char k = 0; k < 5; k++)
        publishTfs(false);

      std::stringstream marker_tf_name;
      marker_tf_name << "marker_" << marker_info->marker_id;

      listener_->waitForTransform("base_marker",marker_tf_name.str(),ros::Time(0),
                                  ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
      try
      {
        listener_->lookupTransform("base_marker",marker_tf_name.str(),ros::Time(0),
                                   marker_info->tf_to_base_marker);
      }
      catch(tf::TransformException &e)
      {
        ROS_ERROR("Not able to lookup transform");
      }

      // Saving TF to Pose
      const tf::Vector3 marker_origin = marker_info->tf_to_base_marker.getOrigin();
      marker_info->geometry_msg_to_base_marker.position.x = marker_origin.getX();
      marker_info->geometry_msg_to_base_marker.position.y = marker_origin.getY();
      marker_info->geometry_msg_to_base_marker.position.z = marker_origin.getZ();

      tf::Quaternion marker_quaternion=marker_info->tf_to_base_marker.getRotation();
      marker_info->geometry_msg_to_base_marker.orientation.x = marker_quaternion.getX();
      marker_info->geometry_msg_to_base_marker.orientation.y = marker_quaternion.getY();
      marker_info->geometry_msg_to_base_marker.orientation.z = marker_quaternion.getZ();
      marker_info->geometry_msg_to_base_marker.orientation.w = marker_quaternion.getW();
    }
  }

  //------------------------------------------------------
  // Compute which of visible markers is the closest to the camera
  //------------------------------------------------------
  bool any_markers_visible=false;
  int num_of_visible_markers=0;

  if(first_marker_detected_ == true)
  {
    double minimal_distance = INIT_MIN_SIZE_VALUE;
    for(int k = 0; k < num_of_markers_; k++)
    {
      double a,b,c,size;
      auto minfo = &markers_[k];
      // If marker is visible, distance is calculated
      if(minfo->visible==true)
      {
        a = minfo->current_camera_pose.position.x;
        b = minfo->current_camera_pose.position.y;
        c = minfo->current_camera_pose.position.z;
        size = std::sqrt((a * a) + (b * b) + (c * c));
        if(size < minimal_distance)
        {
          minimal_distance = size;
          closest_camera_id_ = minfo->marker_id;
        }

        any_markers_visible = true;
        num_of_visible_markers++;
      }
    }
  }

  //------------------------------------------------------
  // Publish all known markers
  //------------------------------------------------------
  if(first_marker_detected_ == true)
    publishTfs(true);

  //------------------------------------------------------
  // Compute global camera pose
  //------------------------------------------------------
  if((first_marker_detected_ == true) && (any_markers_visible == true))
  {
    std::stringstream closest_camera_tf_name;
    closest_camera_tf_name << "camera_" << closest_camera_id_;

    listener_->waitForTransform("base_marker",closest_camera_tf_name.str(),ros::Time(0),
                                ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
    try
    {
      listener_->lookupTransform("base_marker",closest_camera_tf_name.str(),ros::Time(0),
                                 base_marker_position_transform_);
    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("Not able to lookup transform");
    }

    // Saving TF to Pose
    const tf::Vector3 marker_origin = base_marker_position_transform_.getOrigin();
    base_marker_position_geometry_msg_.position.x = marker_origin.getX();
    base_marker_position_geometry_msg_.position.y = marker_origin.getY();
    base_marker_position_geometry_msg_.position.z = marker_origin.getZ();

    tf::Quaternion marker_quaternion = base_marker_position_transform_.getRotation();
    base_marker_position_geometry_msg_.orientation.x = marker_quaternion.getX();
    base_marker_position_geometry_msg_.orientation.y = marker_quaternion.getY();
    base_marker_position_geometry_msg_.orientation.z = marker_quaternion.getZ();
    base_marker_position_geometry_msg_.orientation.w = marker_quaternion.getW();
  }

  //------------------------------------------------------
  // Publish all known markers
  //------------------------------------------------------
  if(first_marker_detected_ == true)
    publishTfs(true);

  //------------------------------------------------------
  // Publish custom marker message
  //------------------------------------------------------
  aruco_mapping::ArucoMarker marker_msg;

  if((any_markers_visible == true))
  {
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = "base_marker";
    marker_msg.marker_visibile = true;
    marker_msg.num_of_visible_markers = num_of_visible_markers;
    marker_msg.global_camera_pose = base_marker_position_geometry_msg_;
    marker_msg.marker_ids.clear();
    marker_msg.global_marker_poses.clear();
    for(size_t j = 0; j < marker_counter_; j++)
    {
      if(markers_[j].visible == true)
      {
        marker_msg.marker_ids.push_back(markers_[j].marker_id);
        marker_msg.global_marker_poses.push_back(markers_[j].geometry_msg_to_base_marker);
      }
    }
  }
  else
  {
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = "base_marker";
    marker_msg.num_of_visible_markers = num_of_visible_markers;
    marker_msg.marker_visibile = false;
    marker_msg.marker_ids.clear();
    marker_msg.global_marker_poses.clear();
  }

  // Publish custom marker msg
  marker_msg_pub_.publish(marker_msg);
*/
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
ArucoMapping::publishTfs(bool base_marker_option)
{
  for(int i = 0; i < marker_counter_; i++)
  {
    auto minfo = &markers_[i];
    // Actual Marker
    std::stringstream marker_tf_id;
    marker_tf_id << "marker_" << minfo->marker_id;
    // Older marker - or base_marker
    std::stringstream marker_tf_id_old;
    if(i == 0)
      marker_tf_id_old << "base_marker";
    else
      marker_tf_id_old << "marker_" << minfo->previous_marker_id;
    broadcaster_.sendTransform(tf::StampedTransform(minfo->tf_to_previous,ros::Time::now(),marker_tf_id_old.str(),marker_tf_id.str()));

    // Position of camera to its marker
    std::stringstream camera_tf_id;
    camera_tf_id << "camera_" << minfo->marker_id;
    broadcaster_.sendTransform(tf::StampedTransform(minfo->current_camera_tf,ros::Time::now(),marker_tf_id.str(),camera_tf_id.str()));

    if(base_marker_option == true)
    {
      // Global position of marker TF
      std::stringstream marker_globe;
      marker_globe << "marker_globe_" << minfo->marker_id;
      broadcaster_.sendTransform(tf::StampedTransform(minfo->tf_to_base_marker,ros::Time::now(),"base_marker",marker_globe.str()));
    }

    // Cubes for RVIZ - markers
    publishMarker(minfo->geometry_msg_to_previous,minfo->marker_id,i);
  }

  // Global Position of object
  if(base_marker_option == true)
    broadcaster_.sendTransform(tf::StampedTransform(base_marker_position_transform_,ros::Time::now(),"base_marker","camera_position"));
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
ArucoMapping::publishMarker(geometry_msgs::Pose marker_pose, int marker_id, int index)
{
  visualization_msgs::Marker vis_marker;

  if(index == 0)
    vis_marker.header.frame_id = "base_marker";
  else
  {
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

tf::Transform
ArucoMapping::arucoMarker2Tf(const aruco::Marker &marker)
{
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Mat Rvec64;
  marker.Rvec.convertTo(Rvec64, CV_64FC1);
  cv::Rodrigues(Rvec64, rot);
  cv::Mat tran64;
  marker.Tvec.convertTo(tran64, CV_64FC1);

  tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                       rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                       rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

  tf::Vector3 tf_orig(tran64.at<double>(0,0), tran64.at<double>(1,0), tran64.at<double>(2,0));


  return tf::Transform(tf_rot, tf_orig);
}



}  //aruco_mapping

#endif  //ARUCO_MAPPING_CPP
