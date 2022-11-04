/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

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
 ********************************/
/**
 * @file aruco_reader.cpp
 * @author Riccardo Zuppetti
 * @date August 2022
 * @brief Modified copy of marker_publish.cpp to publish only the id of the markers
 * (modified by Riccardo Zuppetti, 2022)
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int64.h>

class ArucoMarkerPublisher
{
private:
  // ArUco stuff
  aruco::MarkerDetector mDetector1_;
  std::vector<aruco::Marker> markers1_;
  aruco::CameraParameters camParam1_;

  aruco::MarkerDetector mDetector2_;
  std::vector<aruco::Marker> markers2_;
  aruco::CameraParameters camParam2_;

  // node params
  double marker_size1_;
  bool useCamInfo1_;
  double marker_size2_;
  bool useCamInfo2_;

  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it1_;
  image_transport::Subscriber image_sub1_;
  image_transport::ImageTransport it2_;
  image_transport::Subscriber image_sub2_;

  ros::Publisher pub_id;
  cv::Mat inImage1_;
  cv::Mat inImage2_;
  
public:
  ArucoMarkerPublisher() :
      nh_("~"), it1_(nh_), useCamInfo1_(true),it2_(nh_), useCamInfo2_(true)
  {
    //two subscribers one for each camera of the robot
    image_sub1_ = it1_.subscribe("/robot/camera1/image_raw", 1, &ArucoMarkerPublisher::image_callback1, this);
    image_sub2_ = it2_.subscribe("/robot/camera2/image_raw", 1, &ArucoMarkerPublisher::image_callback2, this);

    //a single publisher for both the camera
    pub_id =nh_.advertise<std_msgs::Int64>("/id_aruco", 1000);
    nh_.param<bool>("use_camera_info", useCamInfo1_, false);
    nh_.param<bool>("use_camera_info_floor", useCamInfo2_, false);
    camParam1_ = aruco::CameraParameters();
    camParam2_ = aruco::CameraParameters();
  }
  
  //each camera has its own callback
  void image_callback1(const sensor_msgs::ImageConstPtr& msg)
  {

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage1_ = cv_ptr->image;
   
      // clear out previous detection results
      markers1_.clear();

      // ok, let's detect
      mDetector1_.detect(inImage1_, markers1_, camParam1_, marker_size1_, false);

	//publish all the detected id on the /aruco_id topic
        for (std::size_t i = 0; i < markers1_.size(); ++i)
        {
        
          std_msgs::Int64 msg;

    	  msg.data =  markers1_.at(i).id;

          pub_id.publish(msg);
        }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
void image_callback2(const sensor_msgs::ImageConstPtr& msg)
  {

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage2_ = cv_ptr->image;
   
      // clear out previous detection results
      markers2_.clear();

      // ok, let's detect
      mDetector2_.detect(inImage2_, markers2_, camParam2_, marker_size2_, false);

	//std::cout << "The id of the detected marker detected is: ";
        for (std::size_t i = 0; i < markers2_.size(); ++i)
        {
          std_msgs::Int64 msg;

    	  msg.data =  markers2_.at(i).id;

          pub_id.publish(msg);
        }
        
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
     

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");

  ArucoMarkerPublisher node;

  ros::spin();
}
