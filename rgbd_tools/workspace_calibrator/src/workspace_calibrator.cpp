/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

static const char WINDOW_NAME[] = "Workspace Calibrator";
int x_size_;
int y_size_;
double size_;
bool square_;
cv::Mat camera_matrix_;
cv::Mat camera_distortion_;
std::vector<cv::Point3f> object_points_;

void imageCallback(const sensor_msgs::ImageConstPtr& image)
{
  ROS_INFO_STREAM_THROTTLE(1.0, image->encoding);

   // convert to cv image
  cv_bridge::CvImagePtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(image, image->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform RGB image.");
    return;
  }

  //find chessboard
  cv::Size pattern_size(x_size_, y_size_);
  std::vector<cv::Point2f> corners;
  bool pattern_found = false;

  if(square_)
  {
    //CALIB_CB_FAST_CHECK saves a lot of time on images
    //that do not contain any chessboard corners
    pattern_found = cv::findChessboardCorners(
        bridge->image, pattern_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

    if(pattern_found)
      cv::cornerSubPix(bridge->image, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  }
  else
  {
    pattern_found = cv::findCirclesGrid(bridge->image, pattern_size, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
  }

  cv::Mat result;
  cv::cvtColor(bridge->image, result, CV_GRAY2RGB);


  if(pattern_found)
  {
    cv::drawChessboardCorners(result, pattern_size, cv::Mat(corners), pattern_found);

    cv::circle(result, cv::Point(corners[0].x, corners[0].y), 10, cv::Scalar(255, 128, 0), 2);
    cv::circle(result, cv::Point(corners[1].x, corners[1].y), 10, cv::Scalar(128, 128, 0), 2);

    cv::Mat rvec, tvec;
    cv::solvePnP(object_points_, corners, camera_matrix_, camera_distortion_, rvec, tvec, false);

    ROS_INFO_STREAM_THROTTLE(1.0, rvec);
    ROS_INFO_STREAM_THROTTLE(1.0, tvec);

    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

  }



  // display
  cv::imshow(WINDOW_NAME, result);

  cv::waitKey(3);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "workspace_calibrator");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  nh.param("x_size", x_size_, 2);
  nh.param("y_size", y_size_, 7);
  nh.param("size", size_, 0.10);
  nh.param("square", square_, false);

  ROS_INFO("read camera metrix");
  double camera_matrix[9] = {528.825665620759, 0, 313.756893528701, 0, 530.466279366652, 253.210556553393, 0, 0, 1};
  camera_matrix_ = cv::Mat(3,3, CV_64FC1, camera_matrix);
  double camera_distortion[5] = {0.151330528573397, -0.221085177820244, 0.0044050794554637, -0.0039290742571273, 0};
  camera_distortion_ = cv::Mat(1,5, CV_64FC1, camera_distortion);


  ROS_INFO("setup object point");
  if(!square_)
  {
    for(int y = 0; y < y_size_; y++)
    {
      for(int x = 0; x < x_size_; x++)
      {
        if((y%2) == 0)
        {
          object_points_.push_back(cv::Point3f(x*size_, y*(size_/2.0), 0.0));
        }
        else
        {
          object_points_.push_back(cv::Point3f((x*size_)+(size_/2.0), y*(size_/2.0), 0.0));
        }
        ROS_INFO_STREAM(object_points_.back());
      }
      ROS_INFO_STREAM("-----");
    }

  }
  else
  {
    for(int y = 0; y < y_size_; y++)
    {
      for(int x = 0; x < x_size_; x++)
      {
        object_points_.push_back(cv::Point3f((x*size_), y*size_, 0.0));
        ROS_INFO_STREAM(object_points_.back());
      }
    }
  }


  cv::namedWindow(WINDOW_NAME);

  ROS_INFO("Subscribe to image topic");
  ros::Subscriber sub = n.subscribe("/camera/rgb/image_mono", 3, &imageCallback);
  ros::spin();
}
