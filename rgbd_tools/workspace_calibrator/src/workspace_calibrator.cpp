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
std::string MARKER_WINDOWS[4] =
{
  "marker0", "marker1", "marker2", "marker3"
};
std::string image_topic_;
int x_step_;
int y_step_;
double size_;
double x_gap_;
double y_gap_;

cv::Mat camera_matrix_;
cv::Mat camera_distortion_;
std::vector<cv::Point3f> marker_points_;
std::vector<cv::Point3f> object_points_;
int state_ = 0;
cv::Point markers_[4];

void onMouse( int event, int x, int y, int, void* )
{
  switch (event)
  {
    case CV_EVENT_LBUTTONDOWN:
    {
      //put a box;
      if(state_ < 4)
      {
        markers_[state_].x = x;
        markers_[state_].y = y;
        state_++;
      }

      break;
    }
    case CV_EVENT_MOUSEMOVE: break;

  }
}

void onKey(int key)
{
  //ROS_INFO_STREAM("get key: " << (char)key);
  switch(tolower((char)key))
  {
    case
      'r': state_ = 0;
      markers_[0] = markers_[1] = markers_[2] = markers_[3] = cv::Point();
    break;
  }
}

int getQuaternion(const cv::Mat_<double> &rvec, cv::Mat_<double> &quat) {
    cv::Mat_<double> R(3, 3);
    cv::Rodrigues(rvec, R);

    if ((quat.rows == 4) && (quat.cols == 1)) {
        //Mat size OK
    } else {
        quat = cv::Mat_<double>::eye(4,1);
    }
    double   w;

    w = R(0,0) + R(1,1)+ R(2,2) + 1;
    if ( w < 0.0 ) return 1;

    w = std::sqrt( w );
    quat(0,0) = (R(2,1) - R(1,2)) / (w*2.0);
    quat(1,0) = (R(0,2) - R(2,0)) / (w*2.0);
    quat(2,0) = (R(1,0) - R(0,1)) / (w*2.0);
    quat(3,0) = w / 2.0;
    return 0;
}

cv::Mat_<double> getQuaternion(const cv::Point3d &vec, float angle)
{
  float sinAngle;
  angle *= 0.5f;
  cv::Mat_<double> quat = cv::Mat_<double>::eye(4, 1);

  sinAngle = std::sin(angle);

  quat(0, 0) = (vec.x * sinAngle);
  quat(1, 0) = (vec.y * sinAngle);
  quat(2, 0) = (vec.z * sinAngle);
  quat(3, 0) = std::cos(angle);
  return quat;
}

void imageCallback(const sensor_msgs::ImageConstPtr& image)
{
  //ROS_INFO_STREAM_THROTTLE(1.0, image->encoding);

  int key = cv::waitKey(10);
  onKey(key);

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

  //cv::imshow("input image", bridge->image);

  cv::Mat input_image;
  cv::cvtColor(bridge->image, input_image, CV_GRAY2RGB);


  for(int i = 0; i < 4; i++)
  {
    cv::rectangle(input_image,
                  cv::Point(markers_[i].x - 50, markers_[i].y - 50),
                  cv::Point(markers_[i].x + 50, markers_[i].y + 50),
                  cv::Scalar(255, 128, 0));
    std::stringstream ss;
    ss << i;
    std::string text;
    ss >> text;
    cv::putText(input_image,
                text,
                cv::Point(markers_[i].x - 50, markers_[i].y - 50),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(0, 0, 255));
  }



  if(state_ == 4)
  {
    std::vector<cv::Point2f> all_blobs;

    for(int i = 0; i < 4; i++)
    {


      cv::Mat roi_image = cv::Mat(
          bridge->image,
          cv::Rect(markers_[i].x - 50, markers_[i].y - 50,
                   100, 100));
          //cv::Rect(10, 10, 100, 100));


      cv::Size pattern_size(x_step_, y_step_);
      std::vector<cv::Point2f> blobs;
      bool pattern_found = false;
      pattern_found = cv::findCirclesGrid(roi_image, pattern_size, blobs, cv::CALIB_CB_ASYMMETRIC_GRID);

      if(pattern_found)
      {
        if(blobs.back().y < blobs.front().y)
        {
          std::reverse(blobs.begin(), blobs.end());
        }


        cv::Mat roi_image_draw;
        cv::cvtColor(roi_image, roi_image_draw, CV_GRAY2BGR);
        cv::drawChessboardCorners(roi_image_draw, pattern_size, cv::Mat(blobs), pattern_found);
        cv::imshow(MARKER_WINDOWS[i], roi_image_draw);

        for(std::vector<cv::Point2f>::iterator it = blobs.begin(); it != blobs.end(); it++)
        {
          it->x += (markers_[i].x - 50);
          it->y += (markers_[i].y - 50);
        }


        all_blobs.insert(all_blobs.end(), blobs.begin(), blobs.end());

/*
        if(i==0)
        {
          for(int idx = 0; idx < blobs.size(); idx++)
          {
            ROS_INFO_STREAM(blobs[idx] << marker_points_[idx]);
          }


          cv::Mat rvec, tvec;
          cv::solvePnP(marker_points_, blobs, camera_matrix_, camera_distortion_, rvec, tvec, false);
          ROS_INFO_STREAM(rvec);
          ROS_INFO_STREAM(tvec);
        }
*/
      }
    }


    if(all_blobs.size() == object_points_.size())
    {
      ROS_INFO_THROTTLE(1.0, "got all blobs");
      cv::Mat_<double> rvec, tvec, q, rotation_matrix;
      cv::solvePnP(object_points_, all_blobs, camera_matrix_, camera_distortion_, rvec, tvec, false);
      ROS_INFO_STREAM_THROTTLE(1.0, tvec);
      ROS_INFO_STREAM_THROTTLE(1.0, rvec);

      //board -> camera
      Rodrigues(rvec, rotation_matrix);

      double* rptr = rotation_matrix.ptr<double>(0);
      double* tptr = tvec.ptr<double>(0);
      cv::Mat transformation_matrix =
          (cv::Mat_<double>(4, 4) <<
              rptr[0], rptr[1], rptr[2], tptr[0],
              rptr[3], rptr[4], rptr[5], tptr[1],
              rptr[6], rptr[7], rptr[8], tptr[2],
              0, 0, 0, 1);
      ROS_INFO_STREAM_THROTTLE(1.0, transformation_matrix);
      ROS_INFO_STREAM_THROTTLE(1.0, transformation_matrix.inv());




    }




  }






#if 0
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

#endif

  cv::imshow(WINDOW_NAME, input_image);


}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "workspace_calibrator");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");


  nh.param("x_step", x_step_, 3);
  nh.param("y_step", y_step_, 4);
  nh.param("size", size_, 0.08);
  nh.param("x_gap", x_gap_, 0.432 + (0.08*y_step_/2.0));
  nh.param("y_gap", y_gap_, 0.892 + (0.08*x_step_));
  ROS_INFO_STREAM("x step: " << x_step_);
  ROS_INFO_STREAM("y step: " << y_step_);
  ROS_INFO_STREAM("size: " << size_);
  ROS_INFO_STREAM("x gap: " << x_gap_);
  ROS_INFO_STREAM("y gap: " << y_gap_);


  n.param("image_topic", image_topic_, std::string("/camera/rgb/image_mono"));
  ROS_INFO_STREAM("image topic: " << image_topic_);


  ROS_INFO("read camera metrix");
  double camera_matrix[9] = {528.825665620759, 0, 313.756893528701, 0, 530.466279366652, 253.210556553393, 0, 0, 1};
  camera_matrix_ = cv::Mat(3,3, CV_64FC1, camera_matrix);
  double camera_distortion[5] = {0.151330528573397, -0.221085177820244, 0.0044050794554637, -0.0039290742571273, 0};
  camera_distortion_ = cv::Mat(1,5, CV_64FC1, camera_distortion);


  cv::Mat q1 = getQuaternion(cv::Point3d(1.0, 0, 0), M_PI);
  ROS_INFO_STREAM_THROTTLE(1.0, q1);

  /*
  for(int y = 0; y < y_step_; y++)
  {
    for(int x = 0; x < x_step_; x++)
    {
      if((y%2) == 0)
      {
        marker_points_.push_back(cv::Point3f(y*(size_/2.0), x*size_, 0));
      }
      else
      {
        marker_points_.push_back(cv::Point3f(y*(size_/2.0), (x*size_)+(size_/2.0), 0));
      }
      //ROS_INFO_STREAM(marker_points.back());
    }
    //ROS_INFO_STREAM("-----");
  }
  */

  for(int y = 0; y < y_step_; y++)
  {
    for(int x = 0; x < x_step_; x++)
    {
      if ((y % 2) == 0)
      {
        //marker_points_.push_back(cv::Point3f(-(x * size_), -(y * (size_ / 2.0)), 0.0));
        //marker_points_.push_back(cv::Point3f(x * size_, y * (size_ / 2.0), 0.0));
        marker_points_.push_back(cv::Point3f(x * size_, y * (size_ / 2.0), 0.0));
      }
      else
      {
        //marker_points_.push_back(cv::Point3f(-((x * size_) + (size_ / 2.0)), -(y * (size_ / 2.0)), 0.0));
        //marker_points_.push_back(cv::Point3f((x * size_) + (size_ / 2.0), y * (size_ / 2.0), 0.0));
        marker_points_.push_back(cv::Point3f((x * size_) + (size_ / 2.0), y * (size_ / 2.0), 0.0));
      }
      ROS_INFO_STREAM(marker_points_.back());
    }
    ROS_INFO_STREAM("-----");
  }


  //object_points_
  for(size_t i = 0; i < marker_points_.size(); i++)
  {
    cv::Point3f p(marker_points_[i]);
    p.x -= 0;
    p.y -= 0;
    object_points_.push_back(p);
    ROS_INFO_STREAM(object_points_.back());
    if((i+1)%3==0) ROS_INFO_STREAM("-----");
  }


  for(size_t i = 0; i < marker_points_.size(); i++)
  {
    cv::Point3f p(marker_points_[i]);
    p.x += 0;
    p.y += x_gap_;
    object_points_.push_back(p);
    ROS_INFO_STREAM(object_points_.back());
    if((i+1)%3==0) ROS_INFO_STREAM("-----");
  }


  for(size_t i = 0; i < marker_points_.size(); i++)
  {
    cv::Point3f p(marker_points_[i]);
    p.x += y_gap_;
    p.y += x_gap_;
    object_points_.push_back(p);
    ROS_INFO_STREAM(object_points_.back());
    if((i+1)%3==0) ROS_INFO_STREAM("-----");
  }

  for(size_t i = 0; i < marker_points_.size(); i++)
  {
    cv::Point3f p(marker_points_[i]);
    p.x += y_gap_;
    p.y += 0;
    object_points_.push_back(p);
    ROS_INFO_STREAM(object_points_.back());
    if((i+1)%3==0) ROS_INFO_STREAM("-----");
  }

  /*
  double tmp;
  for(size_t i = 0; i < object_points_.size(); i++)
  {
    tmp = object_points_[i].x;
    object_points_[i].x = object_points_[i].y;
    object_points_[i].y = tmp;
    ROS_INFO_STREAM(object_points_[i]);
  }
  */

  ROS_INFO_STREAM("total points: " << object_points_.size());




/*
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
*/

  cv::namedWindow(WINDOW_NAME);
  cv::setMouseCallback(WINDOW_NAME, onMouse, 0 );

  ROS_INFO("Subscribe to image topic");
  ros::Subscriber sub = n.subscribe(image_topic_, 3, &imageCallback);
  ros::spin();
}
