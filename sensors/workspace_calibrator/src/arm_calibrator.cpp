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
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>


using namespace std;

static const char WINDOW_NAME[] = "Workspace Calibrator";
std::string MARKER_WINDOWS[4] =
{
  "marker0", "marker1", "marker2", "marker3"
};

string camera_;
bool chessboard_;
int x_step_;
int y_step_;
double size_;
double x_gap_;
double y_gap_;

bool got_camera_info_ = false;

cv::Mat camera_matrix_;
cv::Mat camera_distortion_;
std::vector<cv::Point2f> corner_points_;
cv::Mat input_image_gray_;
cv::Mat input_image_rgb_;

string source_frames_;
string target_frames_;
tf::TransformBroadcaster* broadcaster_;

boost::mutex mutex_;

void onMouse( int event, int x, int y, int, void* )
{
  switch (event)
  {
    case CV_EVENT_LBUTTONDOWN:
    {
      //detect corner
      ROS_INFO("(%d, %d)", x, y);

      cv::TermCriteria criteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.001);


      mutex_.lock();
      corner_points_.clear();
      corner_points_.push_back(cv::Point2f(x, y));
      cv::cornerSubPix(input_image_gray_, corner_points_, cv::Size(5, 5), cv::Size(-1, -1), criteria);
      mutex_.unlock();
      for(int i = 0; i < corner_points_.size(); i++)
      {
        cout << corner_points_[i] << endl;
      }



      break;
    }
    case CV_EVENT_MOUSEMOVE: break;

  }
}

void onKey(int key)
{
  //ROS_INFO_STREAM("get key: " << (char)key);
  //switch(tolower((char)key))
  //{
    //case
      //'r': state_ = 0;
      //markers_[0] = markers_[1] = markers_[2] = markers_[3] = cv::Point();
    //break;
  //}
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

void onSlider(int value, void* data)
{

}

void infoCallBack(const sensor_msgs::CameraInfoConstPtr& info)
{
  if(!got_camera_info_)
  {
    camera_distortion_ = cv::Mat(1,5, CV_64FC1, (double*)info->D.data()).clone();
    camera_matrix_ = cv::Mat(3,3, CV_64FC1, (double*)info->K.data()).clone();
    //camera_distortion_ = cv::Mat(info->D;]
    got_camera_info_ = true;
    cout << camera_matrix_ << endl;
    cout << camera_distortion_ << endl;
  }
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

  mutex_.lock();
  input_image_gray_ = bridge->image.clone();
  cv::cvtColor(input_image_gray_, input_image_rgb_, CV_GRAY2RGB);

  for(int i = 0; i < corner_points_.size(); i++)
  {
    cv::circle(input_image_rgb_, corner_points_[i], 3, cv::Scalar(255, 128, 0));
  }



  cv::imshow(WINDOW_NAME, input_image_rgb_);
  mutex_.unlock();


}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "arm_calibrator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  //ros::NodeHandle nh("~");




  nh_private.param("camera", camera_, string("/camera"));
  ROS_INFO_STREAM("camera: " << camera_);



  nh_private.param("source_frames", source_frames_, camera_ + string("_rgb_optical_frame"));
  nh_private.param("target_frames", target_frames_, string("world"));
  ROS_INFO_STREAM("source_frames: " << source_frames_);
  ROS_INFO_STREAM("source_frames: " << target_frames_);


  cv::namedWindow(WINDOW_NAME);
  cv::setMouseCallback(WINDOW_NAME, onMouse, 0);

  broadcaster_ = new tf::TransformBroadcaster();

  ros::Subscriber sub_image = nh.subscribe(camera_ + "/rgb/image_mono", 3, &imageCallback);
  ros::Subscriber sub_info = nh.subscribe(camera_ + "/rgb/camera_info", 3, &infoCallBack);

  ros::spin();
}
