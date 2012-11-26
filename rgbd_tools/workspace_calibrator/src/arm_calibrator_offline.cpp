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
#include <fstream>

using namespace std;

static const char WINDOW_NAME[] = "Arm Calibrator";

char g_key = 0;
cv::Mat image;
cv::Mat image_gray;
int current_image = -1;
int last_image = -1;
std::vector<cv::Point2f> corner_points;
std::vector<std::string> files;
std::vector<cv::Point2f> marker_points;
std::vector<cv::Point3f> object_points;
cv::Mat camera_matrix;
cv::Mat camera_distortion;
tf::Transform camera_tf;
bool calibrated = false;
int dx_, dy_, dz_, rx_, ry_, rz_;
tf::TransformBroadcaster* broadcaster_;

void onMouse( int event, int x, int y, int, void* )
{
  switch (event)
  {
    case CV_EVENT_LBUTTONDOWN:
    {
      //detect corner
      printf("(%d, %d)\n", x, y);

      cv::TermCriteria criteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 100, 0.001);
      corner_points.clear();
      corner_points.push_back(cv::Point2f(x, y));
      cv::cornerSubPix(image_gray, corner_points, cv::Size(7, 7), cv::Size(-1, -1), criteria);
      //cout << criteria.maxCount << ":" << criteria.epsilon << endl;

      cv::circle(image, corner_points[0], 5, cv::Scalar(0, 128, 255));
      cv::imshow(WINDOW_NAME, image);
      marker_points[current_image] = corner_points[0];
      cout << files[current_image] << ":" << marker_points[current_image] << endl;
      break;
    }
    case CV_EVENT_MOUSEMOVE: break;

  }
}

void onSlider(int value, void* data)
{

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "arm_calibrator_offline");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  if(argc < 3)
  {
    cout << "Usage: " << endl;
    return -1;
  }

  for(int i = 0; i <  argc; i++)
  {
    cout << argv[i] << endl;
  }

  string dir(argv[1]);
  string filename =  dir + "/" + string(argv[2]);
  fstream filestr;
  filestr.open (filename.c_str(), fstream::in);

  if(!filestr.is_open())
  {
    cout << "cannot open " << filename << endl;
    return -1;
  }

  object_points.clear();

  while(true)
  {
    std::string file;
    filestr >> file;
    cv::Point3f position;
    filestr >> position.x;
    filestr >> position.y;
    filestr >> position.z;



    if(filestr.eof()) break;

    cout << file << " " << position << endl;

    files.push_back(file);
    object_points.push_back(position);

  }

  cout << "object_points.size(): " << object_points.size() << endl;
  marker_points.resize(object_points.size());

  dx_ = dy_ = dz_ = rx_ = ry_ = rz_ = 500;

  cv::namedWindow(WINDOW_NAME);
  cv::setMouseCallback(WINDOW_NAME, onMouse, 0);
  cv::createTrackbar("dx", WINDOW_NAME, &dx_, 1000, onSlider, 0);
  cv::createTrackbar("dy", WINDOW_NAME, &dy_, 1000, onSlider, 0);
  cv::createTrackbar("dz", WINDOW_NAME, &dz_, 1000, onSlider, 0);
  cv::createTrackbar("rx", WINDOW_NAME, &rx_, 1000, onSlider, 0);
  cv::createTrackbar("ry", WINDOW_NAME, &ry_, 1000, onSlider, 0);
  cv::createTrackbar("rz", WINDOW_NAME, &rz_, 1000, onSlider, 0);



  filestr.close();

  //543.040574181687, 0, 319.01064529742, 0, 543.307336591517, 253.523080951277, 0, 0, 1

  camera_matrix = cv::Mat(3,3, CV_64FC1);
  camera_matrix.at<double>(0, 0) = 543.040574181687;
  camera_matrix.at<double>(0, 1) = 0;
  camera_matrix.at<double>(0, 2) = 319.01064529742;

  camera_matrix.at<double>(1, 0) = 0;
  camera_matrix.at<double>(1, 1) = 543.307336591517;
  camera_matrix.at<double>(1, 2) = 253.523080951277;

  camera_matrix.at<double>(2, 0) = 0;
  camera_matrix.at<double>(2, 1) = 0;
  camera_matrix.at<double>(2, 2) = 1;

  camera_distortion = cv::Mat(1,5, CV_64FC1);

  cout << camera_matrix << endl;
  cout << camera_distortion << endl;

  broadcaster_ = new tf::TransformBroadcaster();

  while(ros::ok())
  {
    ros::spinOnce();
    g_key = tolower((char)cv::waitKey(10));
    if('q' == g_key)
      break;

    switch(g_key)
    {
      case ' ':
      {
        current_image = last_image + 1;
        if(current_image >= files.size())
          current_image = 0;
        image = cv::imread(dir + "/" + files[current_image]);
        last_image = current_image;
        break;
      }


      case 'p':
        current_image = last_image - 1;
        if(current_image < 0)
          current_image = files.size() - 1;
        image = cv::imread(dir + "/" + files[current_image]);
        last_image = current_image;


        break;
      case 'c':
      {
        cv::Mat_<double> rvec, tvec;
        //cv::solvePnP(object_points, marker_points, camera_matrix, camera_distortion, rvec, tvec, false);
        cv::solvePnP(object_points, marker_points, camera_matrix, camera_distortion, rvec, tvec, false);
        cout << rvec << endl;
        cout << tvec << endl;

        double* rptr = rvec.ptr<double>(0);
        double* tptr = tvec.ptr<double>(0);

        tf::Quaternion q;
        q.setRPY(rptr[0],rptr[1],rptr[2]);
        camera_tf = tf::Transform(q, tf::Vector3(tptr[0], tptr[1], tptr[2]));
        calibrated = true;

        printf("tf\t %f %f %f %f %f %f %f\n",
                          camera_tf.getOrigin().x(), camera_tf.getOrigin().y(), camera_tf.getOrigin().z(),
                          camera_tf.getRotation().x(), camera_tf.getRotation().y(), camera_tf.getRotation().z(), camera_tf.getRotation().w());

        /*
        tf::Quaternion dq;
        dq.setRPY(M_PI/2, 0, 0);
        tf::Transform dtf(dq, tf::Vector3(0, 0, 0));

        tf::Transform result = camera_tf * dtf;

        printf("tf\t %f %f %f %f %f %f %f\n",
                  camera_tf.getOrigin().x(), camera_tf.getOrigin().y(), camera_tf.getOrigin().z(),
                  camera_tf.getRotation().x(), camera_tf.getRotation().y(), camera_tf.getRotation().z(), camera_tf.getRotation().w());

        printf("result\t %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f\n",
                  result.getOrigin().x(), result.getOrigin().y(), result.getOrigin().z(),
                  result.getRotation().x(), result.getRotation().y(), result.getRotation().z(), result.getRotation().w());

        */
        break;
      }
      default:
      {
        if(calibrated)
        {
          tf::Quaternion dq;
          double droll, dpitch, dyaw, dx, dy, dz;
          droll = ((rx_ - 500) / 500.0) * M_PI;
          dpitch = ((ry_ - 500) / 500.0) * M_PI;
          dyaw = ((rz_ - 500) / 500.0) * M_PI;
          dx = ((dx_ - 500) / 500.0);
          dy = ((dy_ - 500) / 500.0);
          dz = ((dz_ - 500) / 500.0);
          //dq.setRPY(droll + M_PI, dpitch, dyaw + M_PI/2);
          dq.setRPY(droll, dpitch, dyaw);
          tf::Transform dtf1(dq, tf::Vector3(0, 0, 0));
          tf::Transform result = camera_tf * dtf1;
          tf::Transform dtf2(tf::Quaternion(0, 0, 0, 1), tf::Vector3(dx, dy, dz));
          result = result * dtf2;

          ROS_INFO_THROTTLE(1.0, "result\t %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f\n",
                            result.getOrigin().x(), result.getOrigin().y(), result.getOrigin().z(),
                            result.getRotation().x(), result.getRotation().y(), result.getRotation().z(), result.getRotation().w());


          tf::StampedTransform stf(result, ros::Time::now(),
                                   "kinect_center_rgb_optical_frame",
                                   "world");
          broadcaster_->sendTransform(stf);

        }
        break;
      }


    }

    if(image.empty()) continue;
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    cv::putText(image, files[last_image], cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0));
    cv::imshow(WINDOW_NAME, image);
  }

  if(ros::ok())
    ros::shutdown();


}

