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


using namespace std;
static const int SEARCH_SIZE = 50;
static const char WINDOW_NAME[] = "Workspace Calibrator";
std::string MARKER_WINDOWS[4] =
{
  "marker0", "marker1", "marker2", "marker3"
};

string g_camera;
double g_width;
double g_height;
string g_source_frame;
string g_target_frame;
std::vector<cv::Point2f> g_markers;
tf::TransformBroadcaster* g_broadcaster;
int g_dx, g_dy, g_dz, g_rx, g_ry, g_rz;
int g_state;
ros::Subscriber g_image_sub;
ros::Subscriber g_camera_info_sub;
bool g_got_camera_info = false;
cv::Mat g_camera_matrix;
cv::Mat g_camera_distortion;
std::vector<cv::Point3f> g_object_points;



void onMouse( int event, int x, int y, int, void* )
{
  switch (event)
  {
    case CV_EVENT_LBUTTONDOWN:
    {
      //put a box;
      if(g_state < 4)
      {
        g_markers[g_state].x = x;
        g_markers[g_state].y = y;
        g_state++;
      }

      break;
    }
    case CV_EVENT_MOUSEMOVE: break;

  }
}

void onKey(int key)
{
  switch(tolower((char)key))
  {
    case
      'r': g_state = 0;
      g_markers[0] = g_markers[1] = g_markers[2] = g_markers[3] = cv::Point();
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

void onSlider(int value, void* data)
{

}

void infoCallBack(const sensor_msgs::CameraInfoConstPtr& info)
{
  if(!g_got_camera_info)
  {
    g_camera_distortion = cv::Mat(1,5, CV_64FC1, (double*)info->D.data()).clone();
    g_camera_matrix = cv::Mat(3,3, CV_64FC1, (double*)info->K.data()).clone();
    g_got_camera_info = true;
    cout << g_camera_distortion << endl;
    cout << g_camera_matrix << endl;
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

  cv::Mat input_image;
  cv::cvtColor(bridge->image, input_image, CV_GRAY2RGB);

  for (int i = 0; i < 4; i++)
  {
    cv::rectangle(input_image, cv::Point(g_markers[i].x - (SEARCH_SIZE / 2), g_markers[i].y - (SEARCH_SIZE / 2)),
                  cv::Point(g_markers[i].x + (SEARCH_SIZE / 2), g_markers[i].y + (SEARCH_SIZE / 2)),
                  cv::Scalar(255, 128, 0));
    std::stringstream ss;
    ss << i;
    std::string text;
    ss >> text;
    cv::putText(input_image, text, cv::Point(g_markers[i].x - (SEARCH_SIZE / 2), g_markers[i].y - (SEARCH_SIZE / 2)),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255));
  }

  if (g_state == 4)
  {
    cv::cornerSubPix(bridge->image, g_markers, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.01));
    cv::drawChessboardCorners(input_image, cv::Size(2,2) , g_markers, true);

    cv::Mat_<double> rvec, tvec;
    cv::solvePnP(g_object_points, g_markers, g_camera_matrix, g_camera_distortion, rvec, tvec, false);
    ROS_INFO_STREAM_THROTTLE(1.0, "tvec: " << tvec);
    ROS_INFO_STREAM_THROTTLE(1.0, "rvec: " << rvec);

    double* rptr = rvec.ptr<double>(0);
    double* tptr = tvec.ptr<double>(0);

    tf::Quaternion q;
    q.setRPY(rptr[0], rptr[1], rptr[2]);
    tf::Transform tf(q, tf::Vector3(tptr[0], tptr[1], tptr[2]));

    tf::Quaternion dq;
    double droll, dpitch, dyaw, dx, dy, dz;
    droll = ((g_rx - 500) / 500.0) * M_PI;
    dpitch = ((g_ry - 500) / 500.0) * M_PI;
    dyaw = ((g_rz - 500) / 500.0) * M_PI;
    dx = ((g_dx - 500) / 500.0) * g_width;
    dy = ((g_dy - 500) / 500.0) * g_height;
    dz = ((g_dz - 500) / 500.0);
    dq.setRPY(droll, dpitch, dyaw);

    //tf::Transform result(tf.getRotation() * dq, tf.getOrigin() + tf::Vector3(dx, dy, dz));
    tf::Transform result = tf * tf::Transform(dq, tf::Vector3(dx, dy, dz));
    tf::Transform result_inv = result.inverse();

    geometry_msgs::Transform tf1, tf2;
    tf::transformTFToMsg(result, tf1);
    tf::transformTFToMsg(result_inv, tf2);
    ROS_INFO_STREAM_THROTTLE(1.0, tf1);
    ROS_INFO_STREAM_THROTTLE(1.0, tf2);

    tf::StampedTransform stf(result_inv, ros::Time::now(), g_source_frame, g_target_frame);
    g_broadcaster->sendTransform(stf);

  }



#if 0

  if(!chessboard_)
  {
    for (int i = 0; i < 4; i++)
    {
      cv::rectangle(input_image, cv::Point(markers_[i].x - (SEARCH_SIZE/2), markers_[i].y - (SEARCH_SIZE/2)),
                    cv::Point(markers_[i].x + (SEARCH_SIZE/2), markers_[i].y + (SEARCH_SIZE/2)), cv::Scalar(255, 128, 0));
      std::stringstream ss;
      ss << i;
      std::string text;
      ss >> text;
      cv::putText(input_image, text, cv::Point(markers_[i].x - (SEARCH_SIZE/2), markers_[i].y - (SEARCH_SIZE/2)), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                  cv::Scalar(0, 0, 255));
    }

    if (state_ == 4)
    {
      std::vector<cv::Point2f> all_blobs;

      for (int i = 0; i < 4; i++)
      {

        cv::Mat roi_image = cv::Mat(bridge->image, cv::Rect(markers_[i].x - (SEARCH_SIZE/2), markers_[i].y - (SEARCH_SIZE/2), SEARCH_SIZE, SEARCH_SIZE));
        //cv::Rect(10, 10, 100, 100));

        cv::Size pattern_size(x_step_, y_step_);
        std::vector<cv::Point2f> blobs;
        bool pattern_found = false;
        pattern_found = cv::findCirclesGrid(roi_image, pattern_size, blobs, cv::CALIB_CB_ASYMMETRIC_GRID);

        if (pattern_found)
        {
          if (blobs.back().y < blobs.front().y)
          {
            std::reverse(blobs.begin(), blobs.end());
          }

          cv::Mat roi_image_draw;
          cv::cvtColor(roi_image, roi_image_draw, CV_GRAY2BGR);
          cv::drawChessboardCorners(roi_image_draw, pattern_size, cv::Mat(blobs), pattern_found);
          cv::imshow(MARKER_WINDOWS[i], roi_image_draw);

          for (std::vector<cv::Point2f>::iterator it = blobs.begin(); it != blobs.end(); it++)
          {
            it->x += (markers_[i].x - 50);
            it->y += (markers_[i].y - 50);
          }

          all_blobs.insert(all_blobs.end(), blobs.begin(), blobs.end());

        }
      }

      if ((all_blobs.size() == object_points_.size()) && got_camera_info_)
      {
        ROS_INFO_THROTTLE(1.0, "got all blobs");
        cv::Mat_<double> rvec, tvec;
        cv::solvePnP(object_points_, all_blobs, camera_matrix_, camera_distortion_, rvec, tvec, false);
        ROS_INFO_STREAM_THROTTLE(1.0, "tvec: " << tvec);
        ROS_INFO_STREAM_THROTTLE(1.0, "rvec: " << rvec);

        double* rptr = rvec.ptr<double>(0);
        double* tptr = tvec.ptr<double>(0);

        tf::Quaternion q;
        q.setRPY(rptr[0],rptr[1],rptr[2]);

        tf::Transform tf(q, tf::Vector3(tptr[0], tptr[1], tptr[2]));

        tf::Quaternion dq;
        double droll, dpitch, dyaw, dx, dy, dz;
        droll = ((rx_ - 500) / 500.0) * M_PI;
        dpitch = ((ry_ - 500) / 500.0) * M_PI;
        dyaw = ((rz_ - 500) / 500.0) * M_PI;
        dx = ((dx_ - 500) / 500.0);
        dy = ((dy_ - 500) / 500.0);
        dz = ((dz_ - 500) / 500.0);
        dq.setRPY(droll + M_PI, dpitch, dyaw);

        tf::Transform dtf(dq, tf::Vector3(dx, dy, dz));

        tf::Transform result = tf * dtf;
        tf::Transform result_inv = result.inverse();
        geometry_msgs::Transform tf1, tf2;
        tf::transformTFToMsg(result, tf1);
        tf::transformTFToMsg(result_inv, tf2);
        ROS_INFO_STREAM_THROTTLE(1.0,tf1);
        ROS_INFO_STREAM_THROTTLE(1.0, tf2);


        tf::StampedTransform stf(result, ros::Time::now(), source_frames_, target_frames_);
        broadcaster_->sendTransform(stf);




        /*
        tf::Quaternion quat;

        double* rptr = rvec.ptr<double>(0);

        quat.setRPY(rptr[0],rptr[1],rptr[2]);

        double* tptr = tvec.ptr<double>(0);

        tf::Vector3 vec(tptr[0], tptr[1], tptr[2]);

        tf::Transform tf(quat, vec);

        tf::Quaternion r1;
        r1.setRPY(M_PI, 0, M_PI/2);
        quat = quat * r1;

        ROS_INFO_STREAM_THROTTLE(1.0, "new quaternion: " << quat.getX() << " " << quat.getY() << " " << quat.getZ() << " " << quat.getW());
        */
      }//if all blob
    }//if state == 4
  }//if chessboard
  else
  {
    cv::Size pattern_size(x_step_, y_step_);
    std::vector<cv::Point2f> corners;
    bool pattern_found = false;

    pattern_found = cv::findChessboardCorners(
        bridge->image, pattern_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

    if (pattern_found && got_camera_info_)
    {
      cv::cornerSubPix(bridge->image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                       cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));


      cv::drawChessboardCorners(input_image, pattern_size, cv::Mat(corners), pattern_found);

      cv::circle(input_image, cv::Point(corners[0].x, corners[0].y), 10, cv::Scalar(255, 128, 0), 2);
      cv::circle(input_image, cv::Point(corners[1].x, corners[1].y), 10, cv::Scalar(128, 128, 0), 2);

      cv::Mat rvec, tvec;
      cv::solvePnP(object_points_, corners, camera_matrix_, camera_distortion_, rvec, tvec, false);

      ROS_INFO_STREAM_THROTTLE(1.0, rvec);
      ROS_INFO_STREAM_THROTTLE(1.0, tvec);

      cv::Mat rotation_matrix;
      cv::Rodrigues(rvec, rotation_matrix);
    }
  }


#endif


#if 0
  //find chessboardimage

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
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  nh_private.getParam("camera", g_camera);
  nh_private.getParam("width", g_width);
  nh_private.getParam("height", g_height);
  nh_private.getParam("source", g_source_frame);
  nh_private.getParam("target", g_target_frame);
  ROS_INFO("image: %s", g_camera.c_str());
  ROS_INFO("width: %f", g_width);
  ROS_INFO("height: %f", g_height);
  ROS_INFO("source: %s", g_source_frame.c_str());
  ROS_INFO("target: %s", g_target_frame.c_str());

  g_markers.resize(4);
  g_object_points.resize(4);
  g_object_points[0].x = 0.0;
  g_object_points[0].y = 0.0;
  g_object_points[0].z = 0.0;

  g_object_points[1].x = 0.0;
  g_object_points[1].y = g_height;
  g_object_points[1].z = 0.0;

  g_object_points[2].x = g_width;
  g_object_points[2].y = g_height;
  g_object_points[2].z = 0.0;

  g_object_points[3].x = g_width;
  g_object_points[3].y = 0.0;
  g_object_points[3].z = 0.0;

  g_dx = g_dy = g_dz = g_rx = g_ry = g_rz = 500;
  cv::namedWindow(WINDOW_NAME);
  cv::setMouseCallback(WINDOW_NAME, onMouse, 0);
  cv::createTrackbar("dx", WINDOW_NAME, &g_dx, 1000, onSlider, 0);
  cv::createTrackbar("dy", WINDOW_NAME, &g_dy, 1000, onSlider, 0);
  cv::createTrackbar("dz", WINDOW_NAME, &g_dz, 1000, onSlider, 0);
  cv::createTrackbar("rx", WINDOW_NAME, &g_rx, 1000, onSlider, 0);
  cv::createTrackbar("ry", WINDOW_NAME, &g_ry, 1000, onSlider, 0);
  cv::createTrackbar("rz", WINDOW_NAME, &g_rz, 1000, onSlider, 0);

  g_broadcaster = new tf::TransformBroadcaster();
  ros::Subscriber g_image_sub = nh.subscribe(g_camera + "/image_mono", 3, &imageCallback);
  ros::Subscriber g_camera_info_sub = nh.subscribe(g_camera + "/camera_info", 3, &infoCallBack);
  ros::spin();
}
