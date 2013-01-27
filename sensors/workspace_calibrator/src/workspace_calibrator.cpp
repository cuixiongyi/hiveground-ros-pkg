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
static const int SEARCH_SIZE = 150;
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
std::vector<cv::Point3f> marker_points_;
std::vector<cv::Point3f> object_points_;
int state_ = 0;
cv::Point markers_[4];

int dx_, dy_, dz_, rx_, ry_, rz_;

string source_frames_;
string target_frames_;
tf::TransformBroadcaster* broadcaster_;

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

  cv::Mat input_image;
  cv::cvtColor(bridge->image, input_image, CV_GRAY2RGB);


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





#if 0
  //find chessboard

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

  //ros::NodeHandle nh("~");

  nh_private.param("camera", camera_, string("/camera/rgb"));
  nh_private.param("chessboard", chessboard_, false);
  nh_private.param("x_step", x_step_, 3);
  nh_private.param("y_step", y_step_, 4);
  nh_private.param("size", size_, 0.08);
  nh_private.param("x_gap", x_gap_, 0.432);
  x_gap_ = x_gap_ + (size_*y_step_/2.0);
  nh_private.param("y_gap", y_gap_, 0.892);
  y_gap_ = y_gap_ + (size_*x_step_);


  ROS_INFO_STREAM("camera: " << camera_);
  ROS_INFO_STREAM("use chessboard: " << chessboard_);
  ROS_INFO_STREAM("x step: " << x_step_);
  ROS_INFO_STREAM("y step: " << y_step_);
  ROS_INFO_STREAM("size: " << size_);

  if(!chessboard_)
  {
    ROS_INFO_STREAM("x gap: " << x_gap_);
    ROS_INFO_STREAM("y gap: " << y_gap_);
  }


  nh_private.param("source_frame", source_frames_, string("camera_rgb_optical_frame"));
  nh_private.param("target_frame", target_frames_, string("world"));
  ROS_INFO_STREAM("source_frame: " << source_frames_);
  ROS_INFO_STREAM("source_frame: " << target_frames_);

  dx_ = dy_ = dz_ = rx_ = ry_ = rz_ = 500;

  cv::namedWindow(WINDOW_NAME);
  cv::setMouseCallback(WINDOW_NAME, onMouse, 0);
  cv::createTrackbar("dx", WINDOW_NAME, &dx_, 1000, onSlider, 0);
  cv::createTrackbar("dy", WINDOW_NAME, &dy_, 1000, onSlider, 0);
  cv::createTrackbar("dz", WINDOW_NAME, &dz_, 1000, onSlider, 0);
  cv::createTrackbar("rx", WINDOW_NAME, &rx_, 1000, onSlider, 0);
  cv::createTrackbar("ry", WINDOW_NAME, &ry_, 1000, onSlider, 0);
  cv::createTrackbar("rz", WINDOW_NAME, &rz_, 1000, onSlider, 0);

  broadcaster_ = new tf::TransformBroadcaster();
  ros::Subscriber sub_image = nh.subscribe(camera_ + "/image_mono", 3, &imageCallback);
  ros::Subscriber sub_info = nh.subscribe(camera_ + "/camera_info", 3, &infoCallBack);





  if(!chessboard_)
  {
    for(int y = 0; y < y_step_; y++)
    {
      for(int x = 0; x < x_step_; x++)
      {
        if ((y % 2) == 0)
        {
          marker_points_.push_back(cv::Point3f(x * size_, y * (size_ / 2.0), 0.0));
        }
        else
        {
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

    ROS_INFO_STREAM("total points: " << object_points_.size());
  }
  else
  {
    ROS_INFO("setup object point");
    for (int y = 0; y < y_step_; y++)
    {
      for (int x = 0; x < x_step_; x++)
      {
        object_points_.push_back(cv::Point3f((x * size_), y * size_, 0.0));
        ROS_INFO_STREAM(object_points_.back());
      }
    }
  }//chessboard check





  ros::spin();
}
