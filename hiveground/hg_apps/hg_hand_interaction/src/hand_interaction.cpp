/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
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
#include <QtGui/QApplication>
#include <QDebug>
#include <boost/thread.hpp>
#include <hg_hand_interaction/hand_interaction.h>
#include <hg_hand_interaction/gesture_hand_sweep.h>
#include <hg_hand_interaction/gesture_push_pull.h>

using namespace hg_hand_interaction;
using namespace visualization_msgs;

KalmanTraker3d::KalmanTraker3d()
  : predict_count_(0), update_count_(0),
    current_state_(START)
{

}

KalmanTraker3d::~KalmanTraker3d()
{

}

void KalmanTraker3d::initialize(double dt,
                                    const cv::Point3f& point,
                                    double process_noise,
                                    double measurement_noise,
                                    double error_cov)
{
  filter_ = cv::KalmanFilter(6, 3, 0);
  filter_.statePost.at<float>(0) = point.x;
  filter_.statePost.at<float>(1) = point.y;
  filter_.statePost.at<float>(2) = point.z;
  filter_.statePost.at<float>(3) = 0.0;
  filter_.statePost.at<float>(4) = 0.0;
  filter_.statePost.at<float>(5) = 0.0;
  filter_.transitionMatrix = (cv::Mat_<float>(6, 6) <<
      1, 0, 0, dt, 0, 0,
      0, 1, 0, 0, dt, 0,
      0, 0, 1, 0, 0, dt,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1);
  cv::setIdentity(filter_.measurementMatrix);
  cv::setIdentity(filter_.processNoiseCov, cv::Scalar::all(process_noise));
  cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar::all(measurement_noise));
  cv::setIdentity(filter_.errorCovPost, cv::Scalar::all(error_cov));
  predict_count_ = 0;
  update_count_ = 0;
  current_state_ = START;
}

void KalmanTraker3d::predict(cv::Mat& result)
{
  result = filter_.predict();
  predict_count_++;
}

void KalmanTraker3d::update(const cv::Point3f& measurement, cv::Mat& result)
{
  cv::Mat_<float> z(3,1);
  z.setTo(cv::Scalar(0));
  z(0) = measurement.x;
  z(1) = measurement.y;
  z(2) = measurement.z;
  result = filter_.correct(z);
  update_count_++;
}

int KalmanTraker3d::updateState()
{
  switch(current_state_)
  {
    case START:
      if(predict_count_ > update_count_)
      {
        current_state_ = DIE;
      }
      else if(predict_count_ == update_count_)
      {
        if(update_count_ > 5)
        {
          current_state_ = TRACK;
        }
      }
      else
      {
        ROS_ERROR("Should not happen! call predict before update!");
      }
      break;
    case TRACK:
      if(predict_count_ > update_count_)
      {
        current_state_ = LOST;
      }
      else if(predict_count_ == update_count_)
      {
          current_state_ = TRACK;
      }
      else
      {
        ROS_ERROR("Should not happen! call predict before update!");
      }
      break;
    case LOST:
      if((predict_count_ - update_count_) > 30)
      {
        predict_count_ = 0;
        update_count_ = 0;
        current_state_ = DIE;
      }
      break;
    case DIE:
      if(update_count_ > 0)
      {
        current_state_ = START;
      }
      break;
  }
  return current_state_;
}

HandInteraction::HandInteraction(QWidget *parent, Qt::WFlags flags) :
  QMainWindow(parent, flags),
  nh_(), nh_private_("~"),
  quit_threads_(false)
{
  ui.setupUi(this);

  ui.toolBox->removeItem(ui.toolBox->currentIndex());
}

HandInteraction::~HandInteraction()
{
  if(gesture_detectors_.size() != 0)
  {
    for(size_t i = 0; i < gesture_detectors_.size(); i++)
    {
      if(gesture_detectors_[i])
        delete gesture_detectors_[i];
    }
  }
}

bool HandInteraction::initialize()
{
  gesture_detectors_.push_back(new SweepHandGestureDetector(nh_private_));
  gesture_detectors_.push_back(new PushPullHandGestureDetector(nh_private_));

  //UI for each detector if any
  for(size_t i = 0; i < gesture_detectors_.size(); i++)
  {
    gesture_detectors_[i]->addUI(ui.toolBox);
    gesture_detectors_[i]->initialize();

  }


  hands_subscriber_ = nh_private_.subscribe("hands_in", 1, &HandInteraction::handsCallBack, this);
  hand_filtered_publisher_ = nh_private_.advertise<hg_object_tracking::Hands>("filtered_hands", 128);
  hand_gestures_publisher_ = nh_private_.advertise<HandGestures>("hand_gestures", 128);
  marker_array_publisher_ = nh_private_.advertise<MarkerArray>("gesture_history", 128);
  return true;
}


void HandInteraction::handsCallBack(const hg_object_tracking::HandsConstPtr message)
{

  hg_object_tracking::HandsPtr hands(new hg_object_tracking::Hands());
  *hands = *message;
  size_t n_hand = hands->hands.size();

  if ((n_hand != 0) &&  (n_hand != hand_trackers_.size()))
  {
    //ROS_INFO("Reset!");
    hand_history_.clear();
    hand_trackers_.clear();
    hand_history_.resize(n_hand);
    for (size_t i = 0; i < hands->hands.size(); i++)
    {
      KalmanTraker3d traker;
      cv::Point3f point(message->hands[i].hand_centroid.translation.x,
                        message->hands[i].hand_centroid.translation.y,
                        message->hands[i].hand_centroid.translation.z);
      traker.initialize(1.0 / 30.0, point, 1e-2, 1e-1, 1e-1);
      hand_trackers_.push_back(traker);
      hand_history_[i].resize(3);
    }
  }


  geometry_msgs::Point marker_point;
  cv::Point3f measurement;
  cv::Mat result;
  int state;
  for (size_t i = 0; i < hand_trackers_.size(); i++)
  {
    if (i + 1 <= n_hand)
    {
      hand_trackers_[i].predict(result);
      //ROS_INFO_STREAM("[" << i << "] predicted: " << result);
      marker_point.x = result.at<float>(0);
      marker_point.y = result.at<float>(1);
      marker_point.z = result.at<float>(2);
      if (ui.checkBoxShowHandPredicted->isChecked())
        hand_history_[i][KalmanTraker3d::PREDICTED].push_back(marker_point);
      if (hand_history_[i][KalmanTraker3d::PREDICTED].size() > HAND_HISTORY_SIZE)
      {
        hand_history_[i][KalmanTraker3d::PREDICTED].pop_front();
      }

      tf::Vector3 point1(result.at<float>(0), result.at<float>(1), result.at<float>(2));
      int index = closestHand(point1, message);
      state = hand_trackers_[i].getState();
      switch (state)
      {
        case KalmanTraker3d::START:
          //ROS_INFO("%d START", i);
          measurement.x = message->hands[index].hand_centroid.translation.x;
          measurement.y = message->hands[index].hand_centroid.translation.y;
          measurement.z = message->hands[index].hand_centroid.translation.z;
          break;
        case KalmanTraker3d::TRACK:
        case KalmanTraker3d::LOST:
        {
          //ROS_INFO("%d TRACK", i);
          measurement.x = message->hands[index].hand_centroid.translation.x;
          measurement.y = message->hands[index].hand_centroid.translation.y;
          measurement.z = message->hands[index].hand_centroid.translation.z;
          break;
        }
        case KalmanTraker3d::DIE:
          //ROS_INFO("%d DIE", i);
          cv::Point3f point(message->hands[index].hand_centroid.translation.x,
                            message->hands[index].hand_centroid.translation.y,
                            message->hands[index].hand_centroid.translation.z);
          //ROS_INFO_STREAM("[" << i << "] point: " << point);
          hand_trackers_[i].initialize(1.0 / 30.0, point, 1e-2, 1e-1, 1e-1);
          break;
      }

      if ((state == KalmanTraker3d::TRACK) || (state == KalmanTraker3d::START))
      {
        hand_trackers_[i].update(measurement, result);
        marker_point.x = measurement.x;
        marker_point.y = measurement.y;
        marker_point.z = measurement.z;
        if(ui.checkBoxShowHandRaw->isChecked())
          hand_history_[i][KalmanTraker3d::MESUREMENT].push_back(marker_point);
        marker_point.x = result.at<float>(0);
        marker_point.y = result.at<float>(1);
        marker_point.z = result.at<float>(2);
        if(ui.checkBoxShowHandEstimated->isChecked())
          hand_history_[i][KalmanTraker3d::ESTIMATED].push_back(marker_point);

        //ROS_INFO_STREAM("[" << i << "] measurement: " << measurement);
        //ROS_INFO_STREAM("[" << i << "] estimated: " << result);

        if (hand_history_[i][KalmanTraker3d::MESUREMENT].size() > HAND_HISTORY_SIZE)
        {
          hand_history_[i][KalmanTraker3d::MESUREMENT].pop_front();
        }

        if (hand_history_[i][KalmanTraker3d::ESTIMATED].size() > HAND_HISTORY_SIZE)
        {
          hand_history_[i][KalmanTraker3d::ESTIMATED].pop_front();
        }

        hands->hands[i].hand_centroid.translation.x = result.at<float>(0);
        hands->hands[i].hand_centroid.translation.y = result.at<float>(1);
        hands->hands[i].hand_centroid.translation.z = result.at<float>(2);
      }
    }

    state = hand_trackers_[i].updateState();
    if (state == KalmanTraker3d::DIE)
    {
      for (int j = 0; j < 3; j++)
      {
        hand_history_[i][j].clear();
      }
    }
  }




  MarkerArray marker_array;

  if(ui.checkBoxShowHistory->isChecked())
  {
    Marker marker;
    marker.type = Marker::LINE_STRIP;
    marker.lifetime = ros::Duration(0.1);
    marker.header.frame_id = hands->header.frame_id;
    marker.scale.x = 0.005;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    for (size_t i = 0; i < hand_history_.size(); i++)
      for (int j = 0; j < 3; j++)
      {
        marker.points.clear();
        if (hand_history_[i][j].size() != 0)
        {
          std::stringstream ss;
          ss << "point_history" << i << j;
          marker.ns = ss.str();
          marker.id = 0;

          std::list<geometry_msgs::Point>::iterator it;
          for (it = hand_history_[i][j].begin(); it != hand_history_[i][j].end(); it++)
          {
            marker.points.push_back(*it);
          }
          switch (j)
          {
            case 0:
              marker.color.r = 1;
              marker.color.g = 0;
              marker.color.b = 0;
              break;
            case 1:
              marker.color.r = 1;
              marker.color.g = 0;
              marker.color.b = 1;
              break;
            case 2:
              marker.color.r = 0;
              marker.color.g = 1;
              marker.color.b = 0;
              break;
            default:
              marker.color.r = 0;
              marker.color.g = 0;
              marker.color.b = 0;
              break;

          }
          marker.color.a = 1.0;
          marker_array.markers.push_back(marker);
        }
      }
  }

  HandGesture gesture;
  HandGestures gestures;

  HandGestureDetector* hand_gesture_detector;
  gestures.header.stamp = hands->header.stamp;
  for (size_t i = 0; i < gesture_detectors_.size(); i++)
  {
    hand_gesture_detector = dynamic_cast<HandGestureDetector*>(gesture_detectors_[i]);
    if (hand_gesture_detector)
    {
      hand_gesture_detector->addHandMessage(hands);
      gesture.type = hand_gesture_detector->lookForGesture();
      if (gesture.type != HandGesture::NOT_DETECTED)
        gestures.gestures.push_back(gesture);
    }

    if (ui.checkBoxShowHistory->isChecked())
    {
      gesture_detectors_[i]->drawHistory(marker_array, hands->header.frame_id);
    }

    if (ui.checkBoxShowResults->isChecked())
    {
      gesture_detectors_[i]->drawResult(marker_array, hands->header.frame_id);
    }
  }

  if(hand_filtered_publisher_.getNumSubscribers() != 0 && n_hand != 0)
  {
    hand_filtered_publisher_.publish(*hands);
  }

  if((marker_array_publisher_.getNumSubscribers() != 0) && !marker_array.markers.empty())
  {
    marker_array_publisher_.publish(marker_array);
  }

  if(hand_gestures_publisher_.getNumSubscribers() != 0 && !gestures.gestures.empty())
  {
    hand_gestures_publisher_.publish(gestures);
  }
}


int HandInteraction::closestHand(const tf::Vector3& point, const hg_object_tracking::HandsConstPtr message)
{
  double dist, min_distant = 1e6;
  size_t index = 0;
  for (size_t i = 0; i < message->hands.size(); i++)
  {
    tf::Vector3 point2(message->hands[i].hand_centroid.translation.x,
                       message->hands[i].hand_centroid.translation.y,
                       message->hands[i].hand_centroid.translation.z);
    dist = point.distance(point2);
    if (dist < min_distant)
    {
      min_distant = dist;
      index = i;
    }
  }
  return index;
}

HandInteraction* g_hand_interaction = NULL;
bool g_initialized = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_hand_interaction->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_hand_tracking", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);



  HandInteraction w;
  g_hand_interaction = &w;
  w.show();

  g_initialized = w.initialize();
  if(!g_initialized)
    exit(-1);

  int ret = a.exec();

  spin_thread.join();

  return ret;
}

