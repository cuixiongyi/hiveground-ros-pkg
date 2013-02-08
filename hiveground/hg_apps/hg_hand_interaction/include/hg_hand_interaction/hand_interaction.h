/*
 * hand_interaction.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: mahisorn
 */

#include <ros/ros.h>

#include "ui_hand_interaction.h"
#include <hg_hand_interaction/HandGesture.h>
#include <hg_object_tracking/Hands.h>
#include <opencv2/opencv.hpp>
#include <hg_hand_interaction/gesture.h>

namespace hg_hand_interaction
{

class KalmanTraker3d
{
public:
  enum
  {
    ERROR = -1,
    START = 0,
    TRACK,
    LOST,
    DIE
  };

  enum
  {
    MESUREMENT = 0,
    PREDICTED,
    ESTIMATED
  };


  KalmanTraker3d();
  ~KalmanTraker3d();

  void initialize(double dt,
                    const cv::Point3f& point,
                    double process_noise = 1e-4,
                    double measurement_noise = 1e-1,
                    double error_cov = 1e-1);
  void predict(cv::Mat& result);
  void update(const cv::Point3f& measurement, cv::Mat& result);
  int updateState();
  int getState() { return current_state_; }

protected:
  cv::KalmanFilter filter_;
  int predict_count_;
  int update_count_;
  int current_state_;
};

class HandInteraction : public QMainWindow
{
  Q_OBJECT
  static const int HAND_HISTORY_SIZE = 30;
public:

  HandInteraction(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~HandInteraction();

  bool initialize();

protected:
  void handsCallBack(const hg_object_tracking::HandsConstPtr message);

public:
  Ui::HandInteraction ui;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  bool quit_threads_;

  ros::Subscriber hands_subscriber_;
  ros::Publisher hand_filtered_publisher_;
  ros::Publisher hand_gestures_publisher_;
  ros::Publisher marker_array_publisher_;

  std::vector<std::vector<std::list<geometry_msgs::Point> > > hand_history_;
  std::vector<KalmanTraker3d> hand_trackers_;
  std::vector<GestureDetector*> gesture_detectors_;
};

}
