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
#include <hg_hand_interaction/gesture.h>

namespace hg_hand_interaction
{

class HandInteraction : public QMainWindow
{
  Q_OBJECT
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
  ros::Publisher hand_gesture_publisher_;
  ros::Publisher marker_array_publisher_;

  std::vector<GestureDetector*> gesture_detectors_;



};

}
