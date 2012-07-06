#ifndef PRW_H
#define PRW_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/PositionIKRequest.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>

#include <boost/thread.hpp>

#include <QtGui>
#include "ui_prw.h"

class PRW : public QMainWindow
{
Q_OBJECT

public:
  PRW(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~PRW();
  void initialize();

public slots:
  void on_ik_move_go_clicked();
  void on_ik_move_reset_clicked();

protected:
  void closeEvent(QCloseEvent *event);


protected:
  void callbackJointState(const sensor_msgs::JointState& message);



public:
  Ui::PRW ui;
  ros::NodeHandle node_handle_; //!< ROS node handle?

  bool quit_threads_;

  ros::ServiceClient ik_client_;
  ros::ServiceClient ik_info_client_;
  tf::TransformListener tf_listener_;
  std::vector<double> joint_positions_;


  ros::Subscriber subscriber_joint_state_;
  ros::Publisher publisher_joints_[6];

};

#endif
