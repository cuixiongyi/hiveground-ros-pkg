#ifndef PRW_H
#define PRW_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>
#include <tf/transform_broadcaster.h>

//message
#include <std_msgs/Float64.h>

#include <actionlib_msgs/GoalStatus.h>

#include <sensor_msgs/JointState.h>


#include <kinematics_msgs/PositionIKRequest.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>

#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <arm_navigation_msgs/GetStateValidity.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/convert_messages.h>

//rviz
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

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

public:
  void sendPlanningScene();
  void sendMarkers();

  void callbackJointState(const sensor_msgs::JointState& message);

protected:
  void closeEvent(QCloseEvent *event);



public:
  Ui::PRW ui;
  ros::NodeHandle node_handle_; //!< ROS node handle?

  bool quit_threads_;

  //Simple IK
  ros::ServiceClient ik_client_;
  ros::ServiceClient ik_info_client_;
  tf::TransformListener tf_listener_;
  std::vector<double> joint_positions_;
  ros::Subscriber subscriber_joint_state_;
  ros::Publisher publisher_joints_[6];

  ros::ServiceClient set_planning_scene_diff_client_;
  ros::ServiceClient planner_service_client_;
  ros::ServiceClient trajectory_filter_service_client_;

  planning_environment::CollisionModels* cm_;
  planning_models::KinematicState* robot_state_;

  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher vis_marker_publisher_;


};

#endif
