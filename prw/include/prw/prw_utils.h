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
 * A lot of copy & paste code from arm_navigation_experiment
 */


#ifndef PRW_UTILS_H
#define PRW_UTILS_H

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <planning_environment/models/collision_models.h>
#include <planning_environment/models/model_utils.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <arm_navigation_msgs/GetStateValidity.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/convert_messages.h>



#include <string>
#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

#define DEG2RAD(x) (((x)*M_PI)/180.0)

namespace hg
{

inline static geometry_msgs::Pose toGeometryPose(tf::Transform transform)
{
  geometry_msgs::Pose toReturn;
  toReturn.position.x = transform.getOrigin().x();
  toReturn.position.y = transform.getOrigin().y();
  toReturn.position.z = transform.getOrigin().z();
  toReturn.orientation.x = transform.getRotation().x();
  toReturn.orientation.y = transform.getRotation().y();
  toReturn.orientation.z = transform.getRotation().z();
  toReturn.orientation.w = transform.getRotation().w();
  return toReturn;
}

inline static tf::Transform toBulletTransform(geometry_msgs::Pose pose)
{
  tf::Quaternion quat =
    tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Vector3 vec = tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
  return tf::Transform(quat, vec);
}

/**
  @brief Convert a control error code into a string value
  @param error_code The input error code
  @return The resultant string message
*/
inline static std::string getResultErrorFromCode(int error_code)
{
  std::string result;
  if(error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
     result = "Success";
  else if(error_code == control_msgs::FollowJointTrajectoryResult::INVALID_GOAL)
     result = "Invalid Goal";
  else if(error_code == control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS)
     result = "Invalid Joints";
  else if(error_code == control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP)
     result = "Old header timestamp";
  else if(error_code == control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED)
     result = "Path tolerance violated";
  else if(error_code == control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED)
     result = "Goal tolerance violated";
  return result;
}

////
/// Enum PositionType
/// @brief Specifies either the start or end position
/// of a motion plan request.
////
enum PositionType
  {
    StartPosition, GoalPosition
  };

////
/// Enum RenderType
/// @brief Specifies how a set of links should be rendered.
/// CollisionMesh: Mesh resource in URDF file listed for testing collisions.
/// VisualMesh: Mesh resource in URDF file listed for visualization.
/// PaddingMesh: Wireframe mesh representing the link's configuration space padding.
/////
enum RenderType
  {
    CollisionMesh, VisualMesh, PaddingMesh
  };

////
/// Enum TrajectoryRenderType
/// @brief Specifies how a trajectory should be rendered.
///  Kinematic: Trajectories are rendered by iterating through the trajectory points (ignoring timestamps).
///  Temporal:  Trajectories that have valid timestamps, are rendered based on the timestamps.
////
enum TrajectoryRenderType
{
  Kinematic,
  Temporal,
};

/////
/// Struct SelectableObject
/// @brief Struct containing an interactive marker
/// for 6DOF control, and another for selection.
////
struct SelectableObject
{
  SelectableObject() {
    attach_ = false;
    detach_ = false;
    attached_collision_object_.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
    collision_object_.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  }

  bool attach_;
  bool detach_;
  arm_navigation_msgs::AttachedCollisionObject attached_collision_object_;
  arm_navigation_msgs::CollisionObject collision_object_;
  visualization_msgs::InteractiveMarker selection_marker_;
  visualization_msgs::InteractiveMarker control_marker_;
  std_msgs::ColorRGBA color_;

  std::string id_;
};

/////
/// Struct IKController
/// @brief Struct containing the start and end 6DOF controllers
/// for a specific motion plan request.
/////
struct IKController
{
  unsigned int motion_plan_id_;
  visualization_msgs::InteractiveMarker start_controller_;
  visualization_msgs::InteractiveMarker end_controller_;
};


class PlanningSceneData
{
public:
  std::string name_;
  unsigned int id_;
  std::string host_;
  ros::Time timestamp_;
  arm_navigation_msgs::PlanningScene planning_scene_;
  std::vector<std::string> pipeline_stages_;
  std::vector<arm_navigation_msgs::ArmNavigationErrorCodes> error_codes_;
  std::set<unsigned int> motion_plan_requests_;

};

class MotionPlanRequestData
{
public:
  std::string name_;
  unsigned int id_;
  std::string source_;
  unsigned int planning_scene_id_;
  std::string end_effector_link_;
  std::string group_name_;
  arm_navigation_msgs::MotionPlanRequest motion_plan_request_;
  bool is_start_editable_;
  bool is_goal_editable_;
  bool is_start_visible_;
  bool is_goal_visible_;
  bool should_refresh_colors_;
  bool has_refreshed_colors_;
  bool has_path_constraints_;
  bool has_good_goal_ik_solution_;
  bool has_good_start_ik_solution_;
  bool are_collisions_visible_;
  bool has_state_changed_;
  bool are_joint_controls_visible_;

  double roll_tolerance_;
  double pitch_tolerance_;
  double yaw_tolerance_;

  bool constrain_roll_;
  bool constrain_pitch_;
  bool constrain_yaw_;

  std_msgs::ColorRGBA start_color_;
  std_msgs::ColorRGBA goal_color_;
  std::set<unsigned int> trajectories_;
  planning_models::KinematicState* start_state_;
  planning_models::KinematicState* goal_state_;
  tf::Transform last_good_start_pose_;
  tf::Transform last_good_goal_pose_;
  visualization_msgs::MarkerArray collision_markers_;
  RenderType render_type_;
  unsigned int next_trajectory_id_;
};

class TrajectoryData
{
public:

  enum MarkerType {
    VISUAL,
    COLLISION,
    PADDED
  };

protected:
  std::string name_;
  unsigned int id_;
  std::string source_;
  std::string group_name_;
  unsigned int planning_scene_id_;
  unsigned int motion_plan_request_Id_;
  trajectory_msgs::JointTrajectory trajectory_;
  trajectory_msgs::JointTrajectory trajectory_error_;
  bool is_visible_;
  MarkerType marker_type_;
  bool is_playing_;
  ros::Time playback_start_time_;
  bool collisions_visible_;
  bool state_changed_;
  std_msgs::ColorRGBA color_;
  unsigned int current_trajectory_point_;
  unsigned int trajectory_bad_point_;
  planning_models::KinematicState* current_state_;
  ros::Duration duration_;
  bool should_refresh_colors_;
  bool has_refreshed_colors_;
  visualization_msgs::MarkerArray collision_markers_;
  RenderType render_type_;
  TrajectoryRenderType trajectory_render_type_;
  ros::Duration time_to_stop_;
};


typedef std::map<std::string, ros::ServiceClient> ServiceClientMap;
typedef std::map<std::string, boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > > ArmControllerMap;
typedef boost::shared_ptr<ros::ServiceClient> ServiceClientPtr;
typedef std::vector<std::string> StringVector;
typedef std::map<std::string, ros::Subscriber> SubscriberMap;

struct WorkspaceEditorParameters
{
  int number_of_arm_;
  StringVector arm_group_;
  StringVector arm_controller_;
  StringVector ik_name_;
  StringVector ik_link_;
  StringVector non_collision_ik_name_;

  std::string planner_service_name_;
  std::string trajectory_filter_service_name_;
  std::string set_planning_scene_diff_name_;
  std::string visualizer_topic_name_;
};


class WorkspaceEditor
{
protected:
  WorkspaceEditorParameters* parameters_;

  boost::recursive_mutex mutex_;
  ros::NodeHandle nh_;

  boost::shared_ptr<planning_environment::CollisionModels> cm_;
  planning_models::KinematicState* robot_state_;
  ros::Subscriber joint_state_subscriber_;

  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher vis_marker_publisher_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;



  ServiceClientMap collision_aware_ik_services_;
  ServiceClientMap non_collision_aware_ik_services_;
  ArmControllerMap arm_controller_map_;
  ros::ServiceClient set_planning_scene_diff_service_;
  ros::ServiceClient planning_service_;
  ros::ServiceClient trajectory_filter_service_;



  std::map<std::string, double> robot_state_joint_values_;
  tf::TransformBroadcaster transform_broadcaster_;
  tf::TransformListener transform_listener_;


  std::map<std::string, PlanningSceneData> planning_scene_map_;
  std::map<std::string, MotionPlanRequestData> motion_plan_map_;
  std::map<std::string, std::map<std::string, TrajectoryData> > trajectory_map_;
  std::map<std::string, SelectableObject>* selectable_objects_;
  std::map<std::string, IKController>* ik_controllers_;

  interactive_markers::MenuHandler::FeedbackCallback ik_control_feedback_ptr_;



public:
  WorkspaceEditor(WorkspaceEditorParameters* parameter);


  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);
  bool sendPlanningScene();
  void sendMarkers();
  void getTrajectoryMarkers(visualization_msgs::MarkerArray& arr);
  void getMotionPlanningMarkers(visualization_msgs::MarkerArray& arr);
  void createIKController(MotionPlanRequestData& data, PositionType type, bool rePose);
  void IKControllerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void createNewPlanningScene();
  void deleteCollisionObject(std::string& name);




  virtual ~WorkspaceEditor();
};




}









#endif
