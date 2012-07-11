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
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <arm_navigation_msgs/GetPlanningScene.h>
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



class WorkspaceEditorParameters
{

};

typedef std::map<interactive_markers::MenuHandler::EntryHandle, std::string> MenuEntryMap;
typedef std::map<std::string, MenuEntryMap> MenuMap;
typedef std::map<std::string, interactive_markers::MenuHandler> MenuHandlerMap;

class WorkspaceEditor
{
public:
  /// Used to decide which kinematic state the user is controlling. The planner plans from start to end.
  enum IKControlType
  {
    StartPosition, EndPosition
  };

  /// May be for IK control, joint control, or collision objects.
  enum InteractiveMarkerType
  {
    EndEffectorControl, JointControl, CollisionObject
  };

  /// Contains data for selectable markers. Stored in a large map.
  struct SelectableMarker
  {
    /// IK control, joint control, or collision object.
    InteractiveMarkerType type_;

    /// Name of the menu marker.
    std::string name_;

    /// Name of the 6DOF marker
    std::string controlName_;

    /// Text above the control.
    std::string controlDescription_;
  };

  struct TrajectoryData
  {

    TrajectoryData()
    {
      state_ = NULL;
      reset();
    }

    void reset()
    {
      if (state_ != NULL)
      {
        delete state_;
        state_ = NULL;
      }
      has_joint_trajectory_ = false;
      play_joint_trajectory_ = false;
      show_joint_trajectory_ = false;
      current_trajectory_point_ = 0;
      trajectory_bad_point_ = 0;
      trajectory_error_code_.val = 0;
    }

    planning_models::KinematicState* state_;
    trajectory_msgs::JointTrajectory joint_trajectory_;
    unsigned int current_trajectory_point_;
    std_msgs::ColorRGBA color_;
    bool has_joint_trajectory_;
    bool play_joint_trajectory_;
    bool show_joint_trajectory_;
    arm_navigation_msgs::ArmNavigationErrorCodes trajectory_error_code_;unsigned int trajectory_bad_point_;
  };

  typedef std::map<std::string, TrajectoryData> TrajectoryDataMap;

  class PlanningGroupData
  {
  public:
    std::string name_;
    std::string ik_link_name_;
    ros::ServiceClient ik_collision_aware_client_;
    ros::ServiceClient ik_non_collision_aware_client_;
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > arm_controller_;
    planning_models::KinematicState* start_state_;
    planning_models::KinematicState* end_state_;
    std_msgs::ColorRGBA start_color_;
    std_msgs::ColorRGBA end_color_;
    TrajectoryDataMap trajectory_data_map_;
    bool good_ik_solution_;
    tf::Transform last_good_state_;
    std::vector<std::string> joint_names_;

    PlanningGroupData()
    {
      start_state_ = NULL;
      end_state_ = NULL;
      good_ik_solution_ = false;

      start_color_.a = 0.3;
      start_color_.r = 1.0;
      start_color_.g = 0.5;
      start_color_.b = 1.0;

      end_color_.a = 0.3;
      end_color_.r = 0.5;
      end_color_.g = 0.9;
      end_color_.b = 0.5;

      trajectory_data_map_["planner"].color_.a = .6;
      trajectory_data_map_["planner"].color_.r = 1.0;
      trajectory_data_map_["planner"].color_.g = 1.0;
      trajectory_data_map_["planner"].color_.b = 0.5;

      trajectory_data_map_["filter"].color_.a = .6;
      trajectory_data_map_["filter"].color_.r = 0.5;
      trajectory_data_map_["filter"].color_.g = 1.0;
      trajectory_data_map_["filter"].color_.b = 1.0;

    }

    ~PlanningGroupData()
    {
      reset();
    }

    void setState(IKControlType type, planning_models::KinematicState* state)
    {
      switch (type)
      {
        case StartPosition:
          if (start_state_ != NULL)
          {
            delete start_state_;
            start_state_ = NULL;
          }
          start_state_ = state;
          break;
        case EndPosition:
          if (end_state_ != NULL)
          {
            delete end_state_;
            end_state_ = NULL;
          }
          end_state_ = state;
          break;
      }
    }

    void reset()
    {
      for (TrajectoryDataMap::iterator it = trajectory_data_map_.begin(); it != trajectory_data_map_.end(); it++)
      {
        it->second.reset();
      }

      if (start_state_ != NULL)
      {
        delete start_state_;
        start_state_ = NULL;
      }
      if (end_state_ != NULL)
      {
        delete end_state_;
        end_state_ = NULL;
      }
    }

  };

  typedef std::map<std::string, PlanningGroupData> PlanningGroupDataMap;


protected:
  ros::NodeHandle nh_;
  boost::recursive_mutex mutex_;

  planning_environment::CollisionModels* cm_;
  planning_models::KinematicState* robot_state_;
  std::map<std::string, double> robot_state_joint_values_;
  arm_navigation_msgs::MotionPlanRequest last_motion_plan_request_;

  ros::Subscriber joint_state_subscriber_;
  ros::ServiceClient get_planning_scene_client_;
  ros::ServiceClient planner_client_;
  ros::ServiceClient trajectory_filter_client_;
  bool collision_aware_;
  bool constrain_rp_;
  bool is_joint_control_active_;
  bool is_ik_control_active_;




  PlanningGroupDataMap group_map_;
  std::string current_group_name_;

  /// Maps end effector link names to their previously recorded poses.
  std::map<std::string, geometry_msgs::Pose> last_ee_poses_;

  /// Maps selectable marker names to a struct containing their information.
  std::map<std::string, SelectableMarker> selectable_markers_;



  tf::TransformBroadcaster transform_broadcaster_;
  tf::TransformListener transform_listener_;

  ros::Publisher marker_publisher_;
  ros::Publisher marker_array_publisher_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  interactive_markers::MenuHandler::FeedbackCallback process_ik_controller_feedback_ptr_;
  interactive_markers::MenuHandler::FeedbackCallback process_menu_feedback_ptr_;

  /// Maps strings to menu handlers. This is used for convenience and extensibility.
  MenuHandlerMap menu_handler_map_;

  /// Maps MenuHandles to their names. Used to determine which menu entry is selected.
  MenuMap menu_entry_maps_;


public:
  WorkspaceEditor();
  WorkspaceEditor(const WorkspaceEditorParameters& parameter);


  void sendMarkers();



  ~WorkspaceEditor();

protected:
  void lockScene() { mutex_.lock(); };
  void unlockScene() { mutex_.unlock(); };

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);

  void sendPlanningScene();
  void refreshEnvironment();
  void deleteKinematicStates();

  void selectPlanningGroup(unsigned int entry);
  void moveEndEffectorMarkers(double vx, double vy, double vz, double vr, double vp, double vw, bool coll_aware = true);
  void setNewEndEffectorPosition(PlanningGroupData& gc, tf::Transform& cur, bool coll_aware);
  bool solveIKForEndEffectorPose(PlanningGroupData& gc, bool coll_aware = true, bool constrain_pitch_and_roll = false,
                                 double change_redundancy = 0.0);
  void determinePitchRollConstraintsGivenState(const PlanningGroupData& gc,
                                               const planning_models::KinematicState& state,
                                               arm_navigation_msgs::OrientationConstraint& goal_constraint,
                                               arm_navigation_msgs::OrientationConstraint& path_constraint) const;

  bool planToEndEffectorState(PlanningGroupData& gc, bool show = false, bool play = false);
  bool filterPlannerTrajectory(PlanningGroupData& gc, bool show = false, bool play = false);
  void moveThroughTrajectory(PlanningGroupData& gc, const std::string& source_name, int step);
  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                const control_msgs::FollowJointTrajectoryResultConstPtr& result);


  void makeTopLevelMenu();
  void makeIKControllerMarker(tf::Transform transform, std::string name, std::string description, bool selectable,
                              float scale = 1.0, bool publish = true);
  void processIKControllerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void processMenuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);




  PlanningGroupData* getPlanningGroup(unsigned int i)
  {
    unsigned int cmd = 0;
    for(PlanningGroupDataMap::iterator it = group_map_.begin(); it != group_map_.end(); it++)
    {
      if(cmd == i)
      {
        return &(it->second);
      }
      cmd++;
    }

    return NULL;
  }

  bool isGroupName(const std::string& name)
  {
    return group_map_.find(name) != group_map_.end();
  }

  bool selectableMarkerExists(std::string name)
  {
    return selectable_markers_.find(name) != selectable_markers_.end();
  }

  /////
  /// @brief Removes the menu marker given and replaces it with a 6DOF marker
  /// @param marker a reference to the selectablemarker struct.
  /// @param transform location to select the marker.
  /////
  void selectMarker(SelectableMarker& marker, tf::Transform transform);



  /////
  /// @brief Removes the 6DOF control of the given marker and replaces it with a menu.
  /// @param marker a reference to the selectablemarker struct.
  /// @param transform location of the marker when it is de-selected.
  /////
  void deselectMarker(SelectableMarker& marker, tf::Transform transform);


  geometry_msgs::Pose toGeometryPose(tf::Transform transform)
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

  tf::Transform toBulletTransform(geometry_msgs::Pose pose)
  {
    tf::Quaternion quat = tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Vector3 vec = tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
    return tf::Transform(quat, vec);
  }
};

}









#endif
