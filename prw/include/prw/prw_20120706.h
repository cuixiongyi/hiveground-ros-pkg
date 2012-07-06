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

  struct StateTrajectoryDisplay
  {

    StateTrajectoryDisplay()
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
    trajectory_msgs::JointTrajectory joint_trajectory_;unsigned int current_trajectory_point_;
    std_msgs::ColorRGBA color_;
    bool has_joint_trajectory_;
    bool play_joint_trajectory_;
    bool show_joint_trajectory_;
    arm_navigation_msgs::ArmNavigationErrorCodes trajectory_error_code_;unsigned int trajectory_bad_point_;
  };

  struct GroupCollection
  {
    GroupCollection()
    {
      start_state_ = NULL;
      end_state_ = NULL;
      good_ik_solution_ = false;

      state_trajectory_display_map_["planner"].color_.a = .6;
      state_trajectory_display_map_["planner"].color_.r = 1.0;
      state_trajectory_display_map_["planner"].color_.g = 1.0;
      state_trajectory_display_map_["planner"].color_.b = 0.5;

      state_trajectory_display_map_["filter"].color_.a = .6;
      state_trajectory_display_map_["filter"].color_.r = 0.5;
      state_trajectory_display_map_["filter"].color_.g = 1.0;
      state_trajectory_display_map_["filter"].color_.b = 1.0;
    }

    ~GroupCollection()
    {
      reset();
    }

    void setState(IKControlType type, planning_models::KinematicState* state)
    {
      switch (type)
      {
        case PRW::StartPosition:
          if (start_state_ != NULL)
          {
            delete start_state_;
            start_state_ = NULL;
          }
          start_state_ = state;
          break;
        case PRW::EndPosition:
          if (end_state_ != NULL)
          {
            delete end_state_;
            end_state_ = NULL;
          }
          end_state_ = state;
          break;
      }
    }

    planning_models::KinematicState* getState(IKControlType type)
    {
      switch (type)
      {
        case PRW::StartPosition:
          return start_state_;
        case PRW::EndPosition:
          return end_state_;
      }

      return NULL;
    }

    void reset()
    {
      for (std::map<std::string, StateTrajectoryDisplay>::iterator it = state_trajectory_display_map_.begin();
          it != state_trajectory_display_map_.end(); it++)
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

    std::string name_;
    std::string ik_link_name_;
    ros::ServiceClient coll_aware_ik_service_;
    ros::ServiceClient non_coll_aware_ik_service_;
    bool good_ik_solution_;
    planning_models::KinematicState* start_state_;
    planning_models::KinematicState* end_state_;
    std::map<std::string, StateTrajectoryDisplay> state_trajectory_display_map_;
    std::vector<std::string> joint_names_;
    tf::Transform last_good_state_;
  };

  typedef std::map<std::string, planning_models::KinematicModel::GroupConfig> KinematicModelGroupConfigMap;
  typedef std::map<interactive_markers::MenuHandler::EntryHandle, std::string> MenuEntryMap;
  typedef std::map<std::string, MenuEntryMap> MenuMap;
  typedef std::map<std::string, interactive_markers::MenuHandler> MenuHandlerMap;
  typedef std::map<std::string, GroupCollection> GroupMap;

  PRW(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~PRW();
  void initialize();

public slots:
  void on_ik_move_go_clicked();
  void on_ik_move_reset_clicked();


  void closeEvent(QCloseEvent *event);


  void callbackJointState(const sensor_msgs::JointState& message);



  void selectPlanningGroup(unsigned int entry);
  void moveEndEffectorMarkers(double vx, double vy, double vz, double vr, double vp, double vw, bool coll_aware = true);
  void setNewEndEffectorPosition(PRW::GroupCollection& gc, tf::Transform& cur, bool coll_aware);
  bool solveIKForEndEffectorPose(PRW::GroupCollection& gc, bool coll_aware = true,
                                 bool constrain_pitch_and_roll = false, double change_redundancy = 0.0);
  void determinePitchRollConstraintsGivenState(const PRW::GroupCollection& gc,
                                               const planning_models::KinematicState& state,
                                               arm_navigation_msgs::OrientationConstraint& goal_constraint,
                                               arm_navigation_msgs::OrientationConstraint& path_constraint);
  geometry_msgs::Pose toGeometryPose(tf::Transform transform);
  tf::Transform toBulletTransform(geometry_msgs::Pose pose);
  void deleteKinematicStates();
  void createSelectableJointMarkers(PRW::GroupCollection& gc);
  void updateJointStates(PRW::GroupCollection& gc);
  void publishJointStates();

  void makeToLevelMenu();
  void processInteractiveFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void sendPlanningScene();
  void sendMarkers();


  GroupCollection* getPlanningGroup(unsigned int i)
  {
    unsigned int cmd = 0;
    for(GroupMap::iterator it = group_map_.begin(); it != group_map_.end(); it++)
    {
      if(cmd == i)
      {
        return &(it->second);
      }
      cmd++;
    }
    return NULL;
  }


  /////
  /// @brief Creates a marker that is initially a clickable menu. Upon selection it turns into a 6DOF control.
  /// @param type the type (collision object, joint, IK control, etc.) of the selectable marker.
  /// @param transform location and orientation of the marker.
  /// @param name internal, unique representation of the marker (this is for the 6DOF control)
  /// @param description displayed above the menu marker and the 6DOF marker.
  /// @param scale uniformly sizes the marker and its controls
  /// @param publish if true, the marker server will publish the marker. Otherwise, it will not.
  /////
  void makeSelectableMarker(InteractiveMarkerType type, tf::Transform transform, std::string name, std::string description,
                            float scale = 1.0f, bool publish = true)
  {
    SelectableMarker selectable_marker;
    selectable_marker.type_ = type;
    selectable_marker.name_ = name + "_selectable";
    selectable_marker.controlName_ = name;
    selectable_marker.controlDescription_ = description;

    visualization_msgs::InteractiveMarker marker;
    marker.header.frame_id = "/" + cm_->getWorldFrameId();
    ;
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = transform.getOrigin().x();
    marker.pose.position.y = transform.getOrigin().y();
    marker.pose.position.z = transform.getOrigin().z();
    marker.pose.orientation.w = transform.getRotation().w();
    marker.pose.orientation.x = transform.getRotation().x();
    marker.pose.orientation.y = transform.getRotation().y();
    marker.pose.orientation.z = transform.getRotation().z();
    marker.scale = scale;
    marker.name = name + "_selectable";
    marker.description = description;
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    switch (type)
    {
      case PRW::EndEffectorControl:
        control.markers.push_back(makeMarkerBox(marker, 0.5f));
        marker.controls.push_back(control);
        interactive_marker_server_->insert(marker);
        interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
        menu_handler_map_["End Effector Selection"].apply(*interactive_marker_server_, marker.name);
        break;
      case PRW::CollisionObject:
        control.markers.push_back(makeMarkerCylinder(marker, 1.0f));
        marker.controls.push_back(control);
        interactive_marker_server_->insert(marker);
        interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
        menu_handler_map_["Collision Object Selection"].apply(*interactive_marker_server_, marker.name);
        break;
      case PRW::JointControl:
        control.markers.push_back(makeMarkerBox(marker, 0.5f));
        marker.controls.push_back(control);
        interactive_marker_server_->insert(marker);
        interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
        menu_handler_map_["Joint Selection"].apply(*interactive_marker_server_, marker.name);
        break;
    }

    selectable_markers_[marker.name] = selectable_marker;

    if (publish)
    {
      interactive_marker_server_->applyChanges();
    }
  }

  ////
  /// @brief Returns true if a selectable marker of the given name exists in the marker map.
  /// @param name the unique identifier of the marker.
  /// @returns true if the marker exists, or false otherwise.
  ////
  bool selectableMarkerExists(std::string name)
  {
    return selectable_markers_.find(name) != selectable_markers_.end();
  }

  /////
  /// @brief Removes the menu marker given and replaces it with a 6DOF marker
  /// @param marker a reference to the selectablemarker struct.
  /// @param transform location to select the marker.
  /////
  void selectMarker(SelectableMarker& marker, tf::Transform transform)
  {
    visualization_msgs::InteractiveMarker dummy;
    if(interactive_marker_server_->get(marker.controlName_, dummy))
    {
      dummy.header.stamp = ros::Time::now();
      interactive_marker_server_->setPose(marker.controlName_, toGeometryPose(transform), dummy.header);
    }
    else
    {
      if(!interactive_marker_server_->erase(marker.name_))
      {
        return;
      }

      switch (marker.type_)
      {
        case PRW::EndEffectorControl:
          makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, 0.225f, false);
          break;
        case PRW::CollisionObject:
          makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, 2.0f, true);
          break;
        case PRW::JointControl:
          makeInteractive6DOFMarker(false, transform, marker.controlName_, marker.controlDescription_, 0.225f, false);
          break;
      }
    }
  }

  /////
  /// @brief Removes the 6DOF control of the given marker and replaces it with a menu.
  /// @param marker a reference to the selectablemarker struct.
  /// @param transform location of the marker when it is de-selected.
  /////
  void deselectMarker(SelectableMarker& marker, tf::Transform transform)
  {
    if(!interactive_marker_server_->erase(marker.controlName_))
    {
      return;
    }

    float scale = 1.0f;

    switch (marker.type_)
    {
      case PRW::EndEffectorControl:
        scale = 0.5f;
        break;
      case PRW::CollisionObject:
        scale = 2.0f;
        break;
      case PRW::JointControl:
        scale = 0.225f;
        break;
    }

    makeSelectableMarker(marker.type_, transform, marker.controlName_, marker.controlDescription_, scale);
  }


  void makeInteractive1DOFTranslationMarker(tf::Transform transform, tf::Vector3 axis, std::string name, std::string description,
                                            float scale = 1.0f, float value = 0.0f)
  {
    visualization_msgs::InteractiveMarker marker;
    marker.header.frame_id = cm_->getWorldFrameId();
    marker.pose.position.x = transform.getOrigin().x();
    marker.pose.position.y = transform.getOrigin().y();
    marker.pose.position.z = transform.getOrigin().z();
    marker.pose.orientation.w = transform.getRotation().w();
    marker.pose.orientation.x = transform.getRotation().x();
    marker.pose.orientation.y = transform.getRotation().y();
    marker.pose.orientation.z = transform.getRotation().z();
    marker.scale = scale;
    marker.name = name;
    marker.description = description;
    visualization_msgs::InteractiveMarker dummy;
    visualization_msgs::InteractiveMarkerControl control;
    if (interactive_marker_server_->get(marker.name, dummy))
    {
      interactive_marker_server_->setPose(marker.name, marker.pose, marker.header);
    }
    else
    {
      control.orientation.x = axis.x();
      control.orientation.y = axis.z();
      control.orientation.z = axis.y();
      control.orientation.w = 1;
      control.independent_marker_orientation = false;
      control.always_visible = false;
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      marker.controls.push_back(control);
      interactive_marker_server_->insert(marker);
      interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
    }

  }

  void makeInteractive1DOFRotationMarker(tf::Transform transform, tf::Vector3 axis, std::string name, std::string description,
                                         float scale = 1.0f, float angle = 0.0f)
  {
    visualization_msgs::InteractiveMarker marker;
    marker.header.frame_id = cm_->getWorldFrameId();
    marker.pose.position.x = transform.getOrigin().x();
    marker.pose.position.y = transform.getOrigin().y();
    marker.pose.position.z = transform.getOrigin().z();
    marker.pose.orientation.w = transform.getRotation().w();
    marker.pose.orientation.x = transform.getRotation().x();
    marker.pose.orientation.y = transform.getRotation().y();
    marker.pose.orientation.z = transform.getRotation().z();
    marker.scale = scale;
    marker.name = name;
    marker.description = description;

    visualization_msgs::InteractiveMarker dummy;
    if (interactive_marker_server_->get(marker.name, dummy))
    {
      interactive_marker_server_->setPose(marker.name, marker.pose, marker.header);
    }
    else
    {
      visualization_msgs::InteractiveMarkerControl control;
      control.orientation.x = axis.x();
      control.orientation.y = axis.z();
      control.orientation.z = axis.y();
      control.orientation.w = 1;
      control.independent_marker_orientation = false;
      control.always_visible = false;
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      marker.controls.push_back(control);
      interactive_marker_server_->insert(marker);
      interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
    }
  }

  /////
  /// @brief Creates an interactive 6DOF marker control.
  /// @param fixed if true, the axes remain fixed to the frame of the marker. Otherwise they rotate with input.
  /// @param transform the location and rotation of the 6DOF marker.
  /// @param name used internally to represent the marker. Must be unique.
  /// @param description drawn above the marker as a label.
  /// @param scale uniformly sets the size in meters of the marker.
  /// @param pole if true, the marker is a large green cylinder. Otherwise it is a small white cube.
  /////
  void makeInteractive6DOFMarker(bool fixed, tf::Transform transform, std::string name, std::string description,
                                 float scale = 1.0f, bool pole = false)
  {
    visualization_msgs::InteractiveMarker marker;
    marker.header.frame_id = "/" + cm_->getWorldFrameId();
    marker.pose.position.x = transform.getOrigin().x();
    marker.pose.position.y = transform.getOrigin().y();
    marker.pose.position.z = transform.getOrigin().z();
    marker.pose.orientation.w = transform.getRotation().w();
    marker.pose.orientation.x = transform.getRotation().x();
    marker.pose.orientation.y = transform.getRotation().y();
    marker.pose.orientation.z = transform.getRotation().z();
    marker.scale = scale;
    marker.name = name;
    marker.description = description;

    if (!pole)
    {
      makeInteractiveBoxControl(marker, 0.5f);
    }
    else
    {
      makeInteractiveCylinderControl(marker, 1.0f);
    }

    visualization_msgs::InteractiveMarkerControl control;

    if (fixed)
    {
      control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    }

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.always_visible = false;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);

    interactive_marker_server_->insert(marker);

    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    //control.markers.push_back(makeMarkerSphere(marker));
    marker.controls.push_back(control);

    if (!pole)
    {
      menu_handler_map_["End Effector"].apply(*interactive_marker_server_, marker.name);
    }
    else
    {
      menu_handler_map_["Collision Object"].apply(*interactive_marker_server_, marker.name);
    }

    interactive_marker_server_->setCallback(marker.name, process_function_ptr_);
  }


  /////
  /// @brief Creates a box shaped marker.
  /// @param msg the interactive marker to associate this box with.
  /// @param alpha the transparency of the marker.
  /// @return the marker (which is a box).
  /////
  visualization_msgs::Marker makeMarkerBox(visualization_msgs::InteractiveMarker &msg, float alpha = 1.0f)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    // Scale is arbitrarily 1/4 of the marker's scale.
    marker.scale.x = msg.scale * 0.25;
    marker.scale.y = msg.scale * 0.25;
    marker.scale.z = msg.scale * 0.25;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = alpha;

    return marker;
  }

  /////
  /// @brief Creates a cylinder shaped marker.
  /// @param msg the interactive marker to associate this cylinder with.
  /// @param alpha the transparency of the marker.
  /// @return the marker (which is a cylinder).
  /////
  visualization_msgs::Marker makeMarkerCylinder(visualization_msgs::InteractiveMarker &msg, float alpha = 1.0f)
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CYLINDER;
    // Scale is arbitrary
    marker.scale.x = msg.scale * 0.11;
    marker.scale.y = msg.scale * 0.11;
    marker.scale.z = msg.scale * 1.1;
    marker.color.r = 0.2;
    marker.color.g = 0.9;
    marker.color.b = 0.2;
    marker.color.a = alpha;

    return marker;
  }

  /////
  /// @brief Creates a sphere-shaped marker.
  /// @param msg the interactive marker to associate this sphere with.
  /// @return the marker (which is a sphere)
  /////
  visualization_msgs::Marker makeMarkerSphere(visualization_msgs::InteractiveMarker &msg)
  {
    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::SPHERE;
    // Scale is arbitrary.
    marker.scale.x = msg.scale * 0.75;
    marker.scale.y = msg.scale * 0.75;
    marker.scale.z = msg.scale * 0.75;
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 1.0;
    marker.color.a = 0.1;

    return marker;
  }

  /////
  /// @brief Creates a clickable, box shaped marker.
  /// @param msg the interactive marker to associate this box with.
  /// @param alpha the transparency of the marker.
  /// @return the control (which is a clickable box)
  /////
  visualization_msgs::InteractiveMarkerControl& makeInteractiveBoxControl(visualization_msgs::InteractiveMarker &msg,
                                                                          float alpha = 1.0f)
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeMarkerBox(msg, alpha));
    msg.controls.push_back(control);
    return msg.controls.back();
  }

  /////
  /// @brief Creates a clickable, cylinder shaped marker.
  /// @param msg the interactive marker to associate this cylinder with.
  /// @param alpha the transparency of the marker.
  /// @return the control (which is a clickable cylinder)
  /////
  visualization_msgs::InteractiveMarkerControl& makeInteractiveCylinderControl(
      visualization_msgs::InteractiveMarker &msg, float alpha = 1.0f)
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeMarkerCylinder(msg, alpha));
    msg.controls.push_back(control);
    return msg.controls.back();
  }

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

  tf::TransformBroadcaster transform_broadcaster_;
  planning_environment::CollisionModels* cm_;
  planning_models::KinematicState* robot_state_;
  IKControlType ik_control_type_;
  bool constrain_rp_;
  bool collision_aware_;
  bool is_ik_control_active_;
  bool is_joint_control_active_;


  /// Maps strings to menu handlers. This is used for convenience and extensibility.
  MenuHandlerMap menu_handler_map_;

  /// Maps MenuHandles to their names. Used to determine which menu entry is selected.
  MenuMap menu_entry_maps_;

  GroupMap group_map_;
  std::string current_group_name_;

  /// Boost function pointer to the main interactive feedback function.
  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher vis_marker_publisher_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;

  /// Maps selectable marker names to a struct containing their information.
  std::map<std::string, SelectableMarker> selectable_markers_;

  interactive_markers::MenuHandler::FeedbackCallback process_function_ptr_;

  ros::ServiceClient set_planning_scene_diff_client_;
  ros::ServiceClient planner_service_client_;
  ros::ServiceClient trajectory_filter_service_client_;

  boost::recursive_mutex lock_;

  boost::recursive_mutex joint_state_lock_;
  sensor_msgs::JointState last_joint_state_msg_;

};

#endif
