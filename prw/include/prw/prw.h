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

  typedef std::map<std::string, SelectableMarker> SelectableMarkerMap;

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
    trajectory_msgs::JointTrajectory joint_trajectory_;
    unsigned int current_trajectory_point_;
    std_msgs::ColorRGBA color_;
    bool has_joint_trajectory_;
    bool play_joint_trajectory_;
    bool show_joint_trajectory_;
    arm_navigation_msgs::ArmNavigationErrorCodes trajectory_error_code_;
    unsigned int trajectory_bad_point_;
  };

  typedef std::map<std::string, StateTrajectoryDisplay> StateTrajectoryDisplayMap;

  struct GroupCollection
  {
    GroupCollection()
    {
      start_state_ = NULL;
      end_state_ = NULL;
      good_ik_solution_ = false;

      state_trajectory_display_map_["shadow"].color_.a = .5;
      state_trajectory_display_map_["shadow"].color_.r = 0.5;
      state_trajectory_display_map_["shadow"].color_.g = 0.5;
      state_trajectory_display_map_["shadow"].color_.b = 0.5;

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
      for (StateTrajectoryDisplayMap::iterator it = state_trajectory_display_map_.begin();
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
    StateTrajectoryDisplayMap state_trajectory_display_map_;
    std::vector<std::string> joint_names_;
    tf::Transform last_good_state_;
  };

  typedef std::map<std::string, GroupCollection> GroupMap;
  typedef std::map<std::string, planning_models::KinematicModel::GroupConfig> GroupConfigMap;

  typedef std::map<interactive_markers::MenuHandler::EntryHandle, std::string> MenuEntryMap;
  typedef std::map<std::string, MenuEntryMap> MenuMap;
  typedef std::map<std::string, interactive_markers::MenuHandler> MenuHandlerMap;

public:

  PRW(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~PRW();
  void initialize();

public slots:
  void on_ik_move_go_clicked();
  void on_ik_move_reset_clicked();

public:

  void deleteKinematicStates();
  void sendPlanningScene();
  void selectPlanningGroup(unsigned int entry);
  void sendMarkers();

  GroupCollection* getPlanningGroup(unsigned int i);

  void callbackJointState(const sensor_msgs::JointState& message);


  void processInteractiveFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void setNewEndEffectorPosition(GroupCollection& gc, tf::Transform& cur, bool coll_aware);
  void determinePitchRollConstraintsGivenState(const PRW::GroupCollection& gc,
                                               const planning_models::KinematicState& state,
                                               arm_navigation_msgs::OrientationConstraint& goal_constraint,
                                               arm_navigation_msgs::OrientationConstraint& path_constraint) const;
  bool solveIKForEndEffectorPose(PRW::GroupCollection& gc, bool coll_aware = true,
                                 bool constrain_pitch_and_roll = false, double change_redundancy = 0.0);
  void updateJointStates(PRW::GroupCollection& gc);

  bool planToEndEffectorState(PRW::GroupCollection& gc, bool play = true);
  bool filterPlannerTrajectory(PRW::GroupCollection& gc, bool play = true);

  void moveEndEffectorMarkers(double vx, double vy, double vz, double vr, double vp, double vw, bool coll_aware = true);

  bool playTrajectory(PRW::GroupCollection& gc, const std::string& source_name,
                      const trajectory_msgs::JointTrajectory& traj);
  void moveThroughTrajectory(PRW::GroupCollection& gc, const std::string& source_name, int step);



  ////
  /// @brief Returns true if a selectable marker of the given name exists in the marker map.
  /// @param name the unique identifier of the marker.
  /// @returns true if the marker exists, or false otherwise.
  ////
  bool selectableMarkerExists(std::string name);

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

  /////
  /// @brief Creates an interactive 6DOF marker control.
  /// @param fixed if true, the axes remain fixed to the frame of the marker. Otherwise they rotate with input.
  /// @param transform the location and rotation of the 6DOF marker.
  /// @param name used internally to represent the marker. Must be unique.
  /// @param description drawn above the marker as a label.
  /// @param scale uniformly sets the size in meters of the marker.
  /// @param pole if true, the marker is a large green cylinder. Otherwise it is a small white cube.
  /////
  void makeInteractive6DOFMarker(bool fixed, tf::Transform transform, std::string name, std::string description, float scale = 1.0f,
                                 bool pole = false);

  /////
  /// @brief Creates a marker that is initially a clickable menu. Upon selection it turns into a 6DOF control.
  /// @param type the type (collision object, joint, IK control, etc.) of the selectable marker.
  /// @param transform location and orientation of the marker.
  /// @param name internal, unique representation of the marker (this is for the 6DOF control)
  /// @param description displayed above the menu marker and the 6DOF marker.
  /// @param scale uniformly sizes the marker and its controls
  /// @param publish if true, the marker server will publish the marker. Otherwise, it will not.
  /////
  void makeSelectableMarker(InteractiveMarkerType type, tf::Transform transform, std::string name,
                            std::string description, float scale = 1.0f, bool publish = true);
  /////
  /// @brief Creates a box shaped marker.
  /// @param msg the interactive marker to associate this box with.
  /// @param alpha the transparency of the marker.
  /// @return the marker (which is a box).
  /////
  visualization_msgs::Marker makeMarkerBox(visualization_msgs::InteractiveMarker &msg, float alpha = 1.0f);

  /////
  /// @brief Creates a cylinder shaped marker.
  /// @param msg the interactive marker to associate this cylinder with.
  /// @param alpha the transparency of the marker.
  /// @return the marker (which is a cylinder).
  /////
  visualization_msgs::Marker makeMarkerCylinder(visualization_msgs::InteractiveMarker &msg, float alpha = 1.0f);

  /////
  /// @brief Creates a sphere-shaped marker.
  /// @param msg the interactive marker to associate this sphere with.
  /// @return the marker (which is a sphere)
  /////
  visualization_msgs::Marker makeMarkerSphere(visualization_msgs::InteractiveMarker &msg);

  /////
  /// @brief Creates a clickable, box shaped marker.
  /// @param msg the interactive marker to associate this box with.
  /// @param alpha the transparency of the marker.
  /// @return the control (which is a clickable box)
  /////
  visualization_msgs::InteractiveMarkerControl& makeInteractiveBoxControl(visualization_msgs::InteractiveMarker &msg, float alpha = 1.0f);

  /////
  /// @brief Creates a clickable, cylinder shaped marker.
  /// @param msg the interactive marker to associate this cylinder with.
  /// @param alpha the transparency of the marker.
  /// @return the control (which is a clickable cylinder)
  /////
  visualization_msgs::InteractiveMarkerControl& makeInteractiveCylinderControl(visualization_msgs::InteractiveMarker &msg, float alpha = 1.0f);

  void makeTopLevelMenu();

  void deleteJointMarkers(GroupCollection& gc);

  ////
  /// @brief Given the group collection, creates a set of selectable markers for each joint in the group.
  /// @param gc the group collection to create joint markers for.
  ////
  void createSelectableJointMarkers(GroupCollection& gc);

protected:
  tf::Transform toBulletTransform(geometry_msgs::Pose pose);
  geometry_msgs::Pose toGeometryPose(tf::Transform transform);


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
  IKControlType ik_control_type_;
  GroupMap group_map_;
  bool constrain_roll_pitch_;
  bool collision_aware_;
  bool is_ik_control_active_;
  bool is_joint_control_active_;
  std::string current_group_name_;
  arm_navigation_msgs::MotionPlanRequest last_motion_plan_request_;



  boost::recursive_mutex lock_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher vis_marker_publisher_;

  /// Maps selectable marker names to a struct containing their information.
  SelectableMarkerMap selectable_markers_;

  /// Maps end effector link names to their previously recorded poses.
  std::map<std::string, geometry_msgs::Pose> last_ee_poses_;



  /// Boost function pointer to the main interactive feedback function.
  interactive_markers::MenuHandler::FeedbackCallback process_function_ptr_;

  interactive_markers::MenuHandler::EntryHandle start_position_handle_;
  interactive_markers::MenuHandler::EntryHandle end_position_handle_;
  interactive_markers::MenuHandler::EntryHandle ik_control_handle_;
  interactive_markers::MenuHandler::EntryHandle joint_control_handle_;
  interactive_markers::MenuHandler::EntryHandle collision_aware_handle_;
  interactive_markers::MenuHandler::EntryHandle constrain_rp_handle_;


  /// Maps strings to menu handlers. This is used for convenience and extensibility.
  MenuHandlerMap menu_handler_map_;

  /// Maps MenuHandles to their names. Used to determine which menu entry is selected.
  MenuMap menu_entry_maps_;


};

#endif
