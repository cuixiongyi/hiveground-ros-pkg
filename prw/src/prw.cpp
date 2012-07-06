#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>
#include <prw/prw.h>

#include <QtGui/QApplication>

static const std::string VIS_TOPIC_NAME = "prw_visualizer";
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "/environment_server/set_planning_scene_diff";
static const std::string PLANNER_SERVICE_NAME = "/ompl_planning/plan_kinematic_path";
static const std::string TRAJECTORY_FILTER_SERVICE_NAME = "/trajectory_filter_server/filter_trajectory_with_constraints";

//in 100 hz ticks
static const unsigned int CONTROL_SPEED = 10;


#define DEG2RAD(x) (((x)*M_PI)/180.0)

PRW::PRW(QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags),
    //node_handle_("~"),
    quit_threads_(false)

{
  ui.setupUi(this);

}


PRW::~PRW()
{
}

void PRW::initialize()
{
  ROS_INFO_STREAM("Initializing...");

  //initialize collision model with robot model
  cm_ = new planning_environment::CollisionModels("robot_description");
  vis_marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker> (VIS_TOPIC_NAME, 128);
  vis_marker_array_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray> (VIS_TOPIC_NAME + "_array", 128);


  /*
  while (!ros::service::waitForService("/arm_kinematics/get_ik", ros::Duration(1.0)))
  {
    ROS_INFO_STREAM("Waiting for planning scene service " << "/arm_kinematics/get_ik");
  }

  ik_client_ = node_handle_.serviceClient<kinematics_msgs::GetPositionIK>("/arm_kinematics/get_ik", true);

  while (!ros::service::waitForService("/arm_kinematics/get_ik_solver_info", ros::Duration(1.0)))
  {
    ROS_INFO_STREAM("Waiting for planning scene service " << "/arm_kinematics/get_ik_solver_info");
  }

  ik_info_client_
    = node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("/arm_kinematics/get_ik_solver_info");
  */

  while (!ros::service::waitForService(SET_PLANNING_SCENE_DIFF_NAME, ros::Duration(1.0)))
  {
    ROS_INFO_STREAM("Waiting for planning scene service " << SET_PLANNING_SCENE_DIFF_NAME);
  }

  set_planning_scene_diff_client_
    = node_handle_.serviceClient<arm_navigation_msgs::SetPlanningSceneDiff>(SET_PLANNING_SCENE_DIFF_NAME);

  while (!ros::service::waitForService(PLANNER_SERVICE_NAME, ros::Duration(1.0)))
  {
    ROS_INFO_STREAM("Waiting for planner service " << PLANNER_SERVICE_NAME);
  }

  planner_service_client_
    = node_handle_.serviceClient<arm_navigation_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME, true);

  while (!ros::service::waitForService(TRAJECTORY_FILTER_SERVICE_NAME, ros::Duration(1.0)))
  {
    ROS_INFO_STREAM("Waiting for trajectory filter service " << TRAJECTORY_FILTER_SERVICE_NAME);
  }

  trajectory_filter_service_client_
    = node_handle_.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>(TRAJECTORY_FILTER_SERVICE_NAME, true);


  while (!ros::service::waitForService("vp6242_arm_kinematics/get_ik_solver_info", ros::Duration(1.0)))
  {
    ROS_INFO_STREAM("Waiting for trajectory filter service " << "vp6242_arm_kinematics/get_ik_solver_info");
  }

  ik_info_client_
    = node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("vp6242_arm_kinematics/get_ik_solver_info");

  while (!ros::service::waitForService("vp6242_arm_kinematics/get_ik", ros::Duration(1.0)))
  {
    ROS_INFO_STREAM("Waiting for trajectory filter service " << "vp6242_arm_kinematics/get_ik");
  }

  ik_client_
    = node_handle_.serviceClient<kinematics_msgs::GetPositionIK>("vp6242_arm_kinematics/get_ik");


  robot_state_ = new planning_models::KinematicState(cm_->getKinematicModel());
  robot_state_->setKinematicStateToDefault();
  sendPlanningScene();




  subscriber_joint_state_ = node_handle_.subscribe("/joint_states", 1, &PRW::callbackJointState, this);

  publisher_joints_[0] = node_handle_.advertise<std_msgs::Float64>("/arm1/J1/command/position", 1);
  publisher_joints_[1] = node_handle_.advertise<std_msgs::Float64>("/arm1/J2/command/position", 1);
  publisher_joints_[2] = node_handle_.advertise<std_msgs::Float64>("/arm1/J3/command/position", 1);
  publisher_joints_[3] = node_handle_.advertise<std_msgs::Float64>("/arm1/J4/command/position", 1);
  publisher_joints_[4] = node_handle_.advertise<std_msgs::Float64>("/arm1/J5/command/position", 1);
  publisher_joints_[5] = node_handle_.advertise<std_msgs::Float64>("/arm1/J6/command/position", 1);


  ROS_INFO_STREAM("Initialized");
}

void PRW::on_ik_move_go_clicked()
{
  /*
  ROS_INFO("Go clicked! xyz(%4.2f, %4.2f %4.2f) ryp(%4.2f, %4.2f, %4.2f)",
           ui.ik_move_x->value(),
           ui.ik_move_y->value(),
           ui.ik_move_z->value(),
           ui.ik_move_roll->value(),
           ui.ik_move_pitch->value(),
           ui.ik_move_yaw->value());
  */
  /*
  geometry_msgs::PoseStamped ps, pose;
  ps.header.frame_id = "/world";
  ps.pose.position.x = ui.ik_move_x->value();
  ps.pose.position.y = ui.ik_move_y->value();
  ps.pose.position.z = ui.ik_move_z->value();

  tf::Quaternion q(DEG2RAD(ui.ik_move_yaw->value()), DEG2RAD(ui.ik_move_pitch->value()), DEG2RAD(ui.ik_move_roll->value()));
  ps.pose.orientation.x = q.x();
  ps.pose.orientation.y = q.y();
  ps.pose.orientation.z = q.z();
  ps.pose.orientation.w = q.w();

  //tf_listener_.transformPose("/base_link", ps, pose);
  //ROS_INFO_STREAM(ps.pose);
  //ROS_INFO_STREAM(pose.pose);


  kinematics_msgs::GetPositionIKRequest request;
  request.timeout = ros::Duration(5.0);
  request.ik_request.pose_stamped.header.frame_id = "/base_link";
  request.ik_request.ik_link_name = "link5";
  request.ik_request.pose_stamped.pose = ps.pose; //not transformation
  //request.ik_request.pose_stamped.pose.position.y = pose.pose.position.y
  //request.ik_request.pose_stamped.pose.position.z = pose.pose.position.z

  //ik_info_client_.
  kinematics_msgs::GetKinematicSolverInfo srv;
  if(ik_info_client_.call(srv))
  {
    ROS_INFO_STREAM(srv.response.kinematic_solver_info.joint_names.size());
    for(size_t i = 0; i < srv.response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_INFO_STREAM(srv.response.kinematic_solver_info.joint_names[i]);
    }
  }

  request.ik_request.ik_seed_state.joint_state.name = srv.response.kinematic_solver_info.joint_names;
  request.ik_request.ik_seed_state.joint_state.position = joint_positions_;

  kinematics_msgs::GetPositionIKResponse respond;
  if(ik_client_.call(request, respond))
  {
    if(respond.error_code.val == ::arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS)
    {
      for(size_t i = 0; i < respond.solution.joint_state.position.size(); i++)
      {
        ROS_INFO_STREAM(respond.solution.joint_state.position[i]);
        std_msgs::Float64 msg;
        msg.data = respond.solution.joint_state.position[i];
        publisher_joints_[i].publish(msg);
      }
    }
    else
    {
      ROS_INFO("request not success");
    }
  }
  else
  {
    ROS_INFO("request error");
  }
  */




  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;
  if(ik_info_client_.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_INFO("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }

  // define the service messages
  kinematics_msgs::GetPositionIK::Request  gpik_req;
  kinematics_msgs::GetPositionIK::Response gpik_res;
  gpik_req.timeout = ros::Duration(5.0);



  gpik_req.ik_request.pose_stamped.header.frame_id = "base_link";
  gpik_req.ik_request.ik_link_name = "link5";
  //gpik_req.ik_request.pose_stamped.header.stamp = ros::Time::now();
  gpik_req.ik_request.pose_stamped.pose.position.x = ui.ik_move_x->value();
  gpik_req.ik_request.pose_stamped.pose.position.y = ui.ik_move_y->value();
  gpik_req.ik_request.pose_stamped.pose.position.z = ui.ik_move_z->value();

  tf::Quaternion q(DEG2RAD(ui.ik_move_yaw->value()), DEG2RAD(ui.ik_move_pitch->value()), DEG2RAD(ui.ik_move_roll->value()));
  gpik_req.ik_request.pose_stamped.pose.orientation.x = q.x();
  gpik_req.ik_request.pose_stamped.pose.orientation.y = q.y();
  gpik_req.ik_request.pose_stamped.pose.orientation.z = q.z();
  gpik_req.ik_request.pose_stamped.pose.orientation.w = q.w();

  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
  for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++)
  {
    //gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position
      //  + response.kinematic_solver_info.limits[i].max_position) / 2.0;
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = 0;

    ROS_INFO_STREAM(gpik_req.ik_request.ik_seed_state.joint_state.position[i]);
  }
  if (ik_client_.call(gpik_req, gpik_res))
  {
    if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
      for (unsigned int i = 0; i < gpik_res.solution.joint_state.name.size(); i++)
      {
        ROS_INFO(
            "Joint: %s %f", gpik_res.solution.joint_state.name[i].c_str(), gpik_res.solution.joint_state.position[i]);
        std_msgs::Float64 msg;
        msg.data = gpik_res.solution.joint_state.position[i];
        publisher_joints_[i].publish(msg);
      }
    else
    {
      ROS_ERROR("Inverse kinematics failed %d", gpik_res.error_code.val);
    }
  }
  else
    ROS_ERROR("Inverse kinematics service call failed");


}

void PRW::on_ik_move_reset_clicked()
{
  ui.ik_move_x->setValue(0.0);
  ui.ik_move_y->setValue(0.0);
  ui.ik_move_z->setValue(0.0);
  ui.ik_move_roll->setValue(0.0);
  ui.ik_move_pitch->setValue(0.0);
  ui.ik_move_yaw->setValue(0.0);
}

void PRW::sendPlanningScene()
{
  ROS_INFO("Sending Planning Scene....");

  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;


  planning_environment::convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(),
                                                          planning_scene_req.planning_scene_diff.robot_state);

  /*
  planning_models::KinematicState* startState = NULL;
  planning_models::KinematicState* endState = NULL;
  map<string, double> startStateValues;
  map<string, double> endStateValues;

  if (current_group_name_ != "")
  {
    startState = group_map_[current_group_name_].getState(StartPosition);
    endState = group_map_[current_group_name_].getState(EndPosition);

    if (startState != NULL)
    {
      startState->getKinematicStateValues(startStateValues);
    }
    if (endState != NULL)
    {
      endState->getKinematicStateValues(endStateValues);
    }
  }

  deleteKinematicStates();
  */

  if (robot_state_ != NULL)
  {
    ROS_INFO("Reverting planning scene to default.");
    cm_->revertPlanningScene(robot_state_);
    robot_state_ = NULL;
  }


  if (!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res))
  {
    ROS_WARN("Can't get planning scene");
    return;
  }

  robot_state_ = cm_->setPlanningScene(planning_scene_res.planning_scene);

  if (robot_state_ == NULL)
  {
    ROS_ERROR("Something wrong with planning scene");
    return;
  }
  ROS_INFO("Planning scene sent.");
}

void PRW::sendMarkers()
{
  visualization_msgs::MarkerArray arr;

  std_msgs::ColorRGBA stat_color_;
  stat_color_.a = 1.0;
  stat_color_.r = 0.1;
  stat_color_.g = 0.8;
  stat_color_.b = 0.3;

  std_msgs::ColorRGBA attached_color_;
  attached_color_.a = 1.0;
  attached_color_.r = 0.6;
  attached_color_.g = 0.4;
  attached_color_.b = 0.3;

  cm_->getAllCollisionSpaceObjectMarkers(*robot_state_, arr, "", stat_color_, attached_color_, ros::Duration(0.1));
  //if (!current_group_name_.empty())
  {
    std_msgs::ColorRGBA group_color;
    group_color.a = 0.3;
    group_color.r = 0.5;
    group_color.g = 0.9;
    group_color.b = 0.5;

    std_msgs::ColorRGBA updated_color;
    updated_color.a = 0.3;
    updated_color.r = 1.0;
    updated_color.g = 0.5;
    updated_color.b = 1.0;

    std_msgs::ColorRGBA bad_color;
    bad_color.a = 0.6;
    bad_color.r = 1.0;
    bad_color.g = 0.0;
    bad_color.b = 0.0;

    //GroupCollection& gc = group_map_[current_group_name_];
    const planning_models::KinematicModel* kinematic_model = cm_->getKinematicModel();

    /*
    IKControlType otherState;
    if (ik_control_type_ == EndPosition)
    {
      otherState = StartPosition;
    }
    else
    {
      otherState = EndPosition;
    }

    if (is_ik_control_active_)
    {
      if (gc.getState(otherState) != NULL)
      {
        cm_->getGroupAndUpdatedJointMarkersGivenState(*gc.getState(otherState), arr, current_group_name_, group_color,
                                                      updated_color, ros::Duration(0.1));
      }
      else
      {
        ROS_ERROR("Other state invalid!");
      }
    }

    if (!gc.good_ik_solution_ && gc.getState(ik_control_type_) != NULL)
    {
      vector < string > lnames = kinematic_model->getChildLinkModelNames(
          kinematic_model->getLinkModel(gc.ik_link_name_));

      cm_->getRobotMarkersGivenState(*gc.getState(ik_control_type_), arr, bad_color, current_group_name_,
                                     ros::Duration(0.1), &lnames);
      cm_->getAttachedCollisionObjectMarkers(*gc.getState(ik_control_type_), arr, current_group_name_, bad_color,
                                             ros::Duration(.2));

    }
    for (map<string, StateTrajectoryDisplay>::iterator it = gc.state_trajectory_display_map_.begin();
        it != gc.state_trajectory_display_map_.end(); it++)
    {

      if (it->second.play_joint_trajectory_)
      {
        moveThroughTrajectory(gc, it->first, 5);
      }

      if (it->second.show_joint_trajectory_)
      {
        const vector<const KinematicModel::LinkModel*>& updated_links =
            kinematic_model->getModelGroup(gc.name_)->getUpdatedLinkModels();
        vector < string > lnames;
        lnames.resize(updated_links.size());
        for (unsigned int i = 0; i < updated_links.size(); i++)
        {
          lnames[i] = updated_links[i]->getName();
        }
        cm_->getRobotMarkersGivenState(*(it->second.state_), arr, it->second.color_, it->first + "_trajectory",
                                       ros::Duration(0.1), &lnames);

        cm_->getAttachedCollisionObjectMarkers(*(it->second.state_), arr, it->first + "_trajectory", it->second.color_,
                                               ros::Duration(0.1));
      }
    }
    */
  }
  vis_marker_array_publisher_.publish(arr);


}


void PRW::closeEvent(QCloseEvent *event)
{
  ROS_INFO("Close windows");
  quit_threads_ = true;
  event->accept();
}


void PRW::callbackJointState(const sensor_msgs::JointState& message)
{
  //ROS_INFO_STREAM(message);
  joint_positions_ = message.position;
}

#include "prw_marker.cpp"

PRW* prw_ = NULL;
bool initialized_ = false;
void spin_function()
{
  ros::WallRate r(100.0);
  while(ros::ok() && !initialized_)
  {
    r.sleep();
    ros::spinOnce();
  }
  while(ros::ok() && !prw_->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "personal_robotic_workspace", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);

  PRW w;
  w.initialize();
  prw_ = &w;
  w.show();

  initialized_ = true;

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
