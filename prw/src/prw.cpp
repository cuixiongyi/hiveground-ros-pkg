#include <ros/ros.h>
#include <prw/prw.h>

#include <QtGui/QApplication>


PRW::PRW(QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags),
    node_handle_("~"),
    quit_threads_(false)

{
  ui.setupUi(this);

}


PRW::~PRW()
{
}

void PRW::initialize()
{
  ros::service::waitForService("/arm_kinematics/get_ik");
  ros::service::waitForService("/arm_kinematics/get_ik_solver_info");
  ik_client_ = node_handle_.serviceClient<kinematics_msgs::GetPositionIK>("/arm_kinematics/get_ik", true);
  ik_info_client_ = node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("/arm_kinematics/get_ik_solver_info");
  subscriber_joint_state_ = node_handle_.subscribe("/joint_states", 1, &PRW::callbackJointState, this);

}

void PRW::on_ik_move_go_clicked()
{
  ROS_INFO("Go clicked! xyz(%4.2f, %4.2f %4.2f) ryp(%4.2f, %4.2f, %4.2f)",
           ui.ik_move_x->value(),
           ui.ik_move_y->value(),
           ui.ik_move_z->value(),
           ui.ik_move_roll->value(),
           ui.ik_move_pitch->value(),
           ui.ik_move_yaw->value());


  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = "/base_link";
  ps.pose.position.x = ui.ik_move_x->value();
  ps.pose.position.y = ui.ik_move_y->value();
  ps.pose.position.z = ui.ik_move_z->value();

  tf::Quaternion q(ui.ik_move_yaw->value(), ui.ik_move_pitch->value(), ui.ik_move_roll->value());
  ps.pose.orientation.x = q.x();
  ps.pose.orientation.y = q.y();
  ps.pose.orientation.z = q.z();
  ps.pose.orientation.w = q.w();

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
      respond.solution.joint_state.position;
      for(size_t i = 0; i < respond.solution.joint_state.position.size(); i++)
      {
        ROS_INFO_STREAM(respond.solution.joint_state.position[i]);
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
