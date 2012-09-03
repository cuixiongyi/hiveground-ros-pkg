#ifndef KINECT_SERVER_H
#define KINECT_SERVER_H


#include <Windows.h>
#include <NuiApi.h>

#include <QtGui/QMainWindow>
#include "ui_kinect_server.h"

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>



class kinect_server : public QMainWindow
{
	Q_OBJECT

public:
	kinect_server(ros::NodeHandle& nh, QWidget *parent = 0, Qt::WFlags flags = 0);
	~kinect_server();
protected:
  //Qt  
  void closeEvent(QCloseEvent *event);


  //Kinect
  void nuiZero();
  void updateKinectComboBox();
  HRESULT nuiInit();
  void nuiUnInit();
  bool nuiGotColorAlert();
  bool nuiGotDepthAlert();
  bool nuiGotSkeletonAlert();
  void nuiProcessThread();



public:
  ros::NodeHandle& nh_;
  bool quit_threads_;

private:
	Ui::kinect_serverClass ui;

  //Kinect
  INuiSensor* nui_sensor_;
  BSTR instance_id_;
  boost::shared_ptr<boost::thread> nui_thread_; 
  HANDLE h_ev_nui_process_stop_;
  HANDLE h_next_depth_frame_event_;
  HANDLE h_next_color_frame_event_;
  HANDLE h_next_skeleton_event_;
  HANDLE p_depth_stream_handle_;
  HANDLE p_video_stream_handle_;
  DWORD skeleton_tracking_flags_;
  DWORD depth_stream_flags_;

  int depth_frames_total_;
  DWORD last_depth_fps_time_;
  int   last_depth_frames_total_;
  DWORD last_skeleton_found_time_;

  bool screen_blanked_;
    
  
};

#endif // KINECT_SERVER_H
