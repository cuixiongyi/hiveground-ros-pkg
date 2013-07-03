#ifndef KINECT_SERVER_H
#define KINECT_SERVER_H


#include <Windows.h>
#include <NuiApi.h>

#include <QtGui/QMainWindow>
#include "ui_kinect_server.h"
#include "glwidget.h"
#include "gesture.h"
#include "gesture_three_axis_move_detector.h"
#include "gesture_twist_body.h"

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>


class QActionGroup;

class kinect_server : public QMainWindow
{
	Q_OBJECT

public:
	kinect_server(ros::NodeHandle& nh, QWidget *parent = 0, Qt::WFlags flags = 0);
	~kinect_server();

protected slots:
  void setting_kinect_triggered(QAction* action);
  void setting_track_mode_triggered(QAction* action);
  void setting_tracked_skeleton_triggered(QAction* action);
  void setting_range_triggered(QAction* action);
  //void publish_gesture(const QString& gesture);

signals:
  void showImage(const QImage &image);
  void showDepth(const QImage &image);

protected:
  //Qt  
  void closeEvent(QCloseEvent *event);




  //Kinect
  void nuiZero();
  void updateKinectList();
  HRESULT nuiInit();
  void nuiUnInit();
  bool nuiGotColorAlert();
  bool nuiGotDepthAlert();
  bool nuiGotSkeletonAlert();
  void nuiProcessThread();
 
  void updateSkeletonTrackingFlag( DWORD flag, bool value );
  QPointF skeletonToScreen( Vector4 skeletonPoint, int width, int height );
  void nuiDrawBone(QPainter& painter, const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX bone0, NUI_SKELETON_POSITION_INDEX bone1 );
  void nuiDrawSkeleton(QPainter& painter, const NUI_SKELETON_DATA & skel, int windowWidth, int windowHeight );
  QImage nuiDepthToQImage(NUI_LOCKED_RECT LockedRect, int w, int h); 


public:
  ros::NodeHandle& nh_;
  bool quit_threads_;

private:
  //Qt
	Ui::kinect_serverClass ui;
  QActionGroup* setting_kinect_;
  QActionGroup* setting_track_mode_;
  QActionGroup* setting_tracked_skeleton_;
  QActionGroup* setting_range_;
  GLWidget* draw_depth_;
  GLWidget* draw_image_;
  
  //ROS
  //ros::Publisher gesture_pub_;
  ros::Publisher skeleton_pub_;


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
  BYTE depth_rgbx_[640*480*4];
  QPointF points_[NUI_SKELETON_POSITION_COUNT];
  QImage last_depth_image_;

  //gesture
  OnePointSwipeGestureDetector swipe_gesture_;
  TwoPointSwipeGestureDetector double_swipe_gesture_;
  ThreeAsixMoveGestureDetector three_axis_gesture_;
  TwistBodyGestureDetector twist_body_gesture_;
    
  
};

#endif // KINECT_SERVER_H
