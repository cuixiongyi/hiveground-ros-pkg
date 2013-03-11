#include "kinect_server.h"
#include <qdebug.h>
#include <qevent.h>
#include <qmessagebox.h>
#include <qactiongroup.h>

#include <mmsystem.h>
#include <assert.h>
#include <strsafe.h>

#include <std_msgs/String.h>
#include <kinect_msgs/Skeletons.h>

//lookups for color tinting based on player index
static const int g_IntensityShiftByPlayerR[] = { 1, 2, 0, 2, 0, 0, 2, 0 };
static const int g_IntensityShiftByPlayerG[] = { 1, 2, 2, 0, 2, 0, 0, 1 };
static const int g_IntensityShiftByPlayerB[] = { 1, 0, 2, 2, 0, 2, 0, 2 };

// Some smoothing with little latency (defaults).
// Only filters out small jitters.
// Good for gesture recognition.
const NUI_TRANSFORM_SMOOTH_PARAMETERS SMOOTH_DEFAULT = 
{0.5f, 0.5f, 0.5f, 0.05f, 0.04f};
 
// Smoothed with some latency.
// Filters out medium jitters.
// Good for a menu system that needs to be smooth but
// doesn't need the reduced latency as much as gesture recognition does.
const NUI_TRANSFORM_SMOOTH_PARAMETERS SMOOTH_MORE = 
{0.5f, 0.1f, 0.5f, 0.1f, 0.1f};
 
// Very smooth, but with a lot of latency.
// Filters out large jitters.
// Good for situations where smooth data is absolutely required
// and latency is not an issue.
const NUI_TRANSFORM_SMOOTH_PARAMETERS SMOOTH_VERY = 
{0.7f, 0.3f, 1.0f, 1.0f, 1.0f};

// Safe release for interfaces
template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
  if ( pInterfaceToRelease != NULL )
  {
    pInterfaceToRelease->Release();
    pInterfaceToRelease = NULL;
  }
}

kinect_server::kinect_server(ros::NodeHandle& nh, QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags),
  nh_(nh),
  quit_threads_(false)
{
  ui.setupUi(this);

  nuiZero();  
  updateKinectList();


  setting_track_mode_ = new QActionGroup(this);
  setting_track_mode_->addAction(ui.actionDefaultTracking);
  setting_track_mode_->addAction(ui.actionSeated);
  ui.actionSeated->setChecked(true);
  connect(setting_track_mode_, SIGNAL(triggered(QAction*)), this, SLOT(setting_track_mode_triggered(QAction*)));

  setting_tracked_skeleton_ = new QActionGroup(this);
  setting_tracked_skeleton_->addAction(ui.actionDefaultTracked);
  setting_tracked_skeleton_->addAction(ui.actionClosest_1_Player);
  setting_tracked_skeleton_->addAction(ui.actionClosest_2_Players);
  setting_tracked_skeleton_->addAction(ui.actionStick_1_Player);
  setting_tracked_skeleton_->addAction(ui.actionStick_2_Players);
  ui.actionDefaultTracked->setChecked(true);
  connect(setting_tracked_skeleton_, SIGNAL(triggered(QAction*)), this, SLOT(setting_tracked_skeleton_triggered(QAction*)));

  setting_range_ = new QActionGroup(this);
  setting_range_->addAction(ui.actionDefaultRange);
  setting_range_->addAction(ui.actionNear);
  ui.actionNear->setChecked(true);
  connect(setting_range_, SIGNAL(triggered(QAction*)), this, SLOT(setting_range_triggered(QAction*)));

  draw_depth_ = new GLWidget(this);
  draw_image_ = new GLWidget(this);
  ui.gridLayout->addWidget(draw_depth_, 0, 0);
  ui.gridLayout->addWidget(draw_image_, 0, 1);
  connect(this, SIGNAL(showDepth(const QImage &)), draw_depth_, SLOT(updateImage(const QImage&)));
  connect(this, SIGNAL(showImage(const QImage &)), draw_image_, SLOT(updateImage(const QImage&)));


  gesture_pub_ = nh_.advertise<std_msgs::String>("gesture", 10);
  skeleton_pub_ = nh_.advertise<kinect_msgs::Skeletons>("skeletons", 10);
  
  nuiInit();
}

kinect_server::~kinect_server()
{

}

void kinect_server::setting_kinect_triggered(QAction* action)
{
  ROS_INFO("set device");
}

void kinect_server::setting_track_mode_triggered(QAction* action)
{
  ROS_INFO("set tracking mode");
}

void kinect_server::setting_tracked_skeleton_triggered(QAction* action)
{
  ROS_INFO("set tracked skeleton");
}

void kinect_server::setting_range_triggered(QAction* action)
{
  ROS_INFO("set range");
}

/*
void kinect_server::publish_gesture(const QString& gesture)
{
  if(!gesture.isNull())
  {
    std_msgs::String msg;
    msg.data = gesture.toStdString();
    gesture_pub_.publish(msg); 
  }
}
*/
void kinect_server::closeEvent(QCloseEvent *event)
{
  quit_threads_ = true;  
  nuiUnInit();
  event->accept();
}

void kinect_server::nuiZero()
{
  SafeRelease(nui_sensor_);
  h_ev_nui_process_stop_ = NULL;
  h_next_depth_frame_event_ = NULL;
  h_next_color_frame_event_ = NULL;
  h_next_skeleton_event_ = NULL;
  p_video_stream_handle_ = NULL;
  p_depth_stream_handle_ = NULL;  
  skeleton_tracking_flags_ = NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE | NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT;
  //depth_stream_flags_ = 0;
  depth_stream_flags_ = NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE;

  depth_frames_total_ = 0;
  last_depth_fps_time_ = 0;
  last_depth_frames_total_ = 0;
  last_skeleton_found_time_  = 0;
  screen_blanked_ = false;
}

void kinect_server::updateKinectList()
{
  int numDevices = 0;
  HRESULT hr = NuiGetSensorCount(&numDevices);
  if ( FAILED(hr) )
  {
    return;
  }

  qDebug() << "Total device(s):" << numDevices;
  ui.menuKinect->clear();
  setting_kinect_ = new QActionGroup(this);
  if(numDevices > 0)
  {
    connect(setting_kinect_, SIGNAL(triggered ( QAction *)), this, SLOT(setting_kinect_triggered(QAction*)));
  }

  for (int i = 0; i < numDevices; ++i)
  {
    INuiSensor *pNui = NULL;
    HRESULT hr = NuiCreateSensorByIndex(i,  &pNui);
    if (SUCCEEDED(hr))
    {
      HRESULT status = pNui ? pNui->NuiStatus() : E_NUI_NOTCONNECTED;
      if (status == E_NUI_NOTCONNECTED)
      {
        pNui->Release();
        continue;
      }
      
      QAction *kinect = new QAction(QString("Kinect %1").arg(i), ui.menuKinect);
      kinect->setCheckable(true);
      setting_kinect_->addAction(kinect);
      ui.menuKinect->addAction(kinect);
      
      pNui->Release();
    }
  }
}

HRESULT kinect_server::nuiInit()
{
  HRESULT  hr;
  bool     result;

  if ( !nui_sensor_ )
  {
    hr = NuiCreateSensorByIndex(0, &nui_sensor_);

    if ( FAILED(hr) )
    {
      return hr;
    }

    //SysFreeString(instance_id_);

    instance_id_ = nui_sensor_->NuiDeviceConnectionId();   
  }

  h_next_depth_frame_event_ = CreateEvent( NULL, TRUE, FALSE, NULL );
  h_next_color_frame_event_ = CreateEvent( NULL, TRUE, FALSE, NULL );
  h_next_skeleton_event_ = CreateEvent( NULL, TRUE, FALSE, NULL );

  DWORD nuiFlags = NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON |  NUI_INITIALIZE_FLAG_USES_COLOR;

  hr = nui_sensor_->NuiInitialize(nuiFlags);
  if ( E_NUI_SKELETAL_ENGINE_BUSY == hr )
  {
    nuiFlags = NUI_INITIALIZE_FLAG_USES_DEPTH |  NUI_INITIALIZE_FLAG_USES_COLOR;
    hr = nui_sensor_->NuiInitialize( nuiFlags) ;
  }

  if ( FAILED( hr ) )
  {
    if ( E_NUI_DEVICE_IN_USE == hr )
    {
      QMessageBox::critical(this, "Error", "Device in use"); 
    }
    else
    {
      QMessageBox::critical(this, "Error", "Device initialization");
    }
    return hr;
  }

  if ( HasSkeletalEngine( nui_sensor_ ) )
  {
    hr = nui_sensor_->NuiSkeletonTrackingEnable( h_next_skeleton_event_, skeleton_tracking_flags_);
    if( FAILED( hr ) )
    {
      QMessageBox::critical(this, "Error", "Skelentan tracking");
      return hr;
    }
  }


  hr = nui_sensor_->NuiImageStreamOpen(
    NUI_IMAGE_TYPE_COLOR,
    NUI_IMAGE_RESOLUTION_640x480,
    0,
    2,
    h_next_color_frame_event_,
    &p_video_stream_handle_ );

  if ( FAILED( hr ) )
  {
    QMessageBox::critical(this, "Error", "Video stream");
    return hr;
  }

  hr = nui_sensor_->NuiImageStreamOpen(
    HasSkeletalEngine(nui_sensor_) ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH,
    NUI_IMAGE_RESOLUTION_640x480,
    depth_stream_flags_,
    2,
    h_next_depth_frame_event_,
    &p_depth_stream_handle_);

  if ( FAILED( hr ) )
  {
    QMessageBox::critical(this, "Error", "Depth stream");
    return hr;
  }

  // Start the Nui processing thread
  h_ev_nui_process_stop_ = CreateEvent( NULL, FALSE, FALSE, NULL );
  nui_thread_.reset(new boost::thread(boost::bind(&kinect_server::nuiProcessThread, this)));

  return hr;

}

void kinect_server::nuiUnInit()
{
  // Stop the Nui processing thread
  if ( NULL != h_ev_nui_process_stop_ )
  {
    // Signal the thread
    SetEvent(h_ev_nui_process_stop_);

    nui_thread_->join();
  }

  if ( nui_sensor_ )
  {
    nui_sensor_->NuiShutdown( );
  }
  if ( h_next_skeleton_event_ && ( h_next_skeleton_event_ != INVALID_HANDLE_VALUE ) )
  {
    CloseHandle( h_next_skeleton_event_ );
    h_next_skeleton_event_ = NULL;
  }
  if ( h_next_depth_frame_event_ && ( h_next_depth_frame_event_ != INVALID_HANDLE_VALUE ) )
  {
    CloseHandle( h_next_depth_frame_event_ );
    h_next_depth_frame_event_ = NULL;
  }
  if ( h_next_color_frame_event_ && ( h_next_color_frame_event_ != INVALID_HANDLE_VALUE ) )
  {
    CloseHandle( h_next_color_frame_event_ );
    h_next_color_frame_event_ = NULL;
  }

  SafeRelease( nui_sensor_ );

}

bool kinect_server::nuiGotColorAlert()
{
  NUI_IMAGE_FRAME imageFrame;
  bool processedFrame = true;

  HRESULT hr = nui_sensor_->NuiImageStreamGetNextFrame( p_video_stream_handle_, 0, &imageFrame );

  if ( FAILED( hr ) )
  {
    return false;
  }

  INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
  NUI_LOCKED_RECT LockedRect;
  pTexture->LockRect( 0, &LockedRect, NULL, 0 );
  if ( LockedRect.Pitch != 0 )
  {
    QImage image(LockedRect.pBits, 640, 480, QImage::Format_RGB32);    
    emit showImage(image);   
  }
  else
  {
    qDebug() << "Buffer length of received texture is bogus";
    processedFrame = false;
  }

  pTexture->UnlockRect( 0 );
  nui_sensor_->NuiImageStreamReleaseFrame( p_video_stream_handle_, &imageFrame );

  return true;
}

QImage kinect_server::nuiDepthToQImage(NUI_LOCKED_RECT LockedRect, int w, int h)
{
  DWORD frameWidth, frameHeight;
  frameWidth = w;
  frameHeight = h;

  // draw the bits to the bitmap
  BYTE * rgbrun = depth_rgbx_;
  const USHORT * pBufferRun = (const USHORT *)LockedRect.pBits;

  // end pixel is start + width*height - 1
  const USHORT * pBufferEnd = pBufferRun + (frameWidth * frameHeight);

  assert( frameWidth * frameHeight * g_BytesPerPixel <= ARRAYSIZE(m_depthRGBX) );

  while ( pBufferRun < pBufferEnd )
  {
    USHORT depth     = *pBufferRun;
    USHORT realDepth = NuiDepthPixelToDepth(depth);
    USHORT player    = NuiDepthPixelToPlayerIndex(depth);

    // transform 13-bit depth information into an 8-bit intensity appropriate
    // for display (we disregard information in most significant bit)
    BYTE intensity = static_cast<BYTE>(~(realDepth >> 4));

    // tint the intensity by dividing by per-player values
    *(rgbrun++) = intensity >> g_IntensityShiftByPlayerB[player];
    *(rgbrun++) = intensity >> g_IntensityShiftByPlayerG[player];
    *(rgbrun++) = intensity >> g_IntensityShiftByPlayerR[player];

    // no alpha information, skip the last byte
    ++rgbrun;

    ++pBufferRun;
  }


  return QImage(depth_rgbx_, 640, 480, QImage::Format_RGB32);    
}

bool kinect_server::nuiGotDepthAlert()
{
  NUI_IMAGE_FRAME imageFrame;
  bool processedFrame = true;

  HRESULT hr = nui_sensor_->NuiImageStreamGetNextFrame(
    p_depth_stream_handle_,
    0,
    &imageFrame );

  if ( FAILED( hr ) )
  {
    return false;
  }

  INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
  NUI_LOCKED_RECT LockedRect;
  pTexture->LockRect( 0, &LockedRect, NULL, 0 );
  if ( 0 != LockedRect.Pitch )
  {

    DWORD frameWidth, frameHeight;

    NuiImageResolutionToSize( imageFrame.eResolution, frameWidth, frameHeight );
   
    last_depth_image_ = nuiDepthToQImage(LockedRect, frameWidth, frameHeight);
  }
  else
  {
    processedFrame = false;
    qDebug("Buffer length of received texture is bogus");
  }

  pTexture->UnlockRect(0);
  nui_sensor_->NuiImageStreamReleaseFrame( p_depth_stream_handle_, &imageFrame );

  return true;
}

void kinect_server::updateSkeletonTrackingFlag( DWORD flag, bool value )
{
  DWORD newFlags = skeleton_tracking_flags_;

  if (value)
  {
    newFlags |= flag;
  }
  else
  {
    newFlags &= ~flag;
  }

  if (NULL != nui_sensor_ && newFlags != skeleton_tracking_flags_)
  {
    if ( !HasSkeletalEngine(nui_sensor_) )
    {
      QMessageBox::critical(this, "Error", "Skeleton track");
    }

    skeleton_tracking_flags_ = newFlags;

    HRESULT hr = nui_sensor_->NuiSkeletonTrackingEnable( h_next_skeleton_event_, skeleton_tracking_flags_ );

    if ( FAILED( hr ) )
    {
      QMessageBox::critical(this, "Error", "Skeleton track");
    }
  }
}

QPointF kinect_server::skeletonToScreen( Vector4 skeletonPoint, int width, int height )
{
  LONG x, y;
  USHORT depth;

  // calculate the skeleton's position on the screen
  // NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
  NuiTransformSkeletonToDepthImage( skeletonPoint, &x, &y, &depth );

  float screenPointX = static_cast<float>(x * width) / 320;
  float screenPointY = static_cast<float>(y * height) / 240;

  return QPointF(screenPointX, screenPointY);
}

void kinect_server::nuiDrawBone(QPainter& painter, const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX bone0, NUI_SKELETON_POSITION_INDEX bone1 )
{

  NUI_SKELETON_POSITION_TRACKING_STATE bone0State = skel.eSkeletonPositionTrackingState[bone0];
  NUI_SKELETON_POSITION_TRACKING_STATE bone1State = skel.eSkeletonPositionTrackingState[bone1];

  // If we can't find either of these joints, exit
  if ( bone0State == NUI_SKELETON_POSITION_NOT_TRACKED || bone1State == NUI_SKELETON_POSITION_NOT_TRACKED )
  {
    return;
  }

  // Don't draw if both points are inferred
  if ( bone0State == NUI_SKELETON_POSITION_INFERRED && bone1State == NUI_SKELETON_POSITION_INFERRED )
  {
    return;
  }

  // We assume all drawn bones are inferred unless BOTH joints are tracked
  if ( bone0State == NUI_SKELETON_POSITION_TRACKED && bone1State == NUI_SKELETON_POSITION_TRACKED )
  {
    painter.setPen(Qt::cyan);
    painter.drawLine(points_[bone0], points_[bone1]);
  }
  else
  {
    painter.setPen(Qt::yellow);
    painter.drawLine(points_[bone0], points_[bone1]);
  }

}

void kinect_server::nuiDrawSkeleton(QPainter& painter, const NUI_SKELETON_DATA & skel, int windowWidth, int windowHeight )
{      
  int i;

  for (i = 0; i < NUI_SKELETON_POSITION_COUNT; i++)
  {
    points_[i] = skeletonToScreen( skel.SkeletonPositions[i], windowWidth, windowHeight );
  }

  // Render Torso
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT );

  // Left Arm
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT );

  // Right Arm
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT );

  // Left Leg
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT );

  // Right Leg
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT );
  nuiDrawBone(painter, skel, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT );

  // Draw the joints in a different color
  QPen pen;
  pen.setWidth(2);
  for ( i = 0; i < NUI_SKELETON_POSITION_COUNT; i++ )
  {
    if ( skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_INFERRED )
    {
      pen.setColor(Qt::red);
      painter.setPen(pen);
      painter.drawEllipse(points_[i], 6, 6);
    }
    else if ( skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_TRACKED )
    {
      pen.setColor(Qt::green);
      painter.setPen(pen);      
      painter.drawEllipse(points_[i], 6, 6);
    }

  }
}

bool kinect_server::nuiGotSkeletonAlert()
{
  NUI_SKELETON_FRAME SkeletonFrame = {0};

  bool foundSkeleton = false;

  if ( SUCCEEDED(nui_sensor_->NuiSkeletonGetNextFrame( 0, &SkeletonFrame )) )
  {
    for ( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
    {
      NUI_SKELETON_TRACKING_STATE trackingState = SkeletonFrame.SkeletonData[i].eTrackingState;

      if ( trackingState == NUI_SKELETON_TRACKED || trackingState == NUI_SKELETON_POSITION_ONLY )
      {
        foundSkeleton = true;
      }
    }
  }

  nuiGotDepthAlert();


  // no skeletons!
  if( !foundSkeleton )
  {    
    emit showDepth(last_depth_image_);   
    return true;
  }

  // smooth out the skeleton data
  HRESULT hr = nui_sensor_->NuiTransformSmooth(&SkeletonFrame, &SMOOTH_DEFAULT);
  if ( FAILED(hr) )
  {
    return false;
  }

  // we found a skeleton, re-start the skeletal timer  
  screen_blanked_ = false;
  last_skeleton_found_time_ = timeGetTime( );

  //QImage image 
  QImage skeleton_image = last_depth_image_;
  int width = skeleton_image.width();
  int height = skeleton_image.height();
  QPainter painter(&skeleton_image);
   

  kinect_msgs::Skeletons skelentons_message;
  skelentons_message.header.stamp = ros::Time::now();
  skelentons_message.header.seq = SkeletonFrame.dwFrameNumber;
  skelentons_message.header.frame_id = "kinect_server";
  skelentons_message.skeletons.resize(NUI_SKELETON_COUNT);
  for ( int i = 0 ; i < NUI_SKELETON_COUNT; i++ )
  {
    NUI_SKELETON_TRACKING_STATE trackingState = SkeletonFrame.SkeletonData[i].eTrackingState;
    kinect_msgs::Skeleton skeleton;

    skelentons_message.skeletons[i].tracking_id = SkeletonFrame.SkeletonData[i].dwTrackingID;
    skelentons_message.skeletons[i].enrollment_index = SkeletonFrame.SkeletonData[i].dwEnrollmentIndex;
    skelentons_message.skeletons[i].user_index = SkeletonFrame.SkeletonData[i].dwUserIndex;
    skelentons_message.skeletons[i].skeleton_tracking_state = SkeletonFrame.SkeletonData[i].eTrackingState;
    skelentons_message.skeletons[i].quality_flag = SkeletonFrame.SkeletonData[i].dwQualityFlags;

    skelentons_message.skeletons[i].position.translation.x = SkeletonFrame.SkeletonData[i].Position.x; 
    skelentons_message.skeletons[i].position.translation.y = SkeletonFrame.SkeletonData[i].Position.y;
    skelentons_message.skeletons[i].position.translation.z = SkeletonFrame.SkeletonData[i].Position.z;        
    skelentons_message.skeletons[i].position.rotation.x = 0;
    skelentons_message.skeletons[i].position.rotation.y = 0;
    skelentons_message.skeletons[i].position.rotation.z = 0;
    skelentons_message.skeletons[i].position.rotation.w = 1;
      
    skelentons_message.skeletons[i].skeleton_positions.resize(NUI_SKELETON_POSITION_COUNT);
    skelentons_message.skeletons[i].skeleton_position_tracking_state.resize(NUI_SKELETON_POSITION_COUNT);
    for(int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++)
    {
      skelentons_message.skeletons[i].skeleton_position_tracking_state[j] = SkeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[j];
      skelentons_message.skeletons[i].skeleton_positions[j].translation.x = SkeletonFrame.SkeletonData[i].SkeletonPositions[j].x;
      skelentons_message.skeletons[i].skeleton_positions[j].translation.y = SkeletonFrame.SkeletonData[i].SkeletonPositions[j].y;
      skelentons_message.skeletons[i].skeleton_positions[j].translation.z = SkeletonFrame.SkeletonData[i].SkeletonPositions[j].z;        
      skelentons_message.skeletons[i].skeleton_positions[j].rotation.x = 0;
      skelentons_message.skeletons[i].skeleton_positions[j].rotation.y = 0;
      skelentons_message.skeletons[i].skeleton_positions[j].rotation.z = 0;
      skelentons_message.skeletons[i].skeleton_positions[j].rotation.w = 1;
    }

    

    


    if ( trackingState == NUI_SKELETON_TRACKED )
    {

#if 0      
      //skeleton.      
      QString gesture;

      GestureDetector::Positions positions0;
      positions0.push_back(SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT]);
      positions0.push_back(SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT]);
      positions0.push_back(SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER]);
            
      if((SkeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_HAND_RIGHT] == NUI_SKELETON_POSITION_TRACKED) &&
         (SkeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_HAND_LEFT] == NUI_SKELETON_POSITION_TRACKED)) //&&
         //(SkeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_SHOULDER_RIGHT] == NUI_SKELETON_POSITION_TRACKED) &&
         //(SkeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_SHOULDER_LEFT] == NUI_SKELETON_POSITION_TRACKED))
      {
        three_axis_gesture_.addSkeleton(positions0);
        three_axis_gesture_.drawInteractiveUi(painter);
        three_axis_gesture_.drawHistory(painter);
        if(gesture != "") gesture += "_";
        gesture += three_axis_gesture_.getDetectedGesture();
      }

      GestureDetector::Positions positions1;
      positions1.push_back(SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER]);
      positions1.push_back(SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT]);
      positions1.push_back(SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT]);
      twist_body_gesture_.addSkeleton(positions1);
      twist_body_gesture_.drawInteractiveUi(painter);
      twist_body_gesture_.drawHistory(painter);
      if(gesture != "") gesture += "_";
      gesture += twist_body_gesture_.getDetectedGesture();

     


      if(!gesture.isNull())
      {
        std_msgs::String msg;
        msg.data = gesture.toStdString();
        ROS_INFO_STREAM_THROTTLE(1.0, msg.data);
        gesture_pub_.publish(msg); 
      }
#endif      
      // We're tracking the skeleton, draw it
      nuiDrawSkeleton(painter, SkeletonFrame.SkeletonData[i], width, height );
    }
    else if ( trackingState == NUI_SKELETON_POSITION_ONLY )
    {
      QPointF point = skeletonToScreen( SkeletonFrame.SkeletonData[i].Position, width, height );
      painter.setPen(Qt::blue);
      painter.drawEllipse(point, 3, 3);
    }
  }

  skeleton_pub_.publish(skelentons_message);
  emit showDepth(skeleton_image);   
  return true;
}

void kinect_server::nuiProcessThread()
{
  const int numEvents = 4;
  HANDLE hEvents[numEvents] = { h_ev_nui_process_stop_, h_next_depth_frame_event_, h_next_color_frame_event_, h_next_skeleton_event_ };
  int    nEventIdx;
  DWORD  t;

  last_depth_fps_time_ = timeGetTime( );

  // Blank the skeleton display on startup
  last_skeleton_found_time_ = 0;

  // Main thread loop
  bool continueProcessing = true;

  while ( continueProcessing )
  {
    // Wait for any of the events to be signalled
    nEventIdx = WaitForMultipleObjects( numEvents, hEvents, FALSE, 100 );

    // Timed out, continue
    if ( nEventIdx == WAIT_TIMEOUT )
    {
      //ROS_INFO("aa");
      continue;
    }

    // stop event was signalled 
    if ( WAIT_OBJECT_0 == nEventIdx )
    {
      continueProcessing = false;
      break;
    }

    // Wait for each object individually with a 0 timeout to make sure to
    // process all signalled objects if multiple objects were signalled
    // this loop iteration

    // In situations where perfect correspondance between color/depth/skeleton
    // is essential, a priority queue should be used to service the item
    // which has been updated the longest ago
    /*
    if ( WAIT_OBJECT_0 == WaitForSingleObject( h_next_depth_frame_event_, 0 ) )
    {
    //only increment frame count if a frame was successfully drawn
    if ( nuiGotDepthAlert() )
    {
    ++depth_frames_total_;
    }
    }
    */
    if ( WAIT_OBJECT_0 == WaitForSingleObject( h_next_color_frame_event_, 0 ) )
    {
      nuiGotColorAlert();
    }

    if (  WAIT_OBJECT_0 == WaitForSingleObject( h_next_skeleton_event_, 0 ) )
    {
      nuiGotSkeletonAlert( );
      ++depth_frames_total_;
    }

    // Once per second, display the depth FPS
    t = timeGetTime( );
    if ( (t - last_depth_fps_time_) > 1000 )
    {
      float dt =  (t - last_depth_fps_time_);
      float fps = ((depth_frames_total_ - last_depth_frames_total_) * 1000 + 500) / dt;
      ui.statusBar->showMessage(QString("FPS: %1").arg(fps));
      last_depth_frames_total_ = depth_frames_total_;
      last_depth_fps_time_ = t;
    }

    // Blank the skeleton panel if we haven't found a skeleton recently
    if ( (t - last_skeleton_found_time_) > 300 )
    {
      if ( !screen_blanked_ )
      {
        screen_blanked_ = true;
      }
    }

  }
}