#include "kinect_server.h"
#include <qdebug.h>
#include <qevent.h>
#include <qmessagebox.h>

#include <mmsystem.h>
#include <assert.h>
#include <strsafe.h>


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
  updateKinectComboBox();
  nuiInit();
}

kinect_server::~kinect_server()
{

}

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
  skeleton_tracking_flags_ = NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE;
  depth_stream_flags_ = 0;

  depth_frames_total_ = 0;
  last_depth_fps_time_ = 0;
  last_depth_frames_total_ = 0;
  last_skeleton_found_time_  = 0;
  screen_blanked_ = false;


}

void kinect_server::updateKinectComboBox()
{
  int numDevices = 0;
  HRESULT hr = NuiGetSensorCount(&numDevices);
  if ( FAILED(hr) )
  {
    return;
  }

  qDebug() << "Total device(s):" << numDevices;

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

      //qDebug() <<  pNui->
      //BSTR id = pNui->NuiDeviceConnectionId();
      //QString qstr((QChar*)id, ::SysStringLen(id));
      //qDebug() << qstr;
      //SysFreeString(id);

      ui.cbDevice->addItem(QString("Kinect %1").arg(i));     
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
      //MessageBoxResource( IDS_ERROR_IN_USE, MB_OK | MB_ICONHAND );
    }
    else
    {
      QMessageBox::critical(this, "Error", "Device initialization");
      //MessageBoxResource( IDS_ERROR_NUIINIT, MB_OK | MB_ICONHAND );
    }
    return hr;
  }
  
  if ( HasSkeletalEngine( nui_sensor_ ) )
  {
    hr = nui_sensor_->NuiSkeletonTrackingEnable( h_next_skeleton_event_, skeleton_tracking_flags_);
    if( FAILED( hr ) )
    {
      QMessageBox::critical(this, "Error", "Skelentan tracking");
      //MessageBoxResource( IDS_ERROR_SKELETONTRACKING, MB_OK | MB_ICONHAND );
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
    //MessageBoxResource( IDS_ERROR_VIDEOSTREAM, MB_OK | MB_ICONHAND );
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
    //MessageBoxResource(IDS_ERROR_DEPTHSTREAM, MB_OK | MB_ICONHAND);
    return hr;
  }

  // Start the Nui processing thread
  h_ev_nui_process_stop_ = CreateEvent( NULL, FALSE, FALSE, NULL );
  //m_hThNuiProcess = CreateThread( NULL, 0, Nui_ProcessThread, this, 0, NULL );
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

  nui_sensor_->NuiImageStreamReleaseFrame( p_video_stream_handle_, &imageFrame );

  ROS_INFO_THROTTLE(1.0, "got color");
  return true;
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

  nui_sensor_->NuiImageStreamReleaseFrame( p_depth_stream_handle_, &imageFrame );


  ROS_INFO_THROTTLE(1.0, "got depth");
  return true;
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

  // no skeletons!
  if( !foundSkeleton )
  {
    return true;
  }


  ROS_INFO_THROTTLE(1.0, "got skeleton");
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

    if ( WAIT_OBJECT_0 == WaitForSingleObject( h_next_depth_frame_event_, 0 ) )
    {
      //only increment frame count if a frame was successfully drawn
      if ( nuiGotDepthAlert() )
      {
        ++depth_frames_total_;
      }
    }

    if ( WAIT_OBJECT_0 == WaitForSingleObject( h_next_color_frame_event_, 0 ) )
    {
      nuiGotColorAlert();
    }

    if (  WAIT_OBJECT_0 == WaitForSingleObject( h_next_skeleton_event_, 0 ) )
    {
      nuiGotSkeletonAlert( );
    }

    // Once per second, display the depth FPS
    t = timeGetTime( );
    if ( (t - last_depth_fps_time_) > 1000 )
    {
      DWORD dt =  (t - last_depth_fps_time_);
      float fps = ((depth_frames_total_ - last_depth_frames_total_) * 1000 + 500) / dt;
      ROS_INFO("FPS:%d %d %d %f", dt, depth_frames_total_, last_depth_frames_total_,  fps);
      //PostMessageW( m_hWnd, WM_USER_UPDATE_FPS, IDC_FPS, fps );
      last_depth_frames_total_ = depth_frames_total_;
      last_depth_fps_time_ = t;
    }

    // Blank the skeleton panel if we haven't found a skeleton recently
    if ( (t - last_skeleton_found_time_) > 300 )
    {
      if ( !screen_blanked_ )
      {
        //Nui_BlankSkeletonScreen( );
        screen_blanked_ = true;
      }
    }

  }
}