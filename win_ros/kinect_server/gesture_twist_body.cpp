#include "gesture_twist_body.h"

#include "gesture_three_axis_move_detector.h"
#include <qdebug.h>

TwistBodyGestureDetector::TwistBodyGestureDetector()
  : GestureDetector(), 
    min_z_diff_(0.15), //10cm
    twist_detected_(false),
    last_twist_direction_(0),
    min_duration_(100) //100ms    
{
  
}

TwistBodyGestureDetector::~TwistBodyGestureDetector()
{
}


void TwistBodyGestureDetector::drawInteractiveUi(QPainter& painter)
{
 
}

void TwistBodyGestureDetector::lookForGesture()
{
  //ROS_INFO(__FUNCTION__);
  Vector4 center = entries_.first().first[0];
  Vector4 right = entries_.first().first[1];
  Vector4 left = entries_.first().first[2];

  float dz_right = right.z - center.z;
  float dz_left = left.z - center.z;
  float dz_left_right = left.z - right.z;

  //qDebug("-- %6.3f %6.3f %6.3f", left.z, center.z ,right.z);
  //qDebug("== %6.3f %6.3f %6.3f", dz_left, dz_left_right, dz_right);
  QString gesture("TwistBody:");
  if(fabs(dz_left_right) >= min_z_diff_)
  {
    int twist_direction = 0;
    if((fast_sign(dz_left) == 1) && (fast_sign(dz_right) == -1))
    {
      //qDebug("get twist left");
      twist_direction = 1;
    }
    else if((fast_sign(dz_left) == -1) && (fast_sign(dz_right) == 1))
    {
      //qDebug("get twist right");
      twist_direction = -1;
    }
    else
    {
      twist_detected_ = false;
    }
        
    if(twist_detected_)
    {
      //check duration
      if(last_twist_direction_ != twist_direction)
      {
        qDebug() << "twist changed";
        twist_detected_ = false;
        signalGestureDetected(gesture);
        return;
      }
      
      qint64 dt = gesture_start_time_.msecsTo(QDateTime::currentDateTime());
      //qDebug() << "dt" << dt;
      if(dt > min_duration_)
      {
       
        if(twist_direction == 1)
        {
          gesture += "L";
        }
        else if(twist_direction == -1)
        {
          gesture += "R";         
        }
       
      }
      else
      {
        if(twist_direction == 1)
        {
           qDebug() << "left twisting detected and wating...";
        }
        else if(twist_direction == -1)
        {
          qDebug() << "right twisting detected and wating...";
        }       
      }
    }
    else
    {
      if(twist_direction != 0)
      {
        gesture_start_time_ = QDateTime::currentDateTime();
        twist_detected_ = true;
        last_twist_direction_ = twist_direction;
      }
    }
  }
  signalGestureDetected(gesture);
}
