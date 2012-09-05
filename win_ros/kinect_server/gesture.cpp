#include "gesture.h"
#include <qdebug.h>
GestureDetector::GestureDetector()
  : window_size_(20), //60 frames ~ 2 seconds 
    min_period_between_gesture_(0),
    last_gesture_detected_time_(QDateTime::currentDateTime())
{
}

GestureDetector::~GestureDetector()
{
}

QPointF GestureDetector::skeletonToScreen( Vector4 skeletonPoint, int width, int height )
{
  LONG x, y;
  USHORT depth;

  // calculate the skeleton's position on the screen
  // NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
  NuiTransformSkeletonToDepthImage( skeletonPoint, &x, &y, &depth);

  float screenPointX = static_cast<float>(x * width) / 320;
  float screenPointY = static_cast<float>(y * height) / 240;

  return QPointF(screenPointX, screenPointY);
}

void GestureDetector::drawHistory(QPainter& painter)
{
  if(gesture_history_.isEmpty())
    return;

  int w = painter.device()->width();
  int h = painter.device()->height();
  QVector<QVector<QPointF> > points;
  points.resize(gesture_history_.begin()->first.size());
  PositionsStampedList::Iterator itr;
  for(itr = gesture_history_.begin(); itr != gesture_history_.end(); itr++)
  {
    for(int i = 0; i < itr->first.size(); i++)
    {
      points[i].push_back(skeletonToScreen(itr->first[i], w, h));
    }    
  }

  for(int i = 0; i < points.size(); i++)
  {
    painter.setPen(Qt::red);
    painter.drawLines(points[i]);
  }
}

void GestureDetector::addSkeleton(const QVector<Vector4>& skeleton_positions)
{
  PositionsStamped ps;
  ps.first = skeleton_positions;
  ps.second = QDateTime::currentDateTime();
  
  entries_.push_front(ps);
  gesture_history_.push_front(ps);

  if(entries_.size() > window_size_)
  {
    entries_.pop_back();
    if(!gesture_history_.isEmpty())
      gesture_history_.pop_back();
  }

  lookForGesture();
}

void GestureDetector::signalGestureDetected(const QString& gesture)
{
  quint64 dt = last_gesture_detected_time_.msecsTo(QDateTime::currentDateTime());
  
  if(dt > min_period_between_gesture_)
  {
    qDebug() << dt << gesture;
    if(!gesture.isEmpty())
    {
      emit gestureDetected(gesture);
    }
    last_gesture_detected_time_ = QDateTime::currentDateTime();    
  }

  //gesture_history_.clear();  
}

OnePointSwipeGestureDetector::OnePointSwipeGestureDetector()
  : GestureDetector(),
    min_length_(0.1f), //0.3 m
    max_height_(0.2f), //0.2 m
    min_duration_(250), //250 ms
    max_duration_(1500) //1500 ms
{
  min_period_between_gesture_ = 50;
}


OnePointSwipeGestureDetector::~OnePointSwipeGestureDetector()
{
}

void OnePointSwipeGestureDetector::lookForGesture()
{
  PositionsStampedList::Iterator itr, itr_start;
  itr_start = entries_.begin();
  for(itr = entries_.begin() + 1; itr != entries_.end(); itr++)
  {
    float height = fabs(itr_start->first[0].y - itr->first[0].y);
    if(height > max_height_)
    {      
      itr_start = itr;
      continue;
    }

    float lenght = fabs(itr_start->first[0].x - itr->first[0].x);
    if(lenght < min_length_)
    {     
      continue;
    }
    
    quint64 dt = itr->second.msecsTo(itr_start->second);
    if((dt >= min_duration_) && (dt < max_duration_))
    {        
      float sign = itr_start->first[0].x - itr->first[0].x;

      if(sign > -0.01f)
      {
        //swipe to right
        signalGestureDetected("SinglePointSwipeToRight");
      }
      else if(sign < 0.01f)
      {
        //swipe to left
        signalGestureDetected("SinglePointSwipeToLeft");
      }
    }
  }
}



TwoPointSwipeGestureDetector::TwoPointSwipeGestureDetector()
  : GestureDetector(),
    min_length_(0.1f), //0.3 m
    max_height_(0.2f), //0.2 m
    min_duration_(250), //250 ms
    max_duration_(1500) //1500 ms
{
    
}

TwoPointSwipeGestureDetector::~TwoPointSwipeGestureDetector()
{
}
  
void TwoPointSwipeGestureDetector::lookForGesture()
{
  PositionsStampedList::Iterator itr, itr_start;
  itr_start = entries_.begin();
  for(itr = entries_.begin() + 1; itr != entries_.end(); itr++)
  {
    float heightR = fabs(itr_start->first[0].y - itr->first[0].y);
    float heightL = fabs(itr_start->first[1].y - itr->first[1].y);
    if(heightR > max_height_ || heightL > max_height_)
    {      
      itr_start = itr;
      continue;
    }

    float lenghtR = fabs(itr_start->first[0].x - itr->first[0].x);
    float lenghtL = fabs(itr_start->first[1].x - itr->first[1].x);
    if(lenghtR < min_length_ || lenghtL < min_length_)
    {     
      continue;
    }
    
    quint64 dt = itr->second.msecsTo(itr_start->second);
    if((dt >= min_duration_) && (dt < max_duration_))
    {        
      float signR = itr_start->first[0].x - itr->first[0].x;
      float signL = itr_start->first[1].x - itr->first[1].x;


      if(signR > -0.01f && signL > -0.01f)
      {
        //swipe to right
        signalGestureDetected("TwoPointSwipeToRight");
      }
      else if(signR < 0.01f && signL < 0.01f)
      {
        //swipe to left
        signalGestureDetected("TwoPointSwipeToLeft");
      }
    }
  }
}