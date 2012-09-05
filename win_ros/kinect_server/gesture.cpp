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

void GestureDetector::drawHistory(QPainter& painter)
{

}

void GestureDetector::addSkeleton(const QVector<Vector4>& skeleton_positions)
{
  PositionsStamped ps;
  ps.first = skeleton_positions;
  ps.second = QDateTime::currentDateTime();
  
  entries_.push_front(ps);

  if(entries_.size() > window_size_)
  {
    entries_.pop_back();
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
}

SinglePointSwipeGestureDetector::SinglePointSwipeGestureDetector()
  : GestureDetector(),
    min_length_(0.1f), //0.3 m
    max_height_(0.2f), //0.2 m
    min_duration_(250), //250 ms
    max_duration_(1500) //1500 ms
{
  min_period_between_gesture_ = 100;
}


SinglePointSwipeGestureDetector::~SinglePointSwipeGestureDetector()
{
}

void SinglePointSwipeGestureDetector::lookForGesture()
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
      //qDebug() << dt << lenght << height;
      float sign = itr_start->first[0].x - itr->first[0].x;

      if(sign > -0.01f)
      {
        //swipe to right
        //qDebug() << "swipe to right";
        signalGestureDetected("SinglePointSwipeToRight");
      }
      else if(sign < 0.01f)
      {
        //swipe to left
        //qDebug() << "swipe to left";
        signalGestureDetected("SinglePointSwipeToLeft");
      }
    }
  }
}