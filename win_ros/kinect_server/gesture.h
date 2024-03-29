#ifndef _GESTURE_H_
#define _GESTURE_H_

#include <Windows.h>
#include <NuiApi.h>

#include <qobject.h>
#include <qvector.h>
#include <qpair.h>
#include <qlist.h>
#include <qdatetime.h>
#include <qpainter.h>

inline float fast_sign(float f) 
{
    if (f > 0) return 1;
    return (f == 0) ? 0 : -1;
}

class GestureDetector : public QObject
{
  Q_OBJECT
public:
  typedef QVector<Vector4> Positions;
  typedef QPair<Positions, QDateTime> PositionsStamped;
  typedef QList<PositionsStamped> PositionsStampedList;
  
  
  GestureDetector();
  virtual ~GestureDetector();

  void setWindowSize(int window_size) { window_size_ = window_size; }
  int getWindowSize() { return window_size_; }

  static QPointF skeletonToScreen( Vector4 skeletonPoint, int width, int height );
  virtual void drawHistory(QPainter& painter);
  virtual void drawInteractiveUi(QPainter& painter) { }

  virtual void addSkeleton(const QVector<Vector4>& skeleton_positions); 
  virtual void lookForGesture() = 0;

  QString getDetectedGesture() { return detected_gesture_; }
  
signals:
  void gestureDetected(const QString& gesture);

protected:
  void signalGestureDetected(const QString& gesture);

protected:
  int window_size_;
  int min_period_between_gesture_;
  QDateTime last_gesture_detected_time_;
  PositionsStampedList entries_;
  PositionsStampedList gesture_history_;
  QString detected_gesture_;

};


class OnePointSwipeGestureDetector : public GestureDetector
{
public:
  OnePointSwipeGestureDetector();
  ~OnePointSwipeGestureDetector();
  
  void lookForGesture();

protected:
  float min_length_;
  float max_height_;
  int min_duration_;
  int max_duration_;

};


class TwoPointSwipeGestureDetector : public GestureDetector
{
public:
  TwoPointSwipeGestureDetector();
  ~TwoPointSwipeGestureDetector();
  
  void lookForGesture();

protected:
  float min_length_;
  float max_height_;
  int min_duration_;
  int max_duration_;
};


#endif