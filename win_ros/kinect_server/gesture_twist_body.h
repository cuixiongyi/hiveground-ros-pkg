#ifndef _TWIST_BODY_GESTURE_DETECTOR_H_
#define _TWIST_BODY_GESTURE_DETECTOR_H_

#include "gesture.h"

class TwistBodyGestureDetector : public GestureDetector
{
public:
  TwistBodyGestureDetector();
  ~TwistBodyGestureDetector();

  //void drawHistory(QPainter& painter);
  void drawInteractiveUi(QPainter& painter);
  void lookForGesture();

  //void drawCube3D(QPainter& painter, Vector4 center, float size);

protected:
  float min_z_diff_;
  bool twist_detected_;
  int last_twist_direction_;
  int min_duration_;
  QDateTime gesture_start_time_;
};



#endif