#ifndef _LEAN_BODY_GESTURE_DETECTOR_H_
#define _LEAN_BODY_GESTURE_DETECTOR_H_

#include "gesture.h"

class LeanBodyGestureDetector : public GestureDetector
{
public:
  LeanBodyGestureDetector();
  ~LeanBodyGestureDetector();

  //void drawHistory(QPainter& painter);
  void drawInteractiveUi(QPainter& painter);
  void lookForGesture();

  //void drawCube3D(QPainter& painter, Vector4 center, float size);

protected:
  Vector4 left_start_position_;
  Vector4 right_start_position_;

  float min_displacement_;
  int min_duration_;
  int max_duration_;

  float offset_x_;
  float offset_y_;
  float offset_z_;

  bool right_active_;
  bool left_active_;
};



#endif