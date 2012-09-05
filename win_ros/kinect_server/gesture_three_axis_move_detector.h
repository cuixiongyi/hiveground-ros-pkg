#ifndef _THREE_AXIS_MOVE_GESTURE_DETECTOR_H_
#define _THREE_AXIS_MOVE_GESTURE_DETECTOR_H_

#include "gesture.h"

class ThreeAsixMoveGestureDetector : public GestureDetector
{
public:
  ThreeAsixMoveGestureDetector();
  ~ThreeAsixMoveGestureDetector();

  //void drawHistory(QPainter& painter);
  void drawInteractiveUi(QPainter& painter);
  void lookForGesture();

protected:
  Vector4 left_start_position_;
  Vector4 right_start_position_;

  float min_displacement_;
  int min_duration_;
  int max_duration_;

  float offset_x_;
  float offset_y_;
  float offset_z_;
};



#endif