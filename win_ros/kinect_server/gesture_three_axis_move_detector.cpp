#include "gesture_three_axis_move_detector.h"
#include <qdebug.h>

ThreeAsixMoveGestureDetector::ThreeAsixMoveGestureDetector()
  : GestureDetector(),
    min_displacement_(0.1),
    offset_x_(0.15),
    offset_y_(-0.3),
    offset_z_(-0.3)
{

}

ThreeAsixMoveGestureDetector::~ThreeAsixMoveGestureDetector()
{
}


void ThreeAsixMoveGestureDetector::drawInteractiveUi(QPainter& painter)
{
  int w = painter.device()->width();
  int h = painter.device()->height();
  
  QPointF center = skeletonToScreen(entries_.first().first[2], w, h);

  QPointF right = skeletonToScreen(right_start_position_, w, h);
  QPointF left = skeletonToScreen(left_start_position_, w, h);
  
  Vector4 size = right_start_position_;
  size.x += min_displacement_;
  QPointF size_point = skeletonToScreen(size, w, h);
  int r = size_point.x() - right.x();

  painter.setPen(Qt::cyan);
  painter.drawEllipse(center, 10, 10);
  //painter.drawEllipse(right, r, r);
  painter.setPen(right_active_ ? Qt::red : Qt::cyan);
  painter.drawRect(right.x() - r, right.y() - r, r*2, r*2);
  painter.setPen(left_active_ ? Qt::red : Qt::cyan);
  painter.drawRect(left.x() - r, left.y() - r, r*2, r*2);
  //painter.drawEllipse(left, r, r);
  

}

void ThreeAsixMoveGestureDetector::lookForGesture()
{
  left_start_position_;
  right_start_position_;

  
  right_start_position_ = entries_.first().first[2];  
  right_start_position_.x += offset_x_;
  right_start_position_.y += offset_y_;
  right_start_position_.z += offset_z_; 

  left_start_position_ = entries_.first().first[2];
  left_start_position_.x -= offset_x_;
  left_start_position_.y += offset_y_;
  left_start_position_.z += offset_z_;

  Vector4 right_hand = entries_.first().first[0];
  Vector4 left_hand = entries_.first().first[1];
  
  //Right hand
  float dx = right_hand.x - right_start_position_.x; 
  float dy = right_hand.y - right_start_position_.y; 
  float dz = right_hand.z - right_start_position_.z; 
  
  bool move_x = fabs(dx) > min_displacement_ ? true : false;
  bool move_y = fabs(dy) > min_displacement_ ? true : false;
  bool move_z = fabs(dz) > min_displacement_ ? true : false;
  
  QString gesture("MoveThreeAsixR");
  if(move_x) dx > 0 ? gesture += "+X" : gesture += "-X";
  if(move_y) dy > 0 ? gesture += "+Y" : gesture += "-Y";
  if(move_z) dz > 0 ? gesture += "+Z" : gesture += "-Z";

  right_active_ = move_x || move_y || move_z;

  //qDebug() << gesture;

  //Left hand
  dx = left_hand.x - left_start_position_.x; 
  dy = left_hand.y - left_start_position_.y; 
  dz = left_hand.z - left_start_position_.z; 
  
  move_x = fabs(dx) > min_displacement_ ? true : false;
  move_y = fabs(dy) > min_displacement_ ? true : false;
  move_z = fabs(dz) > min_displacement_ ? true : false;
  
  gesture += "L";
  if(move_x) dx > 0 ? gesture += "+X" : gesture += "-X";
  if(move_y) dy > 0 ? gesture += "+Y" : gesture += "-Y";
  if(move_z) dz > 0 ? gesture += "+Z" : gesture += "-Z";

  left_active_ = move_x || move_y || move_z;

  //qDebug() << gesture;
  signalGestureDetected(gesture);



}
