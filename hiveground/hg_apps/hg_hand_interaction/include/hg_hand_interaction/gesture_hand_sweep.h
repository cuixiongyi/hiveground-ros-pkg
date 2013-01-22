/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 */

#ifndef GESTURE_HAND_SWEEP_H_
#define GESTURE_HAND_SWEEP_H_

#include <hg_hand_interaction/gesture.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>


namespace hg_hand_interaction
{

class SweepHandGestureDetector : public HandGestureDetector
{
  Q_OBJECT
  static const double DIRECTION_EPSILON = (1-0.866025404);

public:
  SweepHandGestureDetector(ros::NodeHandle& nh_private);
  virtual ~SweepHandGestureDetector() { }

  bool initialize();
  void drawHistory(visualization_msgs::MarkerArray& marker_array,
                     const std::string& frame_id = "base_link");
  void drawResult(visualization_msgs::MarkerArray& marker_array,
                    const std::string& frame_id = "base_link");
  void addHandMessage(const hg_object_tracking::HandsConstPtr message);
  bool lookForGesture();

  void addUI(QToolBox* tool_box);

protected Q_SLOTS:
  void onWindowTimeValueChanged(double d);
  void onGapTimeValueChanged(double d);
  void onFilterWindowSizeChanged(int d);

  Gesture detectOneHandGesture(int id);
  bool detectTwoHandGesture();
  tf::Vector3 getHandMovingDirection(int id);
  tf::Vector3 getFilteredDirection(int id, const tf::Vector3& latest_vector);

protected:
  int last_hand_count_;
  pcl::PCA<pcl::PointXYZ> pca_[2];
  bool direction_updated_[2];
  tf::Vector3 hand_moving_direction_[2];
  std::vector<std::list<tf::Vector3> > hand_moving_direction_filters_;
  int filter_windows_size_;
  tf::Vector3 three_axes[3];


  QSpinBox* spinbox_filter_windows_size_;
};

}



#endif /* GESTURE_HAND_SWEEP_H_ */
