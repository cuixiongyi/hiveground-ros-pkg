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

#ifndef HG_OBJECT_DETECTOR_H
#define HG_OBJECT_DETECTOR_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/tracking/tracking.h>
#include <pcl/common/pca.h>

#include <qmutex.h>

#include "ui_object_tracking.h"
#include <hg_object_tracking/Hands.h>

namespace hg_object_tracking
{






class ObjectTracking : public QMainWindow
{
  Q_OBJECT
public:

  ObjectTracking(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~ObjectTracking();

  bool initialize();




protected:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message);


  void sacSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_planar,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_objects);
  void objectSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                             std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr >& out);

  void detectHands(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clustered_clouds,
                     hg_object_tracking::Hands& hands);
  void detectFingers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud,
                       hg_object_tracking::Hand& hand);

  void pushHandMarker(pcl::PCA<pcl::PointXYZRGB>& pca, double scale = 0.1);
  void pushEigenMarker(pcl::PCA<pcl::PointXYZRGB>& pca, double scale = 0.1);
  void pushSimpleMarker(double x, double y, double z,
                           double r = 1.0, double g = 0.0, double b = 0.0);

protected Q_SLOTS:
  void on_doubleSpinBoxSacDistance_valueChanged(double d);
  void on_doubleSpinBoxEcTolerance_valueChanged(double d);
  void on_spinBoxEcMinSize_valueChanged(int d);
  void on_spinBoxEcMaxSize_valueChanged(int d);

  void on_doubleSpinBoxAreaXMin_valueChanged(double d);
  void on_doubleSpinBoxAreaXMax_valueChanged(double d);
  void on_doubleSpinBoxAreaYMin_valueChanged(double d);
  void on_doubleSpinBoxAreaYMax_valueChanged(double d);
  void on_doubleSpinBoxAreaZMin_valueChanged(double d);
  void on_doubleSpinBoxAreaZMax_valueChanged(double d);

  void on_spinBoxArmMinSize_valueChanged(int d);
  void on_spinBoxPlamMaxSize_valueChanged(int d);
  void on_doubleSpinArmEigenRatio_valueChanged(double d);

public:
  Ui::ObjectTracking ui;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  bool quit_threads_;

  ros::Publisher cloud_publisher_;
  ros::Subscriber cloud_subscriber_;
  ros::Publisher marker_array_publisher_;
  ros::Publisher hands_publisher_;


  QMutex mutex_cloud_;
  double sac_distance_threshold_;
  double ec_cluster_tolerance_;
  double ec_min_cluster_size_;
  double ec_max_cluster_size_;
  double area_x_min_, area_y_min_, area_z_min_;
  double area_x_max_, area_y_max_, area_z_max_;


  int arm_min_cluster_size_;
  int plam_max_cluster_size_;
  double arm_eigen_ratio_;


  visualization_msgs::MarkerArray marker_array_;
  int marker_id_;


};

}


#endif
