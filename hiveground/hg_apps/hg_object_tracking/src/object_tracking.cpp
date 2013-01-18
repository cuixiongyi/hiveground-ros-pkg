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

#include <ros/ros.h>
#include <hg_object_tracking/object_tracking.h>
#include <QtGui/QApplication>
#include <QDebug>

#include <boost/thread.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


using namespace hg_object_tracking;
using namespace visualization_msgs;

ObjectTracking::ObjectTracking(QWidget *parent, Qt::WFlags flags) :
    QMainWindow(parent, flags),
    nh_(), nh_private_("~"),
    quit_threads_(false)
{
  ui.setupUi(this);
}

ObjectTracking::~ObjectTracking()
{
}

bool ObjectTracking::initialize()
{
  sac_segmentator_.setOptimizeCoefficients(true);
  sac_segmentator_.setModelType(pcl::SACMODEL_PLANE);
  sac_segmentator_.setMethodType(pcl::SAC_RANSAC);
  sac_segmentator_.setMaxIterations(1000);

  double d;
  int i;
  nh_private_.getParam("sac_distance_threshold", d);
  ui.doubleSpinBoxSacDistance->setValue(d);
  nh_private_.getParam("ec_cluster_tolerance", d);
  ui.doubleSpinBoxEcTolerance->setValue(d);
  nh_private_.getParam("ec_min_size", i);
  ui.spinBoxEcMinSize->setValue(i);
  nh_private_.getParam("ec_max_size", i);
  ui.spinBoxEcMaxSize->setValue(i);

  nh_private_.getParam("area_x_min", d);
  ui.doubleSpinBoxAreaXMin->setValue(d);
  nh_private_.getParam("area_x_max", d);
  ui.doubleSpinBoxAreaXMax->setValue(d);
  nh_private_.getParam("area_y_min", d);
  ui.doubleSpinBoxAreaYMin->setValue(d);
  nh_private_.getParam("area_y_max", d);
  ui.doubleSpinBoxAreaYMax->setValue(d);
  nh_private_.getParam("area_z_min", d);
  ui.doubleSpinBoxAreaZMin->setValue(d);
  nh_private_.getParam("area_z_max", d);
  ui.doubleSpinBoxAreaZMax->setValue(d);



  cloud_publisher_ = nh_private_.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);
  cloud_subscriber_ = nh_private_.subscribe("cloud_in", 1, &ObjectTracking::cloudCallback, this);
  marker_publisher_ = nh_.advertise<Marker>("tracked_object", 128);
  marker_array_publisher_ = nh_.advertise<MarkerArray>("tracked_object_array", 128);

  return true;
}


void ObjectTracking::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message)
{
  QMutexLocker lock(&mutex_cloud_);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*message, *cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planar(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_objects(new pcl::PointCloud<pcl::PointXYZRGB>);
  sacSegmentation(cloud, cloud_planar, cloud_objects);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);
  objectSegmentation(cloud_objects, cloud_clustered);

  if(cloud_publisher_.getNumSubscribers() != 0)
  {
    sensor_msgs::PointCloud2 output_cloud;
    if(ui.radioButtonShowObjectCloud->isChecked())
    {
      pcl::toROSMsg(*cloud_objects, output_cloud);
    }
    else if(ui.radioButtonShowPlaneCloud->isChecked())
    {
      pcl::toROSMsg(*cloud_planar, output_cloud);
    }
    else if(ui.radioButtonShowSegmentedCloud->isChecked())
    {
      pcl::toROSMsg(*cloud_clustered, output_cloud);
    }

    output_cloud.header.stamp = message->header.stamp;
    output_cloud.header.frame_id = message->header.frame_id;
    cloud_publisher_.publish(output_cloud);
  }


}

void ObjectTracking::sacSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_planar,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_objects)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  sac_segmentator_.setInputCloud(in->makeShared());
  sac_segmentator_.segment(*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    ROS_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }
  else
  {
    ROS_DEBUG("Indices size: %d", (int)inliers->indices.size ());
  }

  indices_extractor_.setNegative(false);
  indices_extractor_.setInputCloud(in);
  indices_extractor_.setIndices(inliers);
  indices_extractor_.filter(*out_planar);

  indices_extractor_.setNegative(true);
  indices_extractor_.filter(*out_objects);
}

void ObjectTracking::objectSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr out)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(in);
  std::vector<pcl::PointIndices> cluster_indices;

  ec_extractor_.setSearchMethod(tree);
  ec_extractor_.setInputCloud(in);
  ec_extractor_.extract(cluster_indices);

  ROS_DEBUG_THROTTLE(1.0, "total object %d", (int)cluster_indices.size());

  float cx, cy, cz;
  float cr, cg, cb;
  MarkerArray marker_array;
  int id = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    Marker marker;
    marker.lifetime = ros::Duration(0.1);
    marker.type = Marker::SPHERE;
    marker.header.frame_id = "base_link";
    marker.ns = "object_tracking";
    marker.id = id++;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    cx = cy = cz = 0.0;
    cr = cg = cb = 0.0;

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster->points.push_back(in->points[*pit]);

      //compute centroid
      cx += in->points[*pit].x;
      cy += in->points[*pit].y;
      cz += in->points[*pit].z;

      //get color color
      unsigned char * rgb = (unsigned char *)&(in->points[*pit].rgb);
      cr += rgb[0];
      cg += rgb[1];
      cb += rgb[2];
    }


    int size = cloud_cluster->points.size();
    cx = cx / size;
    cy = cy / size;
    cz = cz / size;

    if((cx < area_x_min_) || (cx > area_x_max_)) continue;
    if((cy < area_y_min_) || (cy > area_y_max_)) continue;
    if((cz < area_z_min_) || (cz > area_z_max_)) continue;

    (*out) += (*cloud_cluster);

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.pose.position.x = cx;
    marker.pose.position.y = cy;
    marker.pose.position.z = cz;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.color.b = (cr / size) / 255.0;
    marker.color.g = (cg / size) / 255.0;
    marker.color.r = (cb / size) / 255.0;
    marker.color.a = 0.5;
    //ROS_INFO_STREAM(marker);
    marker_array.markers.push_back(marker);
  }

  if (marker_array_publisher_.getNumSubscribers() != 0)
  {
    marker_array_publisher_.publish(marker_array);
  }


}

void ObjectTracking::on_doubleSpinBoxSacDistance_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  if(d != sac_segmentator_.getDistanceThreshold())
    sac_segmentator_.setDistanceThreshold(d);
}

void ObjectTracking::on_doubleSpinBoxEcTolerance_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  if(d != ec_extractor_.getClusterTolerance())
    ec_extractor_.setClusterTolerance(d);
}

void ObjectTracking::on_spinBoxEcMinSize_valueChanged(int d)
{
  QMutexLocker lock(&mutex_cloud_);
  if(d != ec_extractor_.getMinClusterSize())
    ec_extractor_.setMinClusterSize(d);
}

void ObjectTracking::on_spinBoxEcMaxSize_valueChanged(int d)
{
  QMutexLocker lock(&mutex_cloud_);
  if(d != ec_extractor_.getMaxClusterSize())
    ec_extractor_.setMaxClusterSize(d);
}

void ObjectTracking::on_doubleSpinBoxAreaXMin_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  if(d != area_x_min_) area_x_min_ = d;
}
void ObjectTracking::on_doubleSpinBoxAreaXMax_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  if(d != area_x_max_) area_x_max_ = d;
}
void ObjectTracking::on_doubleSpinBoxAreaYMin_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  if(d != area_y_min_) area_y_min_ = d;
}
void ObjectTracking::on_doubleSpinBoxAreaYMax_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  if(d != area_y_max_) area_y_max_ = d;
}
void ObjectTracking::on_doubleSpinBoxAreaZMin_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  if(d != area_z_min_) area_z_min_ = d;
}
void ObjectTracking::on_doubleSpinBoxAreaZMax_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  if(d != area_z_max_) area_z_max_ = d;
}

ObjectTracking* g_object_tracking = NULL;
bool g_initialized = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_object_tracking->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_hand_tracking", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);



  ObjectTracking w;
  g_object_tracking = &w;
  w.show();

  g_initialized = w.initialize();
  if(!g_initialized)
    exit(-1);

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
