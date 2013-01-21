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


  nh_private_.getParam("arm_min_cluster_size", i);
  ui.spinBoxArmMinSize->setValue(i);
  nh_private_.getParam("plam_max_cluster_size", i);
  ui.spinBoxPlamMaxSize->setValue(i);
  nh_private_.getParam("arm_eigen_value_ratio", d);
  ui.doubleSpinArmEigenRatio->setValue(d);

  cloud_publisher_ = nh_private_.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);
  cloud_subscriber_ = nh_private_.subscribe("cloud_in", 1, &ObjectTracking::cloudCallback, this);
  marker_publisher_ = nh_private_.advertise<Marker>("tracked_object", 128);
  marker_array_publisher_ = nh_private_.advertise<MarkerArray>("tracked_object_array", 128);
  hands_publisher_ = nh_private_.advertise<Hands>("hands", 128);

  return true;
}


void ObjectTracking::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message)
{
  QMutexLocker lock(&mutex_cloud_);
  marker_array_.markers.clear();
  marker_id_ = 0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*message, *cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planar(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_objects(new pcl::PointCloud<pcl::PointXYZRGB>);
  sacSegmentation(cloud, cloud_planar, cloud_objects);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clustered_clouds;
  objectSegmentation(cloud_objects, clustered_clouds);

  //Detect hand
  hg_object_tracking::Hands hands_message;
  detectHands(clustered_clouds, hands_message);

  if((hands_publisher_.getNumSubscribers() != 0) && (hands_message.hands.size() != 0))
  {
    hands_message.header.stamp = message->header.stamp;
    hands_message.header.frame_id = message->header.frame_id;
    hands_publisher_.publish(hands_message);
  }

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
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      for(int i = 0; i < (int)clustered_clouds.size(); i++)
      {
        *cloud += *clustered_clouds[i];
      }
      pcl::toROSMsg(*cloud, output_cloud);
    }

    output_cloud.header.stamp = message->header.stamp;
    output_cloud.header.frame_id = message->header.frame_id;
    cloud_publisher_.publish(output_cloud);
  }

  if((marker_array_publisher_.getNumSubscribers() != 0) && (!marker_array_.markers.empty()))
  {
    marker_array_publisher_.publish(marker_array_);
  }
}

void ObjectTracking::sacSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_planar,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_objects)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZRGB> sac_segmentator;
  sac_segmentator.setOptimizeCoefficients(true);
  sac_segmentator.setModelType(pcl::SACMODEL_PLANE);
  sac_segmentator.setMethodType(pcl::SAC_RANSAC);
  sac_segmentator.setMaxIterations (1000);
  sac_segmentator.setDistanceThreshold(sac_distance_threshold_);
  sac_segmentator.setInputCloud(in->makeShared());
  sac_segmentator.segment(*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    ROS_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }
  else
  {
    ROS_DEBUG("Indices size: %d", (int)inliers->indices.size ());
  }

  pcl::ExtractIndices<pcl::PointXYZRGB> indices_extractor;
  indices_extractor.setNegative(false);
  indices_extractor.setInputCloud(in);
  indices_extractor.setIndices(inliers);
  indices_extractor.filter(*out_planar);

  indices_extractor.setNegative(true);
  indices_extractor.filter(*out_objects);
}

void ObjectTracking::objectSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                                             std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& out)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(in);
  std::vector<pcl::PointIndices> cluster_indices;


  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec_extractor;
  ec_extractor.setClusterTolerance(ec_cluster_tolerance_);
  ec_extractor.setMinClusterSize(ec_min_cluster_size_);
  ec_extractor.setMaxClusterSize(ec_max_cluster_size_);
  ec_extractor.setSearchMethod(tree);
  ec_extractor.setInputCloud(in);
  ec_extractor.extract(cluster_indices);

  ROS_DEBUG_THROTTLE(1.0, "total object %d", (int)cluster_indices.size());

  MarkerArray marker_array;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster->points.push_back(in->points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    out.push_back(cloud_cluster);
  }
}

void ObjectTracking::detectHands(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clustered_clouds,
                                     hg_object_tracking::Hands& hands)
{
  hands.hands.clear();
  pcl::PCA<pcl::PointXYZRGB> pca;
  for(int i = 0; i < (int)clustered_clouds.size(); i++)
  {
    if((int)clustered_clouds[i]->size() < arm_min_cluster_size_) continue;

    pca.setInputCloud(clustered_clouds[i]);
    Eigen::Vector4f mean = pca.getMean();
    if((mean.coeff(0) < area_x_min_) || (mean.coeff(0) > area_x_max_)) continue;
    if((mean.coeff(1) < area_y_min_) || (mean.coeff(1) > area_y_max_)) continue;
    if((mean.coeff(2) < area_z_min_) || (mean.coeff(2) > area_z_max_)) continue;

    ROS_DEBUG_STREAM_THROTTLE(1.0, "arm value:" << pca.getEigenValues());
    ROS_DEBUG_STREAM_THROTTLE(1.0, "arm vector:" << pca.getEigenVectors());
    ROS_DEBUG_STREAM_THROTTLE(1.0, "arm mean:" << pca.getMean());

    //check eigen value for elongate object
    Eigen::Vector3f eigen_value = pca.getEigenValues();
    if(eigen_value.coeff(0) < (arm_eigen_ratio_*eigen_value.coeff(1))) continue;


    if(ui.checkBoxShowClusterMarker->isChecked())
      pushHandMarker(pca);
    if(ui.checkBoxShowEigenVector->isChecked())
      pushEigenMarker(pca);

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(clustered_clouds[i]);

    pcl::PointXYZRGB search_point;
    Eigen::Matrix3f ev = pca.getEigenVectors();
    Eigen::Vector3f axis_x(ev.coeff(0, 0), ev.coeff(1, 0), ev.coeff(2, 0));
    axis_x = ((-axis_x) * 0.3) + Eigen::Vector3f(mean.coeff(0), mean.coeff(1), mean.coeff(2));
    ROS_DEBUG_STREAM_THROTTLE(1.0, "axis_x:" << axis_x);
    pushSimpleMarker(axis_x.coeff(0), axis_x.coeff(1), axis_x.coeff(2));
    search_point.x = axis_x.coeff(0);
    search_point.y = axis_x.coeff(1);
    search_point.z = axis_x.coeff(2);

    int K = plam_max_cluster_size_;
    if(K > (int)clustered_clouds[i]->points.size())
      K = clustered_clouds[i]->points.size();

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if ( kdtree.nearestKSearch (search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      ROS_DEBUG_THROTTLE(1.0, "found %lu", pointIdxNKNSearch.size ());
      for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j)
      {
        hand_cloud->points.push_back(clustered_clouds[i]->points[ pointIdxNKNSearch[j] ]);
        clustered_clouds[i]->points[ pointIdxNKNSearch[j] ].r = 255;
        clustered_clouds[i]->points[ pointIdxNKNSearch[j] ].g = 0;
        clustered_clouds[i]->points[ pointIdxNKNSearch[j] ].b = 0;
      }
      hand_cloud->width = hand_cloud->points.size();
      hand_cloud->height = 1;
      hand_cloud->is_dense = true;
      hg_object_tracking::Hand hand;

      hand.arm_centroid.x = mean.coeff(0);
      hand.arm_centroid.y = mean.coeff(1);
      hand.arm_centroid.z = mean.coeff(2);
      hand.arm_eigen_value.x = eigen_value(0);
      hand.arm_eigen_value.y = eigen_value(1);
      hand.arm_eigen_value.z = eigen_value(2);
      Eigen::Matrix3f ev = pca.getEigenVectors();
      geometry_msgs::Vector3 v;
      v.x = ev.coeff(0, 0);
      v.y = ev.coeff(1, 0);
      v.z = ev.coeff(2, 0);
      hand.arm_eigen_vectors.push_back(v);
      v.x = ev.coeff(0, 1);
      v.y = ev.coeff(1, 1);
      v.z = ev.coeff(2, 1);
      hand.arm_eigen_vectors.push_back(v);
      v.x = ev.coeff(0, 2);
      v.y = ev.coeff(1, 2);
      v.z = ev.coeff(2, 2);
      hand.arm_eigen_vectors.push_back(v);

      detectFingers(hand_cloud, hand);
      hands.hands.push_back(hand);
    }
  }
}

void ObjectTracking::detectFingers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud,
                                        hg_object_tracking::Hand& hand)
{
  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(hand_cloud);
  Eigen::Vector4f mean = pca.getMean();
  Eigen::Vector3f eigen_value = pca.getEigenValues();

  ROS_DEBUG_STREAM_THROTTLE(1.0, "hand value:" << pca.getEigenValues());
  ROS_DEBUG_STREAM_THROTTLE(1.0, "hand vector:" << pca.getEigenVectors());
  ROS_DEBUG_STREAM_THROTTLE(1.0, "hand mean:" << pca.getMean());

  if(ui.checkBoxShowClusterMarker->isChecked())
    pushHandMarker(pca, 0.05);
  if(ui.checkBoxShowEigenVector->isChecked())
    pushEigenMarker(pca, 1.0);

  hand.hand_centroid.x = mean.coeff(0);
  hand.hand_centroid.y = mean.coeff(1);
  hand.hand_centroid.z = mean.coeff(2);
  hand.hand_eigen_value.x = eigen_value(0);
  hand.hand_eigen_value.y = eigen_value(1);
  hand.hand_eigen_value.z = eigen_value(2);

  Eigen::Matrix3f ev = pca.getEigenVectors();
  geometry_msgs::Vector3 v;
  v.x = ev.coeff(0, 0);
  v.y = ev.coeff(1, 0);
  v.z = ev.coeff(2, 0);
  hand.arm_eigen_vectors.push_back(v);
  v.x = ev.coeff(0, 1);
  v.y = ev.coeff(1, 1);
  v.z = ev.coeff(2, 1);
  hand.arm_eigen_vectors.push_back(v);
  v.x = ev.coeff(0, 2);
  v.y = ev.coeff(1, 2);
  v.z = ev.coeff(2, 2);
  hand.arm_eigen_vectors.push_back(v);

}

void ObjectTracking::pushHandMarker(pcl::PCA<pcl::PointXYZRGB>& pca, double scale)
{
  Marker marker;
  marker.type = Marker::SPHERE;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = "base_link";
  marker.ns = "detected_hands";
  marker.id = marker_id_++;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  marker.pose.position.x = pca.getMean().coeff(0);
  marker.pose.position.y = pca.getMean().coeff(1);
  marker.pose.position.z = pca.getMean().coeff(2);
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker_array_.markers.push_back(marker);
}

void ObjectTracking::pushEigenMarker(pcl::PCA<pcl::PointXYZRGB>& pca, double scale)
{
  Marker marker;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = "base_link";
  marker.ns = "cluster_eigen";
  marker.type = Marker::ARROW;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.pose.position.x = pca.getMean().coeff(0);
  marker.pose.position.y = pca.getMean().coeff(1);
  marker.pose.position.z = pca.getMean().coeff(2);

  Eigen::Quaternionf qx, qy, qz;
  Eigen::Matrix3f ev = pca.getEigenVectors();
  Eigen::Vector3f axis_x(ev.coeff(0, 0), ev.coeff(1, 0), ev.coeff(2, 0));
  Eigen::Vector3f axis_y(ev.coeff(0, 1), ev.coeff(1, 1), ev.coeff(2, 1));
  Eigen::Vector3f axis_z(ev.coeff(0, 2), ev.coeff(1, 2), ev.coeff(2, 2));
  qx.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_x);
  qy.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_y);
  qz.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_z);

  marker.id = marker_id_++;
  marker.scale.z = pca.getEigenValues().coeff(0) * scale;
  marker.pose.orientation.x = qx.x();
  marker.pose.orientation.y = qx.y();
  marker.pose.orientation.z = qx.z();
  marker.pose.orientation.w = qx.w();
  marker.color.b = 0.0;
  marker.color.g = 0.0;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  //ROS_INFO_STREAM(marker);
  marker_array_.markers.push_back(marker);

  marker.id = marker_id_++;
  marker.scale.z = pca.getEigenValues().coeff(1) * 0.1;
  marker.pose.orientation.x = qy.x();
  marker.pose.orientation.y = qy.y();
  marker.pose.orientation.z = qy.z();
  marker.pose.orientation.w = qy.w();
  marker.color.b = 0.0;
  marker.color.g = 1.0;
  marker.color.r = 0.0;
  marker_array_.markers.push_back(marker);

  marker.id = marker_id_++;
  marker.scale.z = pca.getEigenValues().coeff(2) * 0.1;
  marker.pose.orientation.x = qz.x();
  marker.pose.orientation.y = qz.y();
  marker.pose.orientation.z = qz.z();
  marker.pose.orientation.w = qz.w();
  marker.color.b = 1.0;
  marker.color.g = 0.0;
  marker.color.r = 0.0;
  marker_array_.markers.push_back(marker);
}

void ObjectTracking::pushSimpleMarker(double x, double y, double z,
                                           double r, double g, double b)
{
  Marker marker;
  marker.type = Marker::SPHERE;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = "base_link";
  marker.ns = "simple_marker";
  marker.id = marker_id_++;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker_array_.markers.push_back(marker);
}


void ObjectTracking::on_doubleSpinBoxSacDistance_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  if(d != sac_distance_threshold_)
    sac_distance_threshold_ = d;
}

void ObjectTracking::on_doubleSpinBoxEcTolerance_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  ec_cluster_tolerance_ = d;
}

void ObjectTracking::on_spinBoxEcMinSize_valueChanged(int d)
{
  QMutexLocker lock(&mutex_cloud_);
  ec_min_cluster_size_ = d;
}

void ObjectTracking::on_spinBoxEcMaxSize_valueChanged(int d)
{
  QMutexLocker lock(&mutex_cloud_);
  ec_max_cluster_size_ = d;
}

void ObjectTracking::on_doubleSpinBoxAreaXMin_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  area_x_min_ = d;
}
void ObjectTracking::on_doubleSpinBoxAreaXMax_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  area_x_max_ = d;
}
void ObjectTracking::on_doubleSpinBoxAreaYMin_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  area_y_min_ = d;
}
void ObjectTracking::on_doubleSpinBoxAreaYMax_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  area_y_max_ = d;
}
void ObjectTracking::on_doubleSpinBoxAreaZMin_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  area_z_min_ = d;
}
void ObjectTracking::on_doubleSpinBoxAreaZMax_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  area_z_max_ = d;
}

void ObjectTracking::on_spinBoxArmMinSize_valueChanged(int d)
{
  QMutexLocker lock(&mutex_cloud_);
  arm_min_cluster_size_ = d;
}

void ObjectTracking::on_spinBoxPlamMaxSize_valueChanged(int d)
{
  QMutexLocker lock(&mutex_cloud_);
  plam_max_cluster_size_ = d;
}

void ObjectTracking::on_doubleSpinArmEigenRatio_valueChanged(double d)
{
  QMutexLocker lock(&mutex_cloud_);
  arm_eigen_ratio_ = d;
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
