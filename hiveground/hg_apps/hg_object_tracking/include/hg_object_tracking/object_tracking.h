#ifndef RPW_HAND_TRACKING_H
#define RPW_HAND_TRACKING_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

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

#include <qmutex.h>

#include "ui_object_tracking.h"

namespace prw
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
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr out);

protected Q_SLOTS:
  void on_doubleSpinBoxSacDistance_valueChanged(double d);
  void on_doubleSpinBoxEcTolerance_valueChanged(double d);
  void on_spinBoxEcMinSize_valueChanged(int d);
  void on_spinBoxEcMaxSize_valueChanged(int d);


public:
  Ui::ObjectTracking ui;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  bool quit_threads_;

  ros::Publisher cloud_publisher_;
  ros::Subscriber cloud_subscriber_;
  pcl::SACSegmentation<pcl::PointXYZRGB> sac_segmentator_;
  pcl::ExtractIndices<pcl::PointXYZRGB> indices_extractor_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec_extractor_;

  QMutex mutex_cloud_;


};

}


#endif
