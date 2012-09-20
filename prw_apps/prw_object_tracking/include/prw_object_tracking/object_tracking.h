#ifndef RPW_OBJECT_TRACKING_H
#define RPW_OBJECT_TRACKING_H

#include "ui_object_tracking.h"
#include "ui_color_object_selector.h"
#include "ui_object_segmentation.h"

#include <qmutex.h>

#include <velib/view.h>
#include <velib/scene.h>
#include <velib/node.h>
#include <velib/opencv_image.h>
#include <velib/color_selector.h>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv2/opencv.hpp>

#include <boost/circular_buffer.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>

#include <prw_message/ObjectArray.h>

class ObjectTracking : public QMainWindow
{
  Q_OBJECT
public:
  ObjectTracking(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~ObjectTracking();

  bool initialize();

protected:
  void connectUi();

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message);

  void voxelFilter(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr out);
  void sacSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_planar,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_objects);

  void objectSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr out);


  void closeEvent(QCloseEvent *event);

protected Q_SLOTS:
  void onObjectSegmentationDialogUpdate();






public:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  bool quit_threads_;

private:
  Ui::ObjectTracking ui;
  ve::View* view_;
  ve::Scene* scene_;
  ve::OpenCVImage* scene_image_;

  QDialog* dialog_object_segmentation_;
  Ui::ObjectSegmentDialog ui_object_segmentation_;

  std::string camera_name_;
  ros::Publisher cloud_publisher_;
  image_transport::Publisher image_publisher_;
  ros::Publisher object_publisher_;

  ros::Subscriber cloud_subscriber_;
  ros::Subscriber camera_info_subscriber_;

  tf::TransformBroadcaster transform_broadcaster_;
  tf::TransformListener transform_listener_;

  //cloud filter/object segmentation
  QMutex object_tracking_mutex_;
  std::string output_frame_;
  double filter_limit_min_x_;
  double filter_limit_max_x_;
  double filter_limit_min_y_;
  double filter_limit_max_y_;
  double filter_limit_min_z_;
  double filter_limit_max_z_;
  bool filter_limit_negative_;
  double leaf_size_;
  double ec_distance_threshold_;
  double ec_cluster_tolerance_;
  int ec_min_size_;
  int ec_max_size_;

  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter_;
  pcl::SACSegmentation<pcl::PointXYZRGB> sac_segmentator_;
  pcl::ExtractIndices<pcl::PointXYZRGB> indices_extractor_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec_extractor_;
  pcl::PCDWriter cloud_writer_;
};



#endif
