#ifndef RPW_OBJECT_TRACKING_H
#define RPW_OBJECT_TRACKING_H

#include <prw_object_tracking/object_tracking_utils.h>
#include "ui_object_tracking.h"
#include "ui_color_object_selector.h"

#include <velib/view.h>
#include <velib/scene.h>
#include <velib/node.h>
#include <velib/opencv_image.h>
#include <velib/color_selector.h>

#include <qmutex.h>
#include <qtimer.h>
#include <qcolordialog.h>

#include <boost/circular_buffer.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <dynamic_reconfigure/server.h>

namespace prw
{


class ObjectTracking : public QMainWindow
{
  Q_OBJECT
public:

  ObjectTracking(ros::NodeHandle & nh, QWidget *parent = 0, Qt::WFlags flags = 0);
  ~ObjectTracking();




public slots:
  void onImageUpdate();
  void onImageClicked(QPointF point, QRgb color);
  void onColorPickerAdd();
  void onColorListClicked(QListWidgetItem * item);
  void onColorPickerSelected(int h, int s, int v);
  void onLuminanceRangeUpdate(int v_min, int v_max);
  void onRangeUpdate();
  void onSizeUpdate();


signals:


protected:
  //UI
  void updateColorPickerRange(const ve::ColorRange& range);
  void updateColorObjectTracker(const QString& name, const ve::ColorRange& range);


  //Qt
  void closeEvent(QCloseEvent *event);

  //ROS
  void cameraCallback(const sensor_msgs::PointCloud2ConstPtr& message);




public:
  ros::NodeHandle& nh_;
  ros::Subscriber cloud_subscriber_;
  image_transport::Publisher image_publisher_;
  ros::Publisher cloud_publisher_;
  ros::Publisher object_publisher_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
  cv_bridge::CvImagePtr bridge_;

  std::vector<ObjectTrackerPtr> object_trackers_;

  typedef QMap<QString, SimpleColorObjectTracker> SimpleColorObjectMap;
  SimpleColorObjectMap color_object_maps_;
  QMutex color_object_mutex_;

  //LutColorObjectTraker lut_color_object_tracker_;

  pcl::PassThrough<pcl::PointXYZRGB> pass_through_filter_;
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter_;
  pcl::SACSegmentation<pcl::PointXYZRGB> seg_;
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec_;
  pcl::PCDWriter writer;


  Ui::ObjectTracking ui;
  Ui::ColorObjectSelector ui_color_object_;
  bool quit_threads_;
  QMutex ui_mutex_;

  ve::View* view_;
  ve::Scene* scene_;
  QVector<QGraphicsItem*> added_graphics_items_;
  //QGraphicsView* view_;
  //QGraphicsScene* scene_;
  ve::OpenCVImage* scene_image_;
  boost::circular_buffer<cv::Mat> image_buffer_;
  boost::circular_buffer<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cloud_buffer_;

  std::string output_frame_;
  double filter_limit_min_z_;
  double filter_limit_max_z_;
  double filter_limit_min_y_;
  double filter_limit_max_y_;
  double filter_limit_min_x_;
  double filter_limit_max_x_;
  bool filter_limit_negative_;
  double leaf_size_;

  //voxel grid






  QTimer *image_timer_;

  //QColorDialog* color_dialog_;
  ve::ColorPicker* color_picker_;
  ve::ColorLuminancePicker* color_luminance_picker_;
  QDialog* color_object_selector_;

};

}


#endif
