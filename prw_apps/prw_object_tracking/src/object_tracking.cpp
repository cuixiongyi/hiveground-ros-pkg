#include <prw_object_tracking/object_tracking.h>


#include <QDebug>
#include <qcolordialog.h>
#include <qevent.h>
#include <qinputdialog.h>

#include <cv_bridge/cv_bridge.h>




using namespace cv;

ObjectTracking::ObjectTracking(QWidget *parent, Qt::WFlags flags) :
     QMainWindow(parent, flags),
     nh_private_("~"),
     quit_threads_(false)
{
  ui.setupUi(this);

  view_ = new ve::View(this);
  ui.gridLayoutMain->addWidget(view_, 0, 0);

  scene_ = new ve::Scene(view_);
  scene_->setSceneRect(0, 0, 800, 600);
  view_->setScene(scene_);
  scene_->set_mode(ve::Scene::MODE_CURSOR);
  scene_image_ = new ve::OpenCVImage();
  scene_->addItem(scene_image_);
  scene_image_->setPos(100, 100);
  view_->ensureVisible(scene_->itemsBoundingRect());

  dialog_object_segmentation_ = new QDialog(this);
  ui_object_segmentation_.setupUi(dialog_object_segmentation_);
  dialog_object_segmentation_->show();
}

ObjectTracking::~ObjectTracking()
{

}

bool ObjectTracking::initialize()
{
  cloud_publisher_ = nh_private_.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);
  image_publisher_ = image_transport::ImageTransport(nh_private_).advertise("image_output", 1);
  object_publisher_ = nh_private_.advertise<prw_message::ObjectArray>("object_output", 1);


  nh_private_.param("camera_name", camera_name_, string("/camera"));
  ROS_INFO_STREAM("camera_name " << camera_name_);

  cloud_subscriber_ = nh_.subscribe(camera_name_ + "/depth_registered/points", 1, &ObjectTracking::cloudCallback, this);
  camera_info_subscriber_ = nh_.subscribe(camera_name_ + "/rgb/camera_info", 1, &ObjectTracking::cameraInfoCallback, this);

  ROS_ASSERT(nh_private_.getParam("output_frame", output_frame_));
  ROS_ASSERT(nh_private_.getParam("filter_limit_min_z", filter_limit_min_z_));
  ROS_ASSERT(nh_private_.getParam("filter_limit_max_z", filter_limit_max_z_));
  ROS_ASSERT(nh_private_.getParam("filter_limit_min_y", filter_limit_min_y_));
  ROS_ASSERT(nh_private_.getParam("filter_limit_max_y", filter_limit_max_y_));
  ROS_ASSERT(nh_private_.getParam("filter_limit_min_x", filter_limit_min_x_));
  ROS_ASSERT(nh_private_.getParam("filter_limit_max_x", filter_limit_max_x_));
  ROS_ASSERT(nh_private_.getParam("filter_limit_negative", filter_limit_negative_));
  ROS_ASSERT(nh_private_.getParam("leaf_size", leaf_size_));
  ROS_ASSERT(nh_private_.getParam("ec_distance_threshold", ec_distance_threshold_));
  ROS_ASSERT(nh_private_.getParam("ec_cluster_tolerance", ec_cluster_tolerance_));
  ROS_ASSERT(nh_private_.getParam("ec_min_size", ec_min_size_));
  ROS_ASSERT(nh_private_.getParam("ec_max_size", ec_max_size_));

  ui_object_segmentation_.voxel_output_frame->setText(output_frame_.c_str());
  ui_object_segmentation_.voxel_min_x->setValue(filter_limit_min_x_);
  ui_object_segmentation_.voxel_min_y->setValue(filter_limit_min_y_);
  ui_object_segmentation_.voxel_min_z->setValue(filter_limit_min_z_);
  ui_object_segmentation_.voxel_max_x->setValue(filter_limit_max_x_);
  ui_object_segmentation_.voxel_max_y->setValue(filter_limit_max_y_);
  ui_object_segmentation_.voxel_max_z->setValue(filter_limit_max_z_);
  ui_object_segmentation_.voxel_leaf_size->setValue(leaf_size_);
  ui_object_segmentation_.voxel_limit_negative->setChecked(filter_limit_negative_);
  ui_object_segmentation_.ec_distance_threshold->setValue(ec_distance_threshold_);
  ui_object_segmentation_.ec_tolerance->setValue(ec_cluster_tolerance_);
  ui_object_segmentation_.ec_min_size->setValue(ec_min_size_);
  ui_object_segmentation_.ec_max_size->setValue(ec_max_size_);

  onObjectSegmentationDialogUpdate();


  sac_segmentator_.setOptimizeCoefficients(true);
  sac_segmentator_.setModelType(pcl::SACMODEL_PLANE);
  sac_segmentator_.setMethodType(pcl::SAC_RANSAC);


  connectUi();

  return true;
}

void ObjectTracking::connectUi()
{
  connect(ui_object_segmentation_.voxel_output_frame, SIGNAL(textChanged(QString)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.voxel_min_x, SIGNAL(valueChanged(double)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.voxel_min_y, SIGNAL(valueChanged(double)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.voxel_min_z, SIGNAL(valueChanged(double)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.voxel_max_x, SIGNAL(valueChanged(double)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.voxel_max_y, SIGNAL(valueChanged(double)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.voxel_max_z, SIGNAL(valueChanged(double)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.voxel_leaf_size, SIGNAL(valueChanged(double)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.voxel_limit_negative, SIGNAL(toggled(bool)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.ec_distance_threshold, SIGNAL(valueChanged(double)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.ec_tolerance, SIGNAL(valueChanged(double)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.ec_min_size, SIGNAL(valueChanged(int)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
  connect(ui_object_segmentation_.ec_max_size, SIGNAL(valueChanged(int)), this,
          SLOT(onObjectSegmentationDialogUpdate()));
}

void ObjectTracking::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message)
{
  ros::Time start_time = ros::Time(ros::WallTime::now().toSec());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

  // convert cloud to PCL cloud
  pcl::fromROSMsg(*message, *cloud);

  // convert cloud to opencv image
  sensor_msgs::Image image_message;
  pcl::toROSMsg(*cloud, image_message);
  cv_bridge::CvImagePtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(image_message, image_message.encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform RGB image");
    return;
  }

  voxelFilter(cloud, cloud_filtered);

  pcl_ros::transformPointCloud(output_frame_, *cloud_filtered, *cloud_transformed, transform_listener_);

  sacSegmentation(cloud_transformed);

  sensor_msgs::PointCloud2 output_cloud;
  pcl::toROSMsg(*cloud_transformed, output_cloud);
  output_cloud.header.stamp = ros::Time::now();
  cloud_publisher_.publish(output_cloud);

  image_message.header.stamp = ros::Time::now();
  image_publisher_.publish(image_message);

  ROS_INFO_STREAM_THROTTLE(1.0, ros::Time(ros::WallTime::now().toSec()) - start_time);
}

void ObjectTracking::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message)
{

}

void ObjectTracking::voxelFilter(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>());

  QMutexLocker locker(&object_tracking_mutex_);
  voxel_filter_.setInputCloud(in);
  voxel_filter_.setFilterFieldName("z");
  voxel_filter_.setFilterLimits(filter_limit_min_z_, filter_limit_max_z_);
  voxel_filter_.filter(*out);

  voxel_filter_.setInputCloud(out);
  voxel_filter_.setFilterFieldName("y");
  voxel_filter_.setFilterLimits(filter_limit_min_y_, filter_limit_max_y_);
  voxel_filter_.filter(*cloud_tmp);

  voxel_filter_.setInputCloud(cloud_tmp);
  voxel_filter_.setFilterFieldName("x");
  voxel_filter_.setFilterLimits(filter_limit_min_x_, filter_limit_max_x_);
  voxel_filter_.filter(*out);
}

void ObjectTracking::sacSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in)
{
  QMutexLocker locker(&object_tracking_mutex_);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  int i = 0, nr_points = (int)in->points.size();

  while (in->points.size() > 0.3 * nr_points)
  {
    sac_segmentator_.setInputCloud(in);
    sac_segmentator_.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      return;
    }

    // Extract the inliers
    indices_extractor_.setInputCloud(in);
    indices_extractor_.setIndices(inliers);
    indices_extractor_.setNegative(false);
    indices_extractor_.filter(*cloud_p);
    //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points."   << std::endl;

    if (ui.actionSaveCloud->isChecked())
    {
      std::stringstream ss;
      ss << "scene_plane_" << i << ".pcd";
      ROS_INFO_STREAM("saved " << ss.str());
      cloud_writer_.write < pcl::PointXYZRGB > (ss.str(), *cloud_p, false);
    }

    // Create the filtering object
    indices_extractor_.setNegative(true);
    indices_extractor_.filter(*cloud_f);
    in.swap(cloud_f);
    i++;
  }
}

void ObjectTracking::objectSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out)
{
  QMutexLocker locker(&object_tracking_mutex_);
  int j = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;
  float cx, cy, cz;
  float cr, cg, cb;
}

void ObjectTracking::closeEvent(QCloseEvent *event)
{
  ROS_INFO("Close windows");
  quit_threads_ = true;
  event->accept();
}

void ObjectTracking::onObjectSegmentationDialogUpdate()
{
  QMutexLocker locker(&object_tracking_mutex_);

  if (ui_object_segmentation_.voxel_limit_negative->isChecked() != voxel_filter_.getFilterLimitsNegative())
  {
    filter_limit_negative_ = ui_object_segmentation_.voxel_limit_negative->isChecked();
    voxel_filter_.setFilterLimitsNegative(filter_limit_negative_);
  }

  if (ui_object_segmentation_.voxel_leaf_size->value() != voxel_filter_.getLeafSize()[0])
  {
    leaf_size_ = ui_object_segmentation_.voxel_leaf_size->value();
    voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  }

  filter_limit_min_x_ = ui_object_segmentation_.voxel_min_x->value();
  filter_limit_min_y_ = ui_object_segmentation_.voxel_min_y->value();
  filter_limit_min_z_ = ui_object_segmentation_.voxel_min_z->value();
  filter_limit_max_x_ = ui_object_segmentation_.voxel_max_x->value();
  filter_limit_max_y_ = ui_object_segmentation_.voxel_max_y->value();
  filter_limit_max_z_ = ui_object_segmentation_.voxel_max_z->value();

  if (ui_object_segmentation_.ec_distance_threshold->value() != sac_segmentator_.getDistanceThreshold())
  {
    ec_distance_threshold_ = ui_object_segmentation_.ec_distance_threshold->value();
    sac_segmentator_.setDistanceThreshold(ec_distance_threshold_);
  }

  if (ui_object_segmentation_.ec_tolerance->value() != ec_extractor_.getClusterTolerance())
  {
    ec_cluster_tolerance_ = ui_object_segmentation_.ec_tolerance->value();
    ec_extractor_.setClusterTolerance(ec_cluster_tolerance_); // 2cm
  }

  if (ui_object_segmentation_.ec_min_size->value() != ec_extractor_.getMinClusterSize())
  {
    ec_min_size_ = ui_object_segmentation_.ec_min_size->value();
    ec_extractor_.setMinClusterSize(ec_min_size_);
  }


  if (ui_object_segmentation_.ec_max_size->value() != ec_extractor_.getMaxClusterSize())
  {
    ec_max_size_ = ui_object_segmentation_.ec_max_size->value();
    ec_extractor_.setMaxClusterSize(ec_max_size_);
  }
}
