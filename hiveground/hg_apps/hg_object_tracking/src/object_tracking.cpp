#include <ros/ros.h>
#include <hg_object_tracking/object_tracking.h>
#include <QtGui/QApplication>
#include <QDebug>

#include <boost/thread.hpp>


using namespace prw;

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

  cloud_publisher_ = nh_private_.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);
  cloud_subscriber_ = nh_private_.subscribe("cloud_in", 1, &ObjectTracking::cloudCallback, this);
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

  int j = 0;
  float cx, cy, cz;
  float cr, cg, cb;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    cx = cy = cz = 0.0;
    cr = cg = cb = 0;

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster->points.push_back(in->points[*pit]);

      //compute centroid
      cx += in->points[*pit].x;
      cy += in->points[*pit].y;
      cz += in->points[*pit].z;

      //get color color
      unsigned char * rgb = (unsigned char *)&(in->points[j].rgb);
      cr += rgb[0];
      cg += rgb[1];
      cb += rgb[2];

    }
    (*out) += (*cloud_cluster);
    int size = cloud_cluster->points.size();
    cx = cx / size;
    cy = cy / size;
    cz = cz / size;
    cr = cr / size;
    cg = cg / size;
    cb = cb / size;
  }
  ROS_DEBUG_THROTTLE(1.0, "total object %d", (int)cluster_indices.size());
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
