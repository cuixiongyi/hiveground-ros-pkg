#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>




#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <dynamic_reconfigure/server.h>
#include <prw_scene_copier/ObjectDetectorConfig.h>

using namespace std;


//[ INFO] [1342407141.664144436]: [-0.7414378772801097; -0.6604714858578639; 1.866349629472909]
//[ INFO] [1342407141.664168804]: [-0.002052093178870036; 0.1134341257555175; 0.01998161737785098]
//Top Kinect 2012/06/17
static double xyz_[3] = {-0.7414378772801097, -0.6604714858578639, 1.866349629472909};
static double rpy_[3] = {-0.002052093178870036, 0.1134341257555175, 0.01998161737785098};


int key_;
tf::Transform tf_;
Eigen::Matrix4f tf_matrix_;

static const string WINDOW_INPUT = "Input Image";


class ObjectDetector
{
public:
  //typedef prw_scene_copier::object_detectorConfig Config;
  //typedef dynamic_reconfigure::Server<Config> ReconfigureServer;



  ObjectDetector(ros::NodeHandle & nh)
    : nh_(nh)
  {


    //publisher
    image_publisher_ = image_transport::ImageTransport(nh_).advertise("image", 1);

    //subscriber
    cloud_subscriber_ = nh_.subscribe("/kinect7051A/depth_registered/points", 1, &ObjectDetector::cameraCallback, this);
    cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);




    // initialize dynamic reconfigure
    reconfigure_server_ = boost::make_shared <dynamic_reconfigure::Server<prw_scene_copier::ObjectDetectorConfig> > (nh_);
    dynamic_reconfigure::Server<prw_scene_copier::ObjectDetectorConfig>::CallbackType f
    =  boost::bind (&ObjectDetector::configCallback, this, _1, _2);
    reconfigure_server_->setCallback (f);
    //reconfigure_server_.reset (new ReconfigureServer (reconfigure_mutex_, nh_));
    //reconfigure_server_->setCallback (boost::bind (&ObjectDetector::configCallback, this, _1, _2));

    tf::Quaternion qt = tf::createQuaternionFromRPY(rpy_[0], rpy_[1], rpy_[2]);
    tf::Vector3 vec(xyz_[0], xyz_[1], xyz_[2]);
    qt *= tf::createQuaternionFromRPY(M_PI, 0, M_PI/2.0);
    tf_ = tf::Transform(qt, vec);
    tf_ = tf_.inverse();

    ROS_INFO_STREAM(tf_.getOrigin() << ":" << tf_.getRotation());


    //tf_.setRotation(tf::createQuaternionFromRPY(0, 0, 0));



    pcl_ros::transformAsMatrix(tf_, tf_matrix_);



  }


  ~ObjectDetector()
  {

  }

  void cameraCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    //int key = cv::waitKey(3);


    sensor_msgs::Image image;

    sensor_msgs::PointCloud2::Ptr cloud_transformed (new sensor_msgs::PointCloud2 ());
    sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());
    sensor_msgs::PointCloud2::Ptr cloud_filtered2 (new sensor_msgs::PointCloud2 ());

    pcl_ros::transformPointCloud(tf_matrix_, *msg, *cloud_transformed);

    voxel_grid_filter_.setInputCloud(cloud_transformed);
    voxel_grid_filter_.filter(*cloud_filtered);


    //sor_filter_.setInputCloud(cloud_filtered);
    //voxel_grid_filter_.filter(*cloud_filtered2);


    // convert cloud to PCL
    pcl::PointCloud <pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*cloud_transformed, cloud);

    // get an OpenCV image from the cloud
    pcl::toROSMsg(cloud, image);

    try
    {
      bridge_ = cv_bridge::toCvCopy(image, image.encoding);
      ROS_INFO_STREAM_THROTTLE(10.0, "New depth_registered/cloud." << " " << image.encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Conversion failed");
      return;
    }

    //get
    //cv::Mat image_gray;
    //cv::cvtColor(bridge_->image, image_gray, CV_BGR2GRAY);

















    cv::imshow(WINDOW_INPUT, bridge_->image);

    if(cfg_output_image_)
    {
      image_publisher_.publish( bridge_->toImageMsg() );
    }

    if(cfg_output_cloud_)
    {
      cloud_publisher_.publish(*cloud_filtered);
    }


  }


  void configCallback (prw_scene_copier::ObjectDetectorConfig &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cfg_output_image_  = config.output_image;
    cfg_output_cloud_  = config.output_cloud;

    //ROS_INFO_STREAM("output_image_: " << cfg_output_image_);

    Eigen::Vector3f leaf_size = voxel_grid_filter_.getLeafSize ();

    if (leaf_size[0] != config.leaf_size)
    {
      leaf_size.setConstant (config.leaf_size);
      ROS_DEBUG ("[config_callback] Setting the downsampling leaf size to: %f.", leaf_size[0]);
      voxel_grid_filter_.setLeafSize (leaf_size[0], leaf_size[1], leaf_size[2]);
    }


    double filter_min, filter_max;
    voxel_grid_filter_.getFilterLimits (filter_min, filter_max);
    if (filter_min != config.filter_limit_min)
    {
      filter_min = config.filter_limit_min;
      ROS_DEBUG ("[config_callback] Setting the minimum filtering value a point will be considered from to: %f.", filter_min);
    }
    if (filter_max != config.filter_limit_max)
    {
      filter_max = config.filter_limit_max;
      ROS_DEBUG ("[config_callback] Setting the maximum filtering value a point will be considered from to: %f.", filter_max);
    }
    voxel_grid_filter_.setFilterLimits (filter_min, filter_max);

    if (voxel_grid_filter_.getFilterLimitsNegative () != config.filter_limit_negative)
    {
      voxel_grid_filter_.setFilterLimitsNegative (config.filter_limit_negative);
      ROS_DEBUG ("[config_callback] Setting the filter negative flag to: %s.", config.filter_limit_negative ? "true" : "false");
    }

    if (voxel_grid_filter_.getFilterFieldName () != config.filter_field_name)
    {
      voxel_grid_filter_.setFilterFieldName (config.filter_field_name);
      ROS_DEBUG ("[config_callback] Setting the filter field name to: %s.", config.filter_field_name.c_str ());
    }


    if(sor_filter_.getMeanK () != config.sor_min_k)
    {
    	sor_filter_.setMeanK (config.sor_min_k);
    }

    if(sor_filter_.getStddevMulThresh () != config.sor_std_dev)
	{
		sor_filter_.setStddevMulThresh (config.sor_std_dev);
	}


  }


protected:
  ros::NodeHandle& nh_;
  ros::Subscriber cloud_subscriber_;
  ros::Publisher cloud_publisher_;
  image_transport::Publisher image_publisher_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  //Process
  cv_bridge::CvImagePtr bridge_;

  //Filter
  pcl::VoxelGrid<sensor_msgs::PointCloud2> voxel_grid_filter_;
  pcl::StatisticalOutlierRemoval<sensor_msgs::PointCloud2> sor_filter_;



  //Reconfigure server
  boost::shared_ptr<dynamic_reconfigure::Server<prw_scene_copier::ObjectDetectorConfig> > reconfigure_server_;
  boost::mutex mutex_;

  bool cfg_output_image_;
  bool cfg_output_cloud_;

  //filter
  /** \brief The minimum allowed filter value a point will be considered from. */
  double filter_limit_min_;

  /** \brief The maximum allowed filter value a point will be considered from. */
  double filter_limit_max_;

  /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a filter_limit_max_). Default: false. */
  bool filter_limit_negative_;




};

void mouseCallback( int event, int x, int y, int, void* ptr)
{
  //ObjectDetector* detector = (ObjectDetector*)ptr;

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_object_detector");
  ros::NodeHandle nh("~"); //<-- needed for dynamic reconfigurator to work
  ObjectDetector detector(nh);

  cv::namedWindow(WINDOW_INPUT);
  cv::setMouseCallback(WINDOW_INPUT, mouseCallback, &detector);

  ros::Rate rate(10.0);
  while(ros::ok())
  {

    rate.sleep();
    ros::spinOnce();
    key_ = cv::waitKey(3);
  }






  return 0;
}
