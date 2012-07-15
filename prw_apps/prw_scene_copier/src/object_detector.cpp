#include <ros/ros.h>


#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <prw_scene_copier/object_detectorConfig.h>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <boost/thread.hpp>

using namespace std;

class ObjectDetector
{
public:
  typedef prw_scene_copier::object_detectorConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;



  ObjectDetector(ros::NodeHandle & nh)
    : nh_(nh)
  {


    //publisher
    image_publisher_ = image_transport::ImageTransport(nh_).advertise("image", 1);

    //subscriber
    cloud_subscriber_ = nh_.subscribe("/camera/depth_registered/points", 1, &ObjectDetector::cameraCallback, this);
    //cloud_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud", 1);




    // initialize dynamic reconfigure
    reconfigure_server_.reset (new ReconfigureServer (reconfigure_mutex_, nh_));
    reconfigure_server_->setCallback (boost::bind (&ObjectDetector::configCallback, this, _1, _2));
  }


  ~ObjectDetector()
  {

  }

  void cameraCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);

    // convert cloud to PCL
    pcl::PointCloud <pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // get an OpenCV image from the cloud
    pcl::toROSMsg(cloud, *image_msg);

    try
    {
      bridge_ = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
      ROS_INFO_THROTTLE(1.0, "New depth_registered/cloud.");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Conversion failed");
      return;
    }

    //get
    cv::Mat image_gray;
    cv::cvtColor(bridge_->image, image_gray, CV_BGR2GRAY);















    if(cfg_output_image_)
    {
      image_publisher_.publish( bridge_->toImageMsg() );
    }


  }

  void configCallback (Config &config, uint32_t level)
  {
    boost::recursive_mutex::scoped_lock lock(reconfigure_mutex_);
    cfg_output_image_  = config.output_image;

    ROS_INFO_STREAM("output_image_: " << cfg_output_image_);


  }

protected:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_subscriber_;
  ros::Publisher cloud_publisher_;
  image_transport::Publisher image_publisher_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  //Process
  cv_bridge::CvImagePtr bridge_;


  //Reconfigure server
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  boost::recursive_mutex reconfigure_mutex_;

  bool cfg_output_image_;


};




int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_object_detector");
  ros::NodeHandle nh;
  ObjectDetector detector(nh);
  ros::spin();
  return 0;
}
