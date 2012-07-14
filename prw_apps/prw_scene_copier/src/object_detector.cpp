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






    // initialize dynamic reconfigure
    reconfigure_server_.reset (new ReconfigureServer (reconfigure_mutex_, nh_));
    reconfigure_server_->setCallback (boost::bind (&ObjectDetector::configCallback, this, _1, _2));
  }


  ~ObjectDetector()
  {

  }


  void configCallback (Config &config, uint32_t level)
  {
    reconfigure_mutex_.lock();
    ROS_INFO_STREAM("test:" << config.test);

    reconfigure_mutex_.unlock();
  }

protected:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_subscriber_;
  ros::Publisher cloud_publisher_;
  image_transport::Publisher image_publisher_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  //Reconfigure server
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  boost::recursive_mutex reconfigure_mutex_;

  int cfg_test_;


};




int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_object_detector");
  ros::NodeHandle nh;
  ObjectDetector detector(nh);
  ros::spin();
  return 0;
}
