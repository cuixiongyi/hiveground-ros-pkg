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

#include <dynamic_reconfigure/server.h>
#include <prw_scene_copier/object_detectorConfig.h>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <boost/thread.hpp>

using namespace std;

//[ INFO] [1342151263.036519507]: [-0.8082887505081064; -0.4723549093709118; 1.87841526005252]
//[ INFO] [1342151263.036554068]: [0.1718112088724865; 0.07161661170882727; 0.018879656723713]

int key_;
tf::Transform tf_;
Eigen::Matrix4f tf_matrix_;

static const string WINDOW_INPUT = "Input Image";


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
    cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);




    // initialize dynamic reconfigure
    reconfigure_server_.reset (new ReconfigureServer (reconfigure_mutex_, nh_));
    reconfigure_server_->setCallback (boost::bind (&ObjectDetector::configCallback, this, _1, _2));

    tf::Quaternion qt = tf::createQuaternionFromRPY(0.1718112088724865, 0.07161661170882727, 0.018879656723713);
    tf::Vector3 vec(-0.8082887505081064, -0.4723549093709118, 1.87841526005252);
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
    //int key = cv::waitKey(3);

    sensor_msgs::Image image;
    sensor_msgs::PointCloud2 msg_transformed_;

    pcl_ros::transformPointCloud(tf_matrix_, *msg, msg_transformed_);

    // convert cloud to PCL
    pcl::PointCloud <pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(msg_transformed_, cloud);

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


    //cv::imshow(WINDOW_INPUT, image_gray);















    if(cfg_output_image_)
    {
      image_publisher_.publish( bridge_->toImageMsg() );
    }

    if(cfg_output_cloud_)
    {
      cloud_publisher_.publish(msg_transformed_);
    }


  }

  void configCallback (Config &config, uint32_t level)
  {
    boost::recursive_mutex::scoped_lock lock(reconfigure_mutex_);
    cfg_output_image_  = config.output_image;
    cfg_output_cloud_  = config.output_cloud;

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
  bool cfg_output_cloud_;


};

void mouseCallback( int event, int x, int y, int, void* ptr)
{
  ObjectDetector* detector = (ObjectDetector*)ptr;

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_object_detector");
  ros::NodeHandle nh;
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
