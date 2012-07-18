#include <prw_object_tracking/object_tracking_utils.h>

using namespace prw;

ObjectTrackingUtils::ObjectTrackingUtils(ros::NodeHandle & nh)
  : nh_(nh)
{
  image_publisher_ = image_transport::ImageTransport(nh_).advertise("image", 1);





}



void ObjectTrackingUtils::cameraCallback(const sensor_msgs::PointCloud2ConstPtr& message)
{
  // convert cloud to PCL
  pcl::PointCloud <pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*message, cloud);

  // get an OpenCV image from the cloud
  sensor_msgs::Image image;
  pcl::toROSMsg(cloud, image);

  cv_bridge::CvImagePtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(image, image.encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Conversion failed");
    return;
  }

  //get
  cv::Mat image_hsv;
  cv::cvtColor(bridge->image, image_hsv, CV_BGR2HSV);



}


ObjectTrackingUtils::~ObjectTrackingUtils()
{

}


