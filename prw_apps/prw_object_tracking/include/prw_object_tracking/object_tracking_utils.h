#ifndef PRW_OBJECT_TRACKING_H_
#define PRW_OBJECT_TRACKING_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>


#include <boost/shared_ptr.hpp>
//#include <QObject>

#include <opencv2/opencv.hpp>

namespace prw
{

/**
 * Base class for object tracking
 */
class ObjectTracker
{
public:
  ObjectTracker() { }
  virtual ~ObjectTracker() { }

  virtual void setInput(const cv::Mat& input_image) = 0;
  virtual void update() = 0;
  virtual bool getResult(cv::Mat& result) = 0;
  virtual bool load(const std::string& file) = 0;
  virtual bool save(const std::string& file) = 0;
};

typedef boost::shared_ptr<ObjectTracker> ObjectTrackerPtr;

class ColorObject
{
public:
  ColorObject()
    : max_size_(0), min_size_(0)
  {}

  cv::Scalar hsv_max_;
  cv::Scalar hav_min_;
  int max_size_;
  int min_size_;
};

/*
 * A class for tracking single color object in an image
 */
class SimpleColorObjectTracker : public ObjectTracker
{
public:
  SimpleColorObjectTracker();
  ~SimpleColorObjectTracker();

  void setInput(const cv::Mat& input_image);
  void update();
  bool getResult(cv::Mat& result);
  bool load(const std::string& file);
  bool save(const std::string& file);

  void setColor(const cv::Scalar& hsv_max, const cv::Scalar& hsv_min);
  void setSize(int max_size, int min_size);

public:
  ColorObject model_;
};



class ObjectTrackingUtils
{
public:
  ObjectTrackingUtils(ros::NodeHandle & nh);



  void cameraCallback(const sensor_msgs::PointCloud2ConstPtr& message);

  //void addObjectTracker(ObjectTrackerPtr tracker);



  virtual ~ObjectTrackingUtils();

protected:
  ros::NodeHandle& nh_;
  ros::Subscriber cloud_subscriber_;
  image_transport::Publisher image_publisher_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  std::vector<ObjectTrackerPtr> object_trakers_;



};




}





#endif
