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
  virtual bool getResult(std::vector<cv::RotatedRect>& result) = 0;
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
  cv::Scalar hsv_min_;
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
  bool getResult(std::vector<cv::RotatedRect>& result);
  bool load(const std::string& file);
  bool save(const std::string& file);

  void setColor(const cv::Scalar& hsv_min, const cv::Scalar& hsv_max);
  void setSize( int min_size, int max_size);
  const ColorObject& model() { return model_; }

protected:
  ColorObject model_;
  cv::Rect tracking_windows_;
  cv::Point last_position_;
  cv::Mat current_image_;
  cv::Mat last_image_;
  cv::Mat mask_;
  bool found_in_last_image_;
  std::vector<std::vector<cv::Point> > contours_filtered_;
};







}





#endif
