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
#include <boost/multi_array.hpp>
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
    : min_size_(0), max_size_(0)
  {}

  cv::Scalar min_hsv_;
  cv::Scalar max_hsv_;
  int min_size_;
  int max_size_;

};

typedef std::map<std::string, ColorObject> ColorObjectMap;

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

class TrackedObject
{
public:
  TrackedObject()
    : area_(0.0)
  { }

  std::string name_;
  cv::RotatedRect boundary_;
  double area_;
};

class LutColorObjectTraker : public ObjectTracker
{
public:
  static const int H_RANGE = 256;
  static const int S_RANGE = 256;
  static const int V_RANGE = 256;



  LutColorObjectTraker();
  ~LutColorObjectTraker();

  void setInput(const cv::Mat& input_image);
  void update();
  bool getResult(cv::Mat& result);
  bool getResult(std::vector<cv::RotatedRect>& result);
  bool load(const std::string& file);
  bool save(const std::string& file);

  void addObject(const std::string& name, const cv::Scalar& min_hsv, const cv::Scalar& max_hsv, int min_size,
                 int max_size);
  void updateObject(const std::string& name, const cv::Scalar& min_hsv, const cv::Scalar& max_hsv, int min_size,
                    int max_size);
  void deleteObject(const std::string& name);

  void updateLut();
protected:


protected:
  typedef boost::multi_array<int, 3> LookupTable;
  typedef LookupTable::index LookupTableIndex;
  LookupTable lut_;
  //std::vector<std::vector<std::vector<unsigned char> > > lut2_;
  ColorObjectMap color_object_map_;
  cv::Mat current_image_;
  cv::Mat last_image_;

};





}





#endif
