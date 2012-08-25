#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <vector>
#include <string>

#include <qmainwindow.h>
#include "ui_les.h"

namespace rviz
{
class GridDisplay;
class RenderPanel;
class VisualizationManager;
}


class LES : public QMainWindow
{
  Q_OBJECT
public:
  typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;
  typedef boost::shared_ptr<ImageSubscriber> ImageSubscriberPtr;
  typedef message_filters::Subscriber<sensor_msgs::CameraInfo> CameraInfoSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxSync1DevicePolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::Image, sensor_msgs::Image> ApproxSync2DevicePolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::Image, sensor_msgs::Image> ApproxSync3DevicePolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::Image, sensor_msgs::Image> ApproxSync4DevicePolicy;
  typedef message_filters::Synchronizer<ApproxSync1DevicePolicy> ApproxSync1Device;
  typedef boost::shared_ptr<ApproxSync1Device> ApproxSync1DevicePtr;
  typedef message_filters::Synchronizer<ApproxSync2DevicePolicy> ApproxSync2Device;
  typedef boost::shared_ptr<ApproxSync2Device> ApproxSync2DevicePtr;
  typedef message_filters::Synchronizer<ApproxSync3DevicePolicy> ApproxSync3Device;
  typedef boost::shared_ptr<ApproxSync3Device> ApproxSync3DevicePtr;
  typedef message_filters::Synchronizer<ApproxSync4DevicePolicy> ApproxSync4Device;
  typedef boost::shared_ptr<ApproxSync4Device> ApproxSync4DevicePtr;



  typedef std::pair<ImageSubscriberPtr, ImageSubscriberPtr> ImageSubscriberPair;
  typedef std::map<std::string, ImageSubscriberPair> KinectSubscriberMap;


  LES(ros::NodeHandle& nh, QWidget *parent = 0, Qt::WFlags flags = 0);
  ~LES();
public:
  ros::NodeHandle& nh_;
  bool quit_threads_;

protected:
  //Qt
  void closeEvent(QCloseEvent *event);

  //RVIZ
  void setupRviz();
  void cleanUpRviz();

  //LES
  void setupDevice();



  //callback
  void callback1Device(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth);
  void callback2Device(const sensor_msgs::ImageConstPtr& rgb0, const sensor_msgs::ImageConstPtr& depth0,
                       const sensor_msgs::ImageConstPtr& rgb1, const sensor_msgs::ImageConstPtr& depth1);
  void callback3Device(const sensor_msgs::ImageConstPtr& rgb0, const sensor_msgs::ImageConstPtr& depth0,
                       const sensor_msgs::ImageConstPtr& rgb1, const sensor_msgs::ImageConstPtr& depth1,
                       const sensor_msgs::ImageConstPtr& rgb2, const sensor_msgs::ImageConstPtr& depth2);
  void callback4Device(const sensor_msgs::ImageConstPtr& rgb0, const sensor_msgs::ImageConstPtr& depth0,
                       const sensor_msgs::ImageConstPtr& rgb1, const sensor_msgs::ImageConstPtr& depth1,
                       const sensor_msgs::ImageConstPtr& rgb2, const sensor_msgs::ImageConstPtr& depth2,
                       const sensor_msgs::ImageConstPtr& rgb3, const sensor_msgs::ImageConstPtr& depth3);


private:
  Ui::LesUi ui;
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::GridDisplay* grid_;

  //Subscriber
  int kinect_num_;
  std::vector<std::string> kinect_id_;
  KinectSubscriberMap kinects_;
  std::vector<ApproxSync1DevicePtr> approx_sync_1_devices_;
  ApproxSync1DevicePtr approx_sync_1_device_;
  ApproxSync2DevicePtr approx_sync_2_device_;
  ApproxSync3DevicePtr approx_sync_3_device_;
  ApproxSync4DevicePtr approx_sync_4_device_;
};
