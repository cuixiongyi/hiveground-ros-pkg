#include <ros/ros.h>
#include <prw_object_tracking/object_tracking.h>

#include <QtGui/QApplication>
#include <QDebug>
#include <qcolordialog.h>
#include <qevent.h>

#include <boost/thread.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

using namespace prw;
using namespace cv;

ObjectTracking::ObjectTracking(ros::NodeHandle & nh, QWidget *parent, Qt::WFlags flags) :
    QMainWindow(parent, flags),
    nh_(nh),
    quit_threads_(false),
    image_buffer_(3)

{
  ui.setupUi(this);
  scene_image_pixmap_ = 0;

  view_ = new ve::View(this);
  ui.gridLayoutMain->addWidget(view_, 0, 0);

  scene_ = new ve::Scene(view_);
  scene_->setSceneRect(0, 0, 640, 480);
  view_->setScene(scene_);
  scene_->set_mode(ve::Scene::MODE_CURSOR);
  view_->ensureVisible(scene_->itemsBoundingRect());

  image_publisher_ = image_transport::ImageTransport(nh_).advertise("image", 1);
  cloud_subscriber_ = nh_.subscribe("input", 1, &ObjectTracking::cameraCallback, this);

  image_timer_ = new QTimer(this);
  connect(image_timer_, SIGNAL(timeout()), this, SLOT(onImageUpdated()));
  image_timer_->start(50);

}

ObjectTracking::~ObjectTracking()
{
}


void ObjectTracking::onImageUpdated()
{
  ui_mutex_.lock();
  if(!image_buffer_.empty())
  {
    QImage image = QImage(image_buffer_.front().data,
                          image_buffer_.front().cols,
                          image_buffer_.front().rows,
                          image_buffer_.front().step, QImage::Format_RGB888);

    if (scene_image_pixmap_)
    {
      scene_->removeItem(scene_image_pixmap_);
      scene_image_pixmap_ = 0;
    }

    scene_image_pixmap_ = scene_->addPixmap(QPixmap::fromImage(image));
  }
    //image_buffer_.pop_front();
  ui_mutex_.unlock();
}

void ObjectTracking::closeEvent(QCloseEvent *event)
{
  ROS_INFO("Close windows");
  quit_threads_ = true;
  event->accept();
}

void ObjectTracking::cameraCallback(const sensor_msgs::PointCloud2ConstPtr& message)
{
  ROS_INFO_STREAM_THROTTLE(1.0, message->header.frame_id);

  // convert cloud to PCL
  pcl::PointCloud <pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*message, cloud);

  // get an OpenCV image from the cloud
  sensor_msgs::Image image_message;
  pcl::toROSMsg(cloud, image_message);


  try
  {
    bridge_ = cv_bridge::toCvCopy(image_message, image_message.encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Conversion failed");
    return;
  }

  //get
  cv::Mat image_rgb;
  cv::cvtColor(bridge_->image, image_rgb, CV_BGR2RGB);

  ui_mutex_.lock();
    image_buffer_.push_back(image_rgb);
  ui_mutex_.unlock();





}


ObjectTracking* object_tracking_ = NULL;
bool initialized_ = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !initialized_)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !object_tracking_->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_object_tracking", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);



  ObjectTracking w(nh);
  object_tracking_ = &w;
  w.show();

  initialized_ = true;

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
