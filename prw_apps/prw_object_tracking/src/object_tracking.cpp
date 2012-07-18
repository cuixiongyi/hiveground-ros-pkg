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
    image_buffer_(2)

{
  ui.setupUi(this);


  view_ = new ve::View(this);
  ui.gridLayoutMain->addWidget(view_, 0, 0);

  scene_ = new ve::Scene(view_);
  scene_->setSceneRect(0, 0, 640, 480);
  view_->setScene(scene_);
  scene_->set_mode(ve::Scene::MODE_CURSOR);
  scene_image_ = new ve::OpenCVImage();
  scene_->addItem(scene_image_);
  view_->ensureVisible(scene_->itemsBoundingRect());

  image_publisher_ = image_transport::ImageTransport(nh_).advertise("image", 1);
  cloud_subscriber_ = nh_.subscribe("input", 1, &ObjectTracking::cameraCallback, this);

  image_timer_ = new QTimer(this);
  connect(image_timer_, SIGNAL(timeout()), this, SLOT(onImageUpdate()));
  image_timer_->start(10);

  color_dialog_ = new QColorDialog(this);
  color_dialog_->setOptions(QColorDialog::NoButtons);
  color_dialog_->show();
  //ui.gridLayoutMain->addWidget(color_dialog_, 0, 1);

}

ObjectTracking::~ObjectTracking()
{
}

void ObjectTracking::onImageUpdate()
{
  QMutexLocker lock(&ui_mutex_);
  if(image_buffer_.full())
  {
    if (!scene_image_)
    {
      scene_image_ = new ve::OpenCVImage(image_buffer_.front());
      scene_->addItem(scene_image_);
    }
    else
    {
      scene_image_->updateImage(image_buffer_.front());
    }
    image_buffer_.pop_front();
  }
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
  image_buffer_.push_back(bridge_->image);
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
