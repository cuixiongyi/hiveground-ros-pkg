#ifndef RPW_OBJECT_TRACKING_H
#define RPW_OBJECT_TRACKING_H

#include <prw_object_tracking/object_tracking_utils.h>
#include "ui_object_tracking.h"

#include <velib/view.h>
#include <velib/scene.h>

#include <qmutex.h>
#include <qtimer.h>

#include <boost/circular_buffer.hpp>


namespace prw
{


class ObjectTracking : public QMainWindow
{
  Q_OBJECT
public:

  ObjectTracking(ros::NodeHandle & nh, QWidget *parent = 0, Qt::WFlags flags = 0);
  ~ObjectTracking();




public slots:

  void onImageUpdated();

signals:
  void imageUpdated();

protected:
  //Qt
  void closeEvent(QCloseEvent *event);

  //ROS
  void cameraCallback(const sensor_msgs::PointCloud2ConstPtr& message);



protected:
  ros::NodeHandle& nh_;
  ros::Subscriber cloud_subscriber_;
  image_transport::Publisher image_publisher_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
  cv_bridge::CvImagePtr bridge_;

  std::vector<ObjectTrackerPtr> object_trakers_;


public:
  Ui::ObjectTracking ui;
  bool quit_threads_;
  QMutex ui_mutex_;

  ve::View* view_;
  ve::Scene* scene_;
  QGraphicsPixmapItem* scene_image_pixmap_;
  boost::circular_buffer<cv::Mat> image_buffer_;
  QTimer *image_timer_;


};

}


#endif
