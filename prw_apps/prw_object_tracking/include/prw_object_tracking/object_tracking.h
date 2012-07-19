#ifndef RPW_OBJECT_TRACKING_H
#define RPW_OBJECT_TRACKING_H

#include <prw_object_tracking/object_tracking_utils.h>
#include "ui_object_tracking.h"
#include "ui_color_picker.h"

#include <velib/view.h>
#include <velib/scene.h>
#include <velib/node.h>
#include <velib/opencv_image.h>
#include <velib/color_selector.h>

#include <qmutex.h>
#include <qtimer.h>
#include <qcolordialog.h>

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
  void onImageUpdate();
  void onImageClicked(QPointF point, QRgb color);
  void onColorPickerAdd();
  void onColorListClicked(QListWidgetItem * item);
  void onColorPickerSelected(int h, int s, int v);
  void onRangeUpdate();


signals:


protected:
  //UI
  void updateColorPickerRange(const ve::ColorRange range);


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

  std::vector<ObjectTrackerPtr> object_trackers_;


public:
  Ui::ObjectTracking ui;
  Ui::ColorPicker ui_color_picker_;
  bool quit_threads_;
  QMutex ui_mutex_;

  ve::View* view_;
  ve::Scene* scene_;
  //QGraphicsView* view_;
  //QGraphicsScene* scene_;
  ve::OpenCVImage* scene_image_;
  boost::circular_buffer<cv::Mat> image_buffer_;
  QTimer *image_timer_;

  //QColorDialog* color_dialog_;
  ve::ColorPicker* color_picker_;
  ve::ColorLuminancePicker* color_luminance_picker_;
  QDialog* color_picker_dialog_;

};

}


#endif
