#include <ros/ros.h>
#include <prw_object_tracking/object_tracking.h>

#include <QtGui/QApplication>
#include <QDebug>
#include <qcolordialog.h>
#include <qevent.h>
#include <qinputdialog.h>

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
  //view_ = new QGraphicsView(this);
  ui.gridLayoutMain->addWidget(view_, 0, 0);

  scene_ = new ve::Scene(view_);
  //scene_ = new QGraphicsScene(this);
  scene_->setSceneRect(0, 0, 800, 600 );
  view_->setScene(scene_);
  scene_->set_mode(ve::Scene::MODE_CURSOR);
  scene_image_ = new ve::OpenCVImage();
  scene_->addItem(scene_image_);
  scene_image_->setPos(100, 100);
  view_->ensureVisible(scene_->itemsBoundingRect());

  image_publisher_ = image_transport::ImageTransport(nh_).advertise("image", 1);
  cloud_subscriber_ = nh_.subscribe("input", 1, &ObjectTracking::cameraCallback, this);

  image_timer_ = new QTimer(this);
  connect(image_timer_, SIGNAL(timeout()), this, SLOT(onImageUpdate()));
  image_timer_->start(10);



  //Color picker
  color_picker_ = new ve::ColorPicker(this);
  color_luminance_picker_ = new ve::ColorLuminancePicker(this);
  color_luminance_picker_->setFixedWidth(20);
  QObject::connect(color_picker_, SIGNAL(newCol(int,int)), color_luminance_picker_, SLOT(setCol(int,int)));

  color_picker_dialog_ = new QDialog(this);
  ui_color_picker_.setupUi(color_picker_dialog_);
  ui_color_picker_.mainLayout->addWidget(color_picker_);
  ui_color_picker_.mainLayout->addWidget(color_luminance_picker_);
  color_picker_dialog_->show();

  QObject::connect(scene_image_, SIGNAL(mouseClicked(QPointF, QRgb)), this, SLOT(onImageClicked(QPointF, QRgb)));
  QObject::connect(ui_color_picker_.pushButtonAdd, SIGNAL(clicked()), this, SLOT(onColorPickerAdd()));
  QObject::connect(ui_color_picker_.colorList, SIGNAL(itemClicked(QListWidgetItem *)), this, SLOT(onColorListClicked(QListWidgetItem *)));
  QObject::connect(color_luminance_picker_, SIGNAL(newHsv(int, int, int)), this, SLOT(onColorPickerSelected(int, int, int)));

  QObject::connect(ui_color_picker_.spinBoxMaxH, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));
  QObject::connect(ui_color_picker_.spinBoxMinH, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));
  QObject::connect(ui_color_picker_.spinBoxMaxS, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));
  QObject::connect(ui_color_picker_.spinBoxMinS, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));
  QObject::connect(ui_color_picker_.spinBoxMaxV, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));
  QObject::connect(ui_color_picker_.spinBoxMinV, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));

  //void onRangeUpdate();

  //color_dialog_ = new QColorDialog(this);
  //color_dialog_->setOptions(QColorDialog::NoButtons);
  //color_dialog_->show();
  //ui.gridLayoutMain->addWidget(color_dialog_, 0, 1);
  /*QObject::connect(ui_color_picker_.spinBoxMaxH, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));
    */
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

void ObjectTracking::onImageClicked(QPointF point, QRgb color)
{
  //update current color of color picker dialog
  int h, s, v;
  ve::rgb2hsv(color, h, s, v);
  color_picker_->setCol(h, s);
  color_luminance_picker_->setCol(h, s, v);
  ui_color_picker_.spinBoxH->setValue(h);
  ui_color_picker_.spinBoxS->setValue(s);
  ui_color_picker_.spinBoxV->setValue(v);

}

void ObjectTracking::onColorPickerAdd()
{
  bool ok;
  QString name = QInputDialog::getText(color_picker_dialog_, tr("Add new color"), tr("Enter color name:"), QLineEdit::Normal,
                                       tr("color_name"), &ok);
  if (ok && !name.isEmpty())
  {
    qDebug() << "get:" << name;
    int h, s, v;
    h = ui_color_picker_.spinBoxH->value();
    s = ui_color_picker_.spinBoxS->value();
    v = ui_color_picker_.spinBoxV->value();
    color_picker_->addColorRange(name,
                                 (h + 10) > 360 ? (h + 10) - 360 : h + 10,
                                 (h - 10) < 0 ? 360 - h : (h - 10),
                                 (s + 10) > 255 ? 255 : (s + 10),
                                 (s - 10) < 0 ? 0 : (s - 10));


    QList<QListWidgetItem*> items = ui_color_picker_.colorList->findItems(name, Qt::MatchExactly);
    foreach(QListWidgetItem* item, items)
    {
      ui_color_picker_.colorList->removeItemWidget(item);
      delete item;
    }

    QListWidgetItem* item = new QListWidgetItem(name);
    if(v < 150)
    {
      item->setTextColor(QColor(255, 255, 255));
    }
    item->setBackgroundColor(QColor::fromHsv(h,s,v));
    ui_color_picker_.colorList->addItem(item);
    color_picker_->setCurrentColor(name);
    ve::ColorRange range;
    if(color_picker_->getColorRange(name, range))
      updateColorPickerRange(range);



  }
}

void ObjectTracking::onColorListClicked(QListWidgetItem * item)
{
  //qDebug() << item->text();
  color_picker_->setCurrentColor(item->text());
  ve::ColorRange range;
  if (color_picker_->getColorRange(color_picker_->getCurrentColorName(), range))
    updateColorPickerRange(range);

}

void ObjectTracking::onColorPickerSelected(int h, int s, int v)
{
  //qDebug() << h << s << v;
  ve::ColorRange range;
  if(color_picker_->getColorRange(color_picker_->getCurrentColorName(), range))
    updateColorPickerRange(range);
}

void ObjectTracking::onRangeUpdate()
{
  //qDebug() << __FUNCTION__;
  color_picker_->updateColorRange(color_picker_->getCurrentColorName(),
                                  ui_color_picker_.spinBoxMaxH->value(),
                                  ui_color_picker_.spinBoxMinH->value(),
                                  ui_color_picker_.spinBoxMaxS->value(),
                                  ui_color_picker_.spinBoxMinS->value(),
                                  ui_color_picker_.spinBoxMaxV->value(),
                                  ui_color_picker_.spinBoxMinV->value());
}

void ObjectTracking::updateColorPickerRange(const ve::ColorRange range)
{
  ui_color_picker_.spinBoxMaxH->setValue(range.max_.hue());
  ui_color_picker_.spinBoxMaxS->setValue(range.max_.saturation());
  ui_color_picker_.spinBoxMaxV->setValue(range.max_.value());
  ui_color_picker_.spinBoxMinH->setValue(range.min_.hue());
  ui_color_picker_.spinBoxMinS->setValue(range.min_.saturation());
  ui_color_picker_.spinBoxMinV->setValue(range.min_.value());
}

void ObjectTracking::closeEvent(QCloseEvent *event)
{
  ROS_INFO("Close windows");
  quit_threads_ = true;
  event->accept();
}

void ObjectTracking::cameraCallback(const sensor_msgs::PointCloud2ConstPtr& message)
{
  ROS_INFO_STREAM_THROTTLE(30.0, message->header.frame_id);

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
