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

  color_object_selector_ = new QDialog(this);
  ui_color_object_.setupUi(color_object_selector_);
  ui_color_object_.mainLayout->addWidget(color_picker_);
  ui_color_object_.mainLayout->addWidget(color_luminance_picker_);
  color_object_selector_->show();

  QObject::connect(scene_image_, SIGNAL(mouseClicked(QPointF, QRgb)), this, SLOT(onImageClicked(QPointF, QRgb)));
  QObject::connect(ui_color_object_.pushButtonAdd, SIGNAL(clicked()), this, SLOT(onColorPickerAdd()));
  QObject::connect(ui_color_object_.colorList, SIGNAL(itemClicked(QListWidgetItem *)), this, SLOT(onColorListClicked(QListWidgetItem *)));
  QObject::connect(color_luminance_picker_, SIGNAL(newHsv(int, int, int)), this, SLOT(onColorPickerSelected(int, int, int)));

  QObject::connect(ui_color_object_.spinBoxMaxH, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));
  QObject::connect(ui_color_object_.spinBoxMinH, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));
  QObject::connect(ui_color_object_.spinBoxMaxS, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));
  QObject::connect(ui_color_object_.spinBoxMinS, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));
  QObject::connect(ui_color_object_.spinBoxMaxV, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));
  QObject::connect(ui_color_object_.spinBoxMinV, SIGNAL(valueChanged(int)), this, SLOT(onRangeUpdate()));

  QObject::connect(color_luminance_picker_, SIGNAL(newRange(int, int)), this, SLOT(onLuminanceRangeUpdate(int, int)));

  ui_color_object_.horizontalSliderMinArea->setValue(ui_color_object_.spinBoxMinArea->value());
  ui_color_object_.horizontalSliderMaxArea->setValue(ui_color_object_.spinBoxMaxArea->value());

  QObject::connect(ui_color_object_.spinBoxMaxArea, SIGNAL(valueChanged(int)), this, SLOT(onSizeUpdate()));
  QObject::connect(ui_color_object_.spinBoxMinArea, SIGNAL(valueChanged(int)), this, SLOT(onSizeUpdate()));


  /*
  cv::Scalar hsv_max_(78/2, 255, 255);
  cv::Scalar hsv_min_(37/2, 113, 128);
  color_object_.setColor(hsv_min_, hsv_max_);
  color_object_.setSize(10, 100);
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
    cv::Mat hsv;
    cv::Mat rgb;
    cv::cvtColor(image_buffer_.front(), hsv, CV_BGR2HSV);
    cv::cvtColor(image_buffer_.front(), rgb, CV_BGR2RGB);
    if (!scene_image_)
    {
      scene_image_ = new ve::OpenCVImage(rgb);
      scene_->addItem(scene_image_);
    }
    else
    {
      scene_image_->updateImage(rgb);
    }
    image_buffer_.pop_front();


    //get




    //remove old graphic items
    if(!added_graphics_items_.isEmpty())
    {
      foreach(QGraphicsItem* item, added_graphics_items_)
      {
              scene_->removeItem(item);
      }
      added_graphics_items_.clear();
    }

    if(1)
    {
      std::vector<cv::RotatedRect> result;
      SimpleColorObjectMap::iterator it;
      for(it = color_object_maps_.begin(); it != color_object_maps_.end(); it++)
      {
        it->setInput(hsv);
        it->update();
        //boost::thread cv_thread(boost::bind(&SimpleColorObjectTracker::update, &color_object_));
        if(it->getResult(result))
        {
          //ui_mutex_.lock();
          for(int i = 0; i < result.size(); i++)
          {
            //result[i].
            cv::Rect rect = result[i].boundingRect();
            //scene_image_->c

            QGraphicsRectItem* item = scene_->addRect(rect.x + scene_image_->x(), rect.y + scene_image_->y(), rect.width, rect.height, QPen(Qt::red, 1));
            added_graphics_items_.push_back(item);
          }
          //ui_mutex_.unlock();
        }
      }
    }//if
  }
}

void ObjectTracking::onImageClicked(QPointF point, QRgb color)
{
  //update current color of color picker dialog
  int h, s, v;
  ve::rgb2hsv(color, h, s, v);
  color_picker_->setCol(h, s);
  color_luminance_picker_->setCol(h, s, v);
  ui_color_object_.spinBoxH->setValue(h);
  ui_color_object_.spinBoxS->setValue(s);
  ui_color_object_.spinBoxV->setValue(v);

}

void ObjectTracking::onColorPickerAdd()
{
  bool ok;
  QString name = QInputDialog::getText(color_object_selector_, tr("Add new color"), tr("Enter color name:"), QLineEdit::Normal,
                                       tr("color_name"), &ok);
  if (ok && !name.isEmpty())
  {
    qDebug() << "get:" << name;
    int h, s, v;
    h = ui_color_object_.spinBoxH->value();
    s = ui_color_object_.spinBoxS->value();
    v = ui_color_object_.spinBoxV->value();
    color_picker_->addColorRange(name,
                                 (h + 10) > 360 ? (h + 10) - 360 : h + 10,
                                 (h - 10) < 0 ? 360 - h : (h - 10),
                                 (s + 10) > 255 ? 255 : (s + 10),
                                 (s - 10) < 0 ? 0 : (s - 10));


    QList<QListWidgetItem*> items = ui_color_object_.colorList->findItems(name, Qt::MatchExactly);
    foreach(QListWidgetItem* item, items)
    {
      ui_color_object_.colorList->removeItemWidget(item);
      delete item;
    }

    QListWidgetItem* item = new QListWidgetItem(name);
    if(v < 150)
    {
      item->setTextColor(QColor(255, 255, 255));
    }
    item->setBackgroundColor(QColor::fromHsv(h,s,v));
    ui_color_object_.colorList->addItem(item);
    color_picker_->setCurrentColor(name);
    ve::ColorRange range;
    if(color_picker_->getColorRange(name, range))
    {
      updateColorPickerRange(range);

      //add color object tracker
      //QMutexLocker locker(&color_object_mutex_);
      SimpleColorObjectTracker color_object_tracker;
      cv::Scalar hsv_max(range.max_.hue()/2, range.max_.saturation(), range.max_.value());
      cv::Scalar hsv_min(range.min_.hue()/2, range.min_.saturation(), range.min_.value());
      color_object_tracker.setColor(hsv_min, hsv_max);
      color_object_tracker.setSize(10, 100);
      color_object_maps_[name] = color_object_tracker;
    }


    /*
    SimpleColorObjectTracker object_tracker;
    cv::Scalar hsv_max_(78/2, 255, 255);
    cv::Scalar hsv_min_(37/2, 113, 128);
    color_object_.setColor(hsv_min_, hsv_max_);
    color_object_.setSize(10, 100);
    */

  }
}

void ObjectTracking::onColorListClicked(QListWidgetItem * item)
{
  //qDebug() << item->text();
  color_picker_->setCurrentColor(item->text());
  ve::ColorRange range;
  QString name = color_picker_->getCurrentColorName();
  if (color_picker_->getColorRange(name, range))
  {
    updateColorPickerRange(range);
    int h, s, v;
    if(range.max_.hue() < range.min_.hue())
    {
      h = (range.max_.hue() + (range.min_.hue() - 359))/ 2;
      if(h < 0) h += 360;
    }
    else
      h = (range.max_.hue() + range.min_.hue()) / 2;
    s = (range.max_.saturation() + range.min_.saturation()) / 2;
    v = (range.max_.value() + range.min_.value()) / 2;
    color_picker_->setCol(h, s);
    color_luminance_picker_->setCol(h, s, v);


    if(color_object_maps_.find(name) != color_object_maps_.end())
    {
      //qDebug() << __FUNCTION__ << name << color_object_maps_[name].model().min_size_ << color_object_maps_[name].model().max_size_;
      ui_color_object_.spinBoxMinArea->blockSignals(true);
      ui_color_object_.spinBoxMinArea->setValue(color_object_maps_[name].model().min_size_);
      //qDebug() << "a";
      ui_color_object_.spinBoxMaxArea->setValue(color_object_maps_[name].model().max_size_);
      ui_color_object_.spinBoxMinArea->blockSignals(false);
      //qDebug() << "b";

      ui_color_object_.horizontalSliderMinArea->setValue(ui_color_object_.spinBoxMinArea->value());
      //ui_color_object_.horizontalSliderMaxArea->setValue(ui_color_object_.spinBoxMaxArea->value());

      //ui_color_object_.spinBoxMinArea->update();
      //ui_color_object_.spinBoxMaxArea->update();

    }

  }

}

void ObjectTracking::onColorPickerSelected(int h, int s, int v)
{
  //qDebug() << h << s << v;
  ve::ColorRange range;
  if(color_picker_->getColorRange(color_picker_->getCurrentColorName(), range))
  {
    updateColorPickerRange(range);
    updateColorObjectTracker(color_picker_->getCurrentColorName(), range);
  }
}

void ObjectTracking::onLuminanceRangeUpdate(int v_min, int v_max)
{
  //qDebug() << v_min << v_max;
  if(color_picker_->getCurrentColorName() == "")
    return;

  ui_color_object_.spinBoxMinV->setValue(v_min);
  ui_color_object_.spinBoxMaxV->setValue(v_max);

}

void ObjectTracking::onRangeUpdate()
{
  //qDebug() << __FUNCTION__;
  if(color_picker_->getCurrentColorName() == "")
      return;

  color_picker_->updateColorRange(color_picker_->getCurrentColorName(),
                                  ui_color_object_.spinBoxMaxH->value(),
                                  ui_color_object_.spinBoxMinH->value(),
                                  ui_color_object_.spinBoxMaxS->value(),
                                  ui_color_object_.spinBoxMinS->value(),
                                  ui_color_object_.spinBoxMaxV->value(),
                                  ui_color_object_.spinBoxMinV->value());
  color_luminance_picker_->setRange(ui_color_object_.spinBoxMinV->value(), ui_color_object_.spinBoxMaxV->value());

  //update color tracker
  ve::ColorRange range;
  if(color_picker_->getColorRange(color_picker_->getCurrentColorName(), range))
  {
    updateColorPickerRange(range);
    updateColorObjectTracker(color_picker_->getCurrentColorName(), range);
  }



}

void ObjectTracking::onSizeUpdate()
{
  //qDebug() << __FUNCTION__;
  if(color_picker_->getCurrentColorName() == "")
      return;
  //update color tracker
  //qDebug() << color_picker_->getCurrentColorName() << ui_color_object_.spinBoxMinArea->value() << ui_color_object_.spinBoxMaxArea->value();
  color_object_maps_[color_picker_->getCurrentColorName()].setSize(ui_color_object_.spinBoxMinArea->value(),
                                                                   ui_color_object_.spinBoxMaxArea->value());

}

void ObjectTracking::updateColorPickerRange(const ve::ColorRange& range)
{
  ui_color_object_.spinBoxMaxH->setValue(range.max_.hue());
  ui_color_object_.spinBoxMaxS->setValue(range.max_.saturation());
  ui_color_object_.spinBoxMaxV->setValue(range.max_.value());
  ui_color_object_.spinBoxMinH->setValue(range.min_.hue());
  ui_color_object_.spinBoxMinS->setValue(range.min_.saturation());
  ui_color_object_.spinBoxMinV->setValue(range.min_.value());
}

void ObjectTracking::updateColorObjectTracker(const QString& name, const ve::ColorRange& range)
{
  if(color_object_maps_.find(name) != color_object_maps_.end())
  {
    //QMutexLocker locker(&color_object_mutex_);
    cv::Scalar hsv_max(range.max_.hue()/2, range.max_.saturation(), range.max_.value());
    cv::Scalar hsv_min(range.min_.hue()/2, range.min_.saturation(), range.min_.value());
    color_object_maps_[name].setColor(hsv_min, hsv_max);
    //color_object_maps_[name].setSize(10, 200);
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

  ui_mutex_.lock();
  image_buffer_.push_back(bridge_->image);
  ui_mutex_.unlock();
}


ObjectTracking* object_tracking_ = NULL;
bool initialized_ = false;

void spin_function()
{
  ros::WallRate r(100.0);
  int count = 0;
  while (ros::ok() && !initialized_)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !object_tracking_->quit_threads_)
  {
    count++;
    r.sleep();
    ros::spinOnce();
  }
}

void opencv_spin_function()
{
  while (ros::ok() && !initialized_)
  {
    cv::waitKey(5);
  }
  while (ros::ok() && !object_tracking_->quit_threads_)
  {
    cv::waitKey(5);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_object_tracking", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  boost::thread spin_thread(boost::bind(&spin_function));
  //boost::thread opencv_spin_thread(boost::bind(&opencv_spin_function));

  QApplication a(argc, argv);



  ObjectTracking w(nh);
  object_tracking_ = &w;
  w.show();

  initialized_ = true;

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
