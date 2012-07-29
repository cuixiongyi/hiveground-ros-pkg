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

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace prw;
using namespace cv;

ObjectTracking::ObjectTracking(ros::NodeHandle & nh, QWidget *parent, Qt::WFlags flags) :
    QMainWindow(parent, flags),
    nh_(nh),
    quit_threads_(false),
    pass_through_filter_(true),
    image_buffer_(2),
    cloud_buffer_(2)

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
  cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2> ("cloud_output", 1);
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

  //lut_color_object_tracker_.updateLut();

  //Voxel grid param
  ROS_ASSERT(nh_.getParam("output_frame", output_frame_));
  ROS_ASSERT(nh_.getParam("filter_limit_min_z", filter_limit_min_z_));
  ROS_ASSERT(nh_.getParam("filter_limit_max_z", filter_limit_max_z_));
  ROS_ASSERT(nh_.getParam("filter_limit_min_y", filter_limit_min_y_));
  ROS_ASSERT(nh_.getParam("filter_limit_max_y", filter_limit_max_y_));
  ROS_ASSERT(nh_.getParam("filter_limit_min_x", filter_limit_min_x_));
  ROS_ASSERT(nh_.getParam("filter_limit_max_x", filter_limit_max_x_));
  ROS_ASSERT(nh_.getParam("filter_limit_negative", filter_limit_negative_));
  ROS_ASSERT(nh_.getParam("leaf_size", leaf_size_));



  voxel_filter_.setFilterLimitsNegative(filter_limit_negative_);
  voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);




  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_PLANE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setDistanceThreshold(0.02);

  ec_.setClusterTolerance (0.02); // 2cm
  ec_.setMinClusterSize(50);
  ec_.setMaxClusterSize(500);



}

ObjectTracking::~ObjectTracking()
{
}

void ObjectTracking::onImageUpdate()
{

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud;
  ui_mutex_.lock();
  if(cloud_buffer_.full())
  {
    cloud = cloud_buffer_.front();
    cloud_buffer_.pop_front();
  }
  else
  {
    ui_mutex_.unlock();
    return;
  }
  ui_mutex_.unlock();

  double t = (double)getTickCount();

  sensor_msgs::Image image_message;
  pcl::toROSMsg(*cloud, image_message);


  try
  {
    bridge_ = cv_bridge::toCvCopy(image_message, image_message.encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Conversion failed");
    return;
  }

  cv::Mat image = bridge_->image;


  cv::Mat hsv;
  cv::Mat rgb;
  cv::cvtColor(image, hsv, CV_BGR2HSV);
  cv::cvtColor(image, rgb, CV_BGR2RGB);

  if (!scene_image_)
  {
    scene_image_ = new ve::OpenCVImage(rgb);
    scene_->addItem(scene_image_);
  }
  else
  {
    scene_image_->updateImage(rgb);
  }

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_transformed (new  pcl::PointCloud <pcl::PointXYZRGB>());
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_filtered (new  pcl::PointCloud <pcl::PointXYZRGB>());
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_tmp (new  pcl::PointCloud <pcl::PointXYZRGB>());
  pcl_ros::transformPointCloud (output_frame_, *cloud, *cloud_transformed, tf_listener_);



  voxel_filter_.setInputCloud(cloud_transformed);
  voxel_filter_.setFilterFieldName("z");
  voxel_filter_.setFilterLimits(filter_limit_min_z_, filter_limit_max_z_);
  voxel_filter_.filter(*cloud_tmp);


  voxel_filter_.setInputCloud(cloud_tmp);
  voxel_filter_.setFilterFieldName("x");
  voxel_filter_.setFilterLimits(filter_limit_min_x_, filter_limit_max_x_);
  voxel_filter_.filter(*cloud_filtered);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  int i = 0, nr_points = (int)cloud_filtered->points.size();
  while (cloud_filtered->points.size() > 0.3 * nr_points)
  {
    seg_.setInputCloud(cloud_filtered);
    seg_.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      return;
    }

    // Extract the inliers
    extract_.setInputCloud(cloud_filtered);
    extract_.setIndices(inliers);
    extract_.setNegative(false);
    extract_.filter(*cloud_p);
    //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points."   << std::endl;

    if(ui.actionSaveCloud->isChecked())
    {
      std::stringstream ss;
      ss << "scene_plane_" << i << ".pcd";
      ROS_INFO_STREAM("saved " << ss.str());
      writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_p, false);
    }

    // Create the filtering object
    extract_.setNegative(true);
    extract_.filter(*cloud_f);
    cloud_filtered.swap(cloud_f);
    i++;
  }


  if(ui.actionSaveCloud->isChecked())
  {
    writer.write<pcl::PointXYZRGB>("object.pcd", *cloud, false);
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;




  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*cloud_filtered, msg);
  cloud_publisher_.publish(boost::make_shared<sensor_msgs::PointCloud2>(msg));


  //setup EuclideanClusterExtraction input
   ec_.setSearchMethod(tree);
   ec_.setInputCloud(cloud_filtered);
   ec_.extract(cluster_indices);

   int j = 0;
   std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;
   float cx, cy, cz;
   float cr, cg, cb;

   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
   {
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
     cx = cy = cz = 0.0; cr = cg = cb = 0;
     for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
     {
       //add point to cloud cluster
       cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*

       //compute centroid
       cx += cloud_filtered->points[*pit].x;
       cy += cloud_filtered->points[*pit].y;
       cz += cloud_filtered->points[*pit].z;

       //get color color
       unsigned char * rgb = (unsigned char *) &(cloud_filtered->points[j].rgb);
       cr += rgb[0];
       cg += rgb[1];
       cb += rgb[2];
     }
     cloud_cluster->width = cloud_cluster->points.size();
     cloud_cluster->height = 1;
     cloud_cluster->is_dense = true;
     cloud_clusters.push_back(cloud_cluster);
     cx = cx/cloud_cluster->points.size();
     cy = cy/cloud_cluster->points.size();
     cz = cz/cloud_cluster->points.size();
     cr = cr/cloud_cluster->points.size();
     cg = cg/cloud_cluster->points.size();
     cb = cb/cloud_cluster->points.size();



     ROS_INFO("Cluster %02d size: %04d [%6.3f, %6.3f, %6.3f] [%03d %03d %03d]",
              j,
              (int)cloud_cluster->points.size(),
              cx, cy, cz,
              (int)cr, (int)cg, (int)cb);



     if(ui.actionSaveCloud->isChecked())
     {
       std::stringstream ss;
       ss << "cloud_cluster_" << j << ".pcd";
       ROS_INFO_STREAM("saved " << ss.str());
       writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_cluster, false); //*
     }
     j++;
   }



  t = ((double)getTickCount() - t)/getTickFrequency();
  ROS_INFO_THROTTLE(1.0, "process time: %5.3f s", t);












#if 0
    cv::Mat image;
    pcl::PointCloud <pcl::PointXYZRGB> cloud;
    ui_mutex_.lock();
    if(image_buffer_.full())
    {
      image = image_buffer_.front();
      cloud = cloud_buffer_.front();
      image_buffer_.pop_front();
      cloud_buffer_.pop_front();
    }
    else
    {
      ui_mutex_.unlock();
      return;
    }
    ui_mutex_.unlock();


    cv::Mat hsv;
    cv::Mat rgb;
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    cv::cvtColor(image, rgb, CV_BGR2RGB);
    if (!scene_image_)
    {
      scene_image_ = new ve::OpenCVImage(rgb);
      scene_->addItem(scene_image_);
    }
    else
    {
      scene_image_->updateImage(rgb);
    }



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

            int idx = (rect.x + rect.width/2) + ((rect.y + rect.height/2)*640);
            qDebug() << cloud.points[idx].x << cloud.points[idx].y << cloud.points[idx].z;

            //tf_listener_.transformPoint

          }
          //ui_mutex_.unlock();
        }
      }
    }//if
#endif
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
#if 0
  static bool saved = false;

  //QMutexLocker locker(&ui_mutex_);


  //ROS_INFO_STREAM_THROTTLE(30.0, message->header.frame_id);

  double t = (double)getTickCount();


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  // convert cloud to PCL
  pcl::fromROSMsg(*message, *cloud);

  if(!saved)
  {
    ROS_INFO_STREAM("saved scene.pcd");
    writer.write<pcl::PointXYZRGB> ("scene.pcd", *cloud, false);
  }

  //extract table plane
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  int i = 0, nr_points = (int)cloud->points.size();
  while (cloud->points.size() > 0.3 * nr_points)
  {
    seg_.setInputCloud(cloud);
    seg_.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      return;
    }

    // Extract the inliers
    extract_.setInputCloud(cloud);
    extract_.setIndices(inliers);
    extract_.setNegative(false);
    extract_.filter(*cloud_p);
    //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points."   << std::endl;

    if(!saved)
    {
      std::stringstream ss;
      ss << "scene_plane_" << i << ".pcd";
      ROS_INFO_STREAM("saved " << ss.str());
      writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_p, false);
    }

    // Create the filtering object
    extract_.setNegative(true);
    extract_.filter(*cloud_f);
    cloud.swap(cloud_f);
    i++;
  }

  if(!saved)
  {
    writer.write<pcl::PointXYZRGB>("object.pcd", *cloud, false);
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud);
  std::vector<pcl::PointIndices> cluster_indices;

  //setup EuclideanClusterExtraction input
  ec_.setSearchMethod(tree);
  ec_.setInputCloud(cloud);
  ec_.extract(cluster_indices);

  int j = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_clusters;
  float cx, cy, cz;
  float cr, cg, cb;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    cx = cy = cz = 0.0; cr = cg = cb = 0;
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      //add point to cloud cluster
      cloud_cluster->points.push_back(cloud->points[*pit]); //*

      //compute centroid
      cx += cloud->points[*pit].x;
      cy += cloud->points[*pit].y;
      cz += cloud->points[*pit].z;

      //get color color
      unsigned char * rgb = (unsigned char *) &(cloud->points[j].rgb);
      cr += rgb[0];
      cg += rgb[1];
      cb += rgb[2];
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    cloud_clusters.push_back(cloud_cluster);
    cx = cx/cloud_cluster->points.size();
    cy = cy/cloud_cluster->points.size();
    cz = cz/cloud_cluster->points.size();
    cr = cr/cloud_cluster->points.size();
    cg = cg/cloud_cluster->points.size();
    cb = cb/cloud_cluster->points.size();


    /*
    ROS_INFO("Cluster %02d size: %04d [%6.3f, %6.3f, %6.3f] [%03d %03d %03d]",
             j,
             (int)cloud_cluster->points.size(),
             cx, cy, cz,
             (int)cr, (int)cg, (int)cb);
    */


    if(!saved)
    {
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      ROS_INFO_STREAM("saved " << ss.str());
      writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_cluster, false); //*
    }
    j++;
  }

  //ROS_INFO("=============");



  t = ((double)getTickCount() - t)/getTickFrequency();
  //std::cout << "Segmentation update time: " << t << std::endl;

  saved = true;


    /*
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                         << coefficients->values[1] << " "
                                         << coefficients->values[2] << " "
                                         << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    */





  //ROS_INFO_STREAM_THROTTLE(1.0, cloud.height << ":" << cloud.width);



#endif

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  // convert cloud to PCL
  pcl::fromROSMsg(*message, *cloud);

  ui_mutex_.lock();
  cloud_buffer_.push_back(cloud);
  ui_mutex_.unlock();

  // get an OpenCV image from the cloud
  //sensor_msgs::Image image_message;
  //pcl::toROSMsg(*cloud, image_message);

/*
  try
  {
    bridge_ = cv_bridge::toCvCopy(image_message, image_message.encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Conversion failed");
    return;
  }
*/


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
