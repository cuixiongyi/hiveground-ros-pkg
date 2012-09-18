#include <les/les.h>
#include <qevent.h>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_wrapper.h>
#include <rviz/default_plugin/grid_display.h>
#include <rviz/ogre_helpers/grid.h>

using namespace std;

LES::LES(ros::NodeHandle& nh, QWidget *parent, Qt::WFlags flags)
  : nh_(nh),
    quit_threads_(false)
{
  ui.setupUi(this);
  setupRviz();

  setupDevice();




}

LES::~LES()
{
  cleanUpRviz();
}

void LES::closeEvent(QCloseEvent *event)
{
  quit_threads_ = true;
  event->accept();
}

void LES::setupDevice()
{
  //setting
  XmlRpc::XmlRpcValue kinects;
  nh_.getParam("kinects", kinects);
  if (kinects.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_FATAL("invalid YAML structure");
    ROS_BREAK();
  }

  kinect_num_ = kinects.size();

  for (XmlRpc::XmlRpcValue::iterator it = kinects.begin(); it != kinects.end(); it++)
  {
    string name = it->first;
    string id;
    if (!nh_.getParam("kinects/" + name + "/id", id))
    {
      ROS_FATAL_STREAM(name + " has no name information");
      ROS_BREAK();
    }

    ROS_INFO_STREAM(id);
    kinect_id_.push_back(id);
    std::string rgb_topic = "/" + id + "/rgb/image_raw";
    std::string depth_topic = "/" + id + "/depth_registered/image_raw";

    ImageSubscriberPair image_pair;
    image_pair.first = ImageSubscriberPtr(new  ImageSubscriber(nh_, rgb_topic, 10));
    image_pair.second = ImageSubscriberPtr(new  ImageSubscriber(nh_, depth_topic, 10));
    kinects_[id] = image_pair;

    //ApproxSync1DevicePtr approx_sync_ = ApproxSync1DevicePtr(new ApproxSync1Device(ApproxSync1DevicePolicy(20), *(kinects_[id].first), *(kinects_[id].second)));
    //approx_sync_->registerCallback(boost::bind(&LES::callback1Device, this, _1, _2));
    //approx_sync_1_devices_.push_back(approx_sync_);

  }

  switch(kinect_num_)
  {
    case 1:
      approx_sync_1_device_ = ApproxSync1DevicePtr(new ApproxSync1Device(ApproxSync1DevicePolicy(20), *(kinects_[kinect_id_[0]].first), *(kinects_[kinect_id_[0]].second)));
      approx_sync_1_device_->registerCallback(boost::bind(&LES::callback1Device, this, _1, _2));
      break;
    case 2:
      approx_sync_2_device_ = ApproxSync2DevicePtr(new ApproxSync2Device(ApproxSync2DevicePolicy(20),
                                                                         *(kinects_[kinect_id_[0]].first), *(kinects_[kinect_id_[0]].second),
                                                                         *(kinects_[kinect_id_[1]].first), *(kinects_[kinect_id_[1]].second)));
      approx_sync_2_device_->registerCallback(boost::bind(&LES::callback2Device, this, _1, _2,  _3, _4));
      break;
    case 3:
      approx_sync_3_device_ = ApproxSync3DevicePtr(new ApproxSync3Device(ApproxSync3DevicePolicy(20),
                                                                         *(kinects_[kinect_id_[0]].first), *(kinects_[kinect_id_[0]].second),
                                                                         *(kinects_[kinect_id_[1]].first), *(kinects_[kinect_id_[1]].second),
                                                                         *(kinects_[kinect_id_[2]].first), *(kinects_[kinect_id_[2]].second)));
      approx_sync_3_device_->registerCallback(boost::bind(&LES::callback3Device, this, _1, _2,  _3, _4, _5, _6));
      break;
    case 4:
      approx_sync_4_device_ = ApproxSync4DevicePtr(new ApproxSync4Device(ApproxSync4DevicePolicy(20),
                                                                         *(kinects_[kinect_id_[0]].first), *(kinects_[kinect_id_[0]].second),
                                                                         *(kinects_[kinect_id_[1]].first), *(kinects_[kinect_id_[1]].second),
                                                                         *(kinects_[kinect_id_[2]].first), *(kinects_[kinect_id_[2]].second),
                                                                         *(kinects_[kinect_id_[3]].first), *(kinects_[kinect_id_[3]].second)));
      approx_sync_4_device_->registerCallback(boost::bind(&LES::callback4Device, this, _1, _2,  _3, _4, _5, _6, _7, _8));
      break;
    default:
      ROS_ERROR("No device connected!");
      ROS_BREAK();
      break;
  }

}

void LES::callback1Device(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth)
{
  //ROS_INFO_THROTTLE(1.0, "yo1");
  ROS_INFO_STREAM_THROTTLE(1.0, "yo1" << rgb->header.frame_id);

}

void LES::callback2Device(const sensor_msgs::ImageConstPtr& rgb0, const sensor_msgs::ImageConstPtr& depth0,
                          const sensor_msgs::ImageConstPtr& rgb1, const sensor_msgs::ImageConstPtr& depth1)
{
  ROS_INFO_THROTTLE(1.0, "yo2");
}
void LES::callback3Device(const sensor_msgs::ImageConstPtr& rgb0, const sensor_msgs::ImageConstPtr& depth0,
                          const sensor_msgs::ImageConstPtr& rgb1, const sensor_msgs::ImageConstPtr& depth1,
                          const sensor_msgs::ImageConstPtr& rgb2, const sensor_msgs::ImageConstPtr& depth2)
{
  ROS_INFO_THROTTLE(1.0, "yo3");
}
void LES::callback4Device(const sensor_msgs::ImageConstPtr& rgb0, const sensor_msgs::ImageConstPtr& depth0,
                          const sensor_msgs::ImageConstPtr& rgb1, const sensor_msgs::ImageConstPtr& depth1,
                          const sensor_msgs::ImageConstPtr& rgb2, const sensor_msgs::ImageConstPtr& depth2,
                          const sensor_msgs::ImageConstPtr& rgb3, const sensor_msgs::ImageConstPtr& depth3)
{
  ROS_INFO_THROTTLE(1.0, "yo4");
}


void LES::setupRviz()
{
  render_panel_ = new rviz::RenderPanel();
  ui.gridMain->addWidget(render_panel_, 0, 0);

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager(render_panel_);
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  manager_->startUpdate();

  rviz::DisplayWrapper* wrapper = manager_->createDisplay("rviz/Grid", "adjustable grid", true);
  ROS_ASSERT( wrapper != NULL);

  // Unwrap it.
  rviz::Display* display = wrapper->getDisplay();
  ROS_ASSERT( display != NULL);


  // Downcast it to the type we think we know it is.
  //
  // (This is one part I would like to improve in the future.  For
  // this to work currently, we need to link against the plugin
  // library containing GridDisplay (libdefault_plugin.so) in addition
  // to linking against librviz.so.  This pretty much negates the
  // benefits of the plugin architecture.)
  grid_ = dynamic_cast<rviz::GridDisplay*>( display );
  ROS_ASSERT( grid_ != NULL );

  // Configure the GridDisplay the way we like it.
  grid_->setStyle( rviz::Grid::Lines ); // Fat lines.
  grid_->setColor( rviz::Color( 1.0f, 1.0f, 0.0f )); // I like yellow.

}

void LES::cleanUpRviz()
{
  if( manager_ != NULL )
  {
    manager_->removeAllDisplays();
  }
  delete render_panel_;
  delete manager_;
}
