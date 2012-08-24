#include <les/les.h>
#include <qevent.h>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_wrapper.h>
#include <rviz/default_plugin/grid_display.h>
#include <rviz/ogre_helpers/grid.h>

LES::LES(ros::NodeHandle& nh, QWidget *parent, Qt::WFlags flags)
  : nh_(nh),
    quit_threads_(false)
{
  ui.setupUi(this);
  setupRviz();

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
  grid_->setStyle( rviz::Grid::Billboards ); // Fat lines.
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
