#include "kinect_server.h"
#include <qevent.h>

kinect_server::kinect_server(ros::NodeHandle& nh, QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags),
    nh_(nh),
    quit_threads_(false)
{
	ui.setupUi(this);
}

kinect_server::~kinect_server()
{

}

void kinect_server::closeEvent(QCloseEvent *event)
{
  quit_threads_ = true;
  event->accept();
}
