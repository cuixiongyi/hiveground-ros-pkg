#include <les/les.h>
#include <qevent.h>

LES::LES(ros::NodeHandle& nh, QWidget *parent, Qt::WFlags flags)
  : nh_(nh),
    quit_threads_(false)
{
  ui.setupUi(this);
}

LES::~LES()
{

}

void LES::closeEvent(QCloseEvent *event)
{
  quit_threads_ = true;
  event->accept();
}
