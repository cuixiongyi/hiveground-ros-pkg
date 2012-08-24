#include <les/les.h>

LES::LES(ros::NodeHandle& nh, QWidget *parent, Qt::WFlags flags)
  : nh_(nh),
    quit_threads_(false)
{
  ui.setupUi(this);
}

LES::~LES()
{

}
