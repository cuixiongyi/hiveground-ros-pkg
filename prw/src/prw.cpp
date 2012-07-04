#include <ros/ros.h>


#include <QtGui/QApplication>
#include <prw/prw.h>

PRW::PRW(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
{
	ui.setupUi(this);
}


PRW::~PRW()
{
}


int main(int argc, char** argv)
{
  QApplication a(argc, argv);

  PRW w;
  w.show();
  int ret = a.exec();
  return ret;
}
