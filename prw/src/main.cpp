#include <QtGui/QApplication>
#include <prw/prw.h>

#include <ros/ros.h>

#include <boost/thread.hpp>

#define CONTROL_SPEED 5

PRW* prw_ = NULL;
bool initialized_ = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !initialized_)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !prw_->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

void update_function()
{
  unsigned int counter = 0;
  while(ros::ok())
  {
    if(initialized_)
    {
      if(counter % 5 == 0)
      {
        counter = 1;
        prw_->sendMarkers();
      }
      else
      {
        counter++;
      }
    }
    usleep(3000);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "personal_robotic_workspace", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));
  boost::thread update_thread(boost::bind(&update_function));

  QApplication a(argc, argv);

  PRW w;
  initialized_ = w.initialize();

  if(!initialized_)
    exit(-1);

  prw_ = &w;
  w.show();

  initialized_ = true;

  int ret = a.exec();

  spin_thread.join();
  update_thread.join();

  return ret;
}
