#ifndef KINECT_SERVER_H
#define KINECT_SERVER_H

#include <QtGui/QMainWindow>
#include "ui_kinect_server.h"

#include <ros/ros.h>

class kinect_server : public QMainWindow
{
	Q_OBJECT

public:
	kinect_server(ros::NodeHandle& nh, QWidget *parent = 0, Qt::WFlags flags = 0);
	~kinect_server();
protected:
  void closeEvent(QCloseEvent *event);


public:
  ros::NodeHandle& nh_;
  bool quit_threads_;

private:
	Ui::kinect_serverClass ui;
  
};

#endif // KINECT_SERVER_H
