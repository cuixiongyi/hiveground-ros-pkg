/*
 * inspector_arm.h
 *
 *  Created on: Jan 7, 2013
 *      Author: mahisorn
 */

#ifndef INSPECTOR_ARM_H_
#define INSPECTOR_ARM_H_

#include <qmainwindow.h>
#include "ui_inspector_arm.h"

#include <ros/ros.h>


class InspectorArm : public QMainWindow
{
  Q_OBJECT
public:

  InspectorArm(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~InspectorArm();

  bool initialize();

protected:
  void closeEvent(QCloseEvent *event);

public:
  bool quit_thread_;

private:
  Ui::InspectorArm ui;
  ros::NodeHandle nh_, nh_private_;


};



#endif /* INSPECTOR_ARM_H_ */
