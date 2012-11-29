#ifndef RPW_HAND_TRACKING_H
#define RPW_HAND_TRACKING_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "ui_camera_setup.h"
#include <qtimer.h>
#include <qstandarditemmodel.h>
#include <QItemDelegate>
#include <QModelIndex>
#include <QObject>
#include <QSize>
#include <QSpinBox>

namespace prw
{

class DoubleSpinBoxDelegate : public QItemDelegate
{
Q_OBJECT

public:
  DoubleSpinBoxDelegate(QObject *parent = 0);

  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const;

  void setEditorData(QWidget *editor, const QModelIndex &index) const;
  void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;

  void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const;
};

class CameraSetup : public QMainWindow
{
Q_OBJECT
public:

  CameraSetup(ros::NodeHandle& nh, ros::NodeHandle& nh_private, QWidget *parent = 0, Qt::WFlags flags = 0);
  ~CameraSetup();

public slots:
  void update();

protected:

public:
  Ui::CameraSetup ui;
  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_private_;
  bool quit_threads_;

  int camera_n_;
  std::vector<std::string> camera_parameter_;
  QStandardItemModel* model_;
  tf::TransformBroadcaster* broadcaster_;
  QTimer* timer_;

};

}

#endif
