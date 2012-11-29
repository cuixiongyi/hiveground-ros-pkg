#include <ros/ros.h>
#include <prw_camera_setup/camera_setup.h>
#include <QtGui/QApplication>
#include <QDebug>


#include <boost/thread.hpp>

using namespace std;
using namespace prw;

DoubleSpinBoxDelegate::DoubleSpinBoxDelegate(QObject *parent) :
    QItemDelegate(parent)
{
}

QWidget *DoubleSpinBoxDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &/* option */,
                                       const QModelIndex &/* index */) const
{
  QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
  editor->setMinimum(-10.0);
  editor->setMaximum(10.0);
  editor->setDecimals(6);
  editor->setSingleStep(0.001);
  return editor;
}

void DoubleSpinBoxDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
  double value = index.model()->data(index, Qt::EditRole).toDouble();

  QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox*>(editor);
  spinBox->setValue(value);
}

void DoubleSpinBoxDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
  QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox*>(editor);
  spinBox->interpretText();
  double value = spinBox->value();

  model->setData(index, value, Qt::EditRole);
}

void DoubleSpinBoxDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option,
                                           const QModelIndex &/* index */) const
{
  editor->setGeometry(option.rect);
}

CameraSetup::CameraSetup(ros::NodeHandle& nh, ros::NodeHandle& nh_private, QWidget *parent, Qt::WFlags flags) :
    QMainWindow(parent, flags), nh_(nh), nh_private_(nh_private), quit_threads_(false), broadcaster_(0)
{
  ui.setupUi(this);

  nh_private_.param("camera_n", camera_n_, 2);
  ROS_DEBUG("Total camera: %d", camera_n_);

  model_ = new QStandardItemModel(camera_n_, (1 + 3 + 3 + 2 + 1), this);
  model_->setHorizontalHeaderItem(0, new QStandardItem(QString("Camera")));
  model_->setHorizontalHeaderItem(1, new QStandardItem(QString("Tx")));
  model_->setHorizontalHeaderItem(2, new QStandardItem(QString("Ty")));
  model_->setHorizontalHeaderItem(3, new QStandardItem(QString("Tz")));
  model_->setHorizontalHeaderItem(4, new QStandardItem(QString("Rx")));
  model_->setHorizontalHeaderItem(5, new QStandardItem(QString("Ry")));
  model_->setHorizontalHeaderItem(6, new QStandardItem(QString("Rz")));
  model_->setHorizontalHeaderItem(7, new QStandardItem(QString("Source")));
  model_->setHorizontalHeaderItem(8, new QStandardItem(QString("Target")));
  model_->setHorizontalHeaderItem(9, new QStandardItem(QString("Rate")));

  ui.tableView->setModel(model_);
  ui.tableView->setItemDelegateForColumn(1, new DoubleSpinBoxDelegate());
  ui.tableView->setItemDelegateForColumn(2, new DoubleSpinBoxDelegate());
  ui.tableView->setItemDelegateForColumn(3, new DoubleSpinBoxDelegate());
  ui.tableView->setItemDelegateForColumn(4, new DoubleSpinBoxDelegate());
  ui.tableView->setItemDelegateForColumn(5, new DoubleSpinBoxDelegate());
  ui.tableView->setItemDelegateForColumn(6, new DoubleSpinBoxDelegate());

  for (int i = 0; i < camera_n_; i++)
  {
    string camera_parameter;
    nh_private_.param((QString("camera_%1").arg(i)).toStdString(), camera_parameter, string(""));
    ROS_DEBUG("Camera %d parameter: %s", i, camera_parameter.c_str());
    QStringList tokens = QString(camera_parameter.c_str()).split(' ');
    ROS_INFO("Camera %d parameter count: %d", i, tokens.size());
    qDebug() << tokens;

    int j = 1;
    foreach(QString text, tokens)
    {
      QStandardItem* item = new QStandardItem(text);
      model_->setItem(i, j++, item);
    }
  }

  broadcaster_ = new tf::TransformBroadcaster();

  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(update()));
  timer_->start(1000);
}

CameraSetup::~CameraSetup()
{
  if (broadcaster_)
    delete broadcaster_;

}


void CameraSetup::update()
{
  //QModelIndex index;
  double tx, ty, tz, rx, ry, rz;
  QString source, target;
  for(int i = 0; i < camera_n_; i++)
  {
    tx = model_->data(model_->index(i, 1, QModelIndex()), Qt::DisplayRole).toDouble();
    ty = model_->data(model_->index(i, 2, QModelIndex()), Qt::DisplayRole).toDouble();
    tz = model_->data(model_->index(i, 3, QModelIndex()), Qt::DisplayRole).toDouble();
    rx = model_->data(model_->index(i, 4, QModelIndex()), Qt::DisplayRole).toDouble();
    ry = model_->data(model_->index(i, 5, QModelIndex()), Qt::DisplayRole).toDouble();
    rz = model_->data(model_->index(i, 6, QModelIndex()), Qt::DisplayRole).toDouble();
    source = model_->data(model_->index(i, 7, QModelIndex()), Qt::DisplayRole).toString();
    target = model_->data(model_->index(i, 8, QModelIndex()), Qt::DisplayRole).toString();
    ROS_DEBUG("%d %f %f %f %f %f %f %s %s", i, tx, ty, tz, rx, ry, rz, source.toStdString().c_str(), target.toStdString().c_str());
    tf::Quaternion q;
    q.setRPY(rx, ry, rz);
    tf::Transform tf(q, tf::Vector3(tx, ty, tz));
    tf::StampedTransform stf(tf, ros::Time::now(), source.toStdString(), target.toStdString());
    broadcaster_->sendTransform(stf);
  }
}

CameraSetup* g_camera_setup_ = NULL;
bool g_initialized_ = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized_)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_camera_setup_->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "prw_camera_setup", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);

  CameraSetup w(nh, nh_private);
  g_camera_setup_ = &w;
  w.show();

  g_initialized_ = true;

  int ret = a.exec();

  spin_thread.join();

  return ret;
}
