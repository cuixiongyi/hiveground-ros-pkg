#include <ros/ros.h>
#include <camera_setup/camera_setup.h>
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
  //connect(editor, SIGNAL(valueChanged(double)), this, SLOT(value_changed(double)));
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

void DoubleSpinBoxDelegate::value_changed(double d)
{

}

CameraSetup::CameraSetup(ros::NodeHandle& nh, ros::NodeHandle& nh_private, QWidget *parent, Qt::WFlags flags) :
    QMainWindow(parent, flags), nh_(nh), nh_private_(nh_private), quit_threads_(false), broadcaster_(0)
{
  ui.setupUi(this);

  nh_private_.param("camera_n", camera_n_, 2);
  ROS_DEBUG("Total camera: %d", camera_n_);

  broadcaster_.resize(camera_n_);
  camera_parameter_.resize(camera_n_);
  camera_tf_.resize(camera_n_);

  ui.tableWidget->setRowCount(camera_n_*2);
  ui.tableWidget->setColumnCount(10);
  ui.tableWidget->setHorizontalHeaderItem(0, new QTableWidgetItem(QString("Camera")));
  ui.tableWidget->setHorizontalHeaderItem(1, new QTableWidgetItem(QString("Tx")));
  ui.tableWidget->setHorizontalHeaderItem(2, new QTableWidgetItem(QString("Ty")));
  ui.tableWidget->setHorizontalHeaderItem(3, new QTableWidgetItem(QString("Tz")));
  ui.tableWidget->setHorizontalHeaderItem(4, new QTableWidgetItem(QString("Rx")));
  ui.tableWidget->setHorizontalHeaderItem(5, new QTableWidgetItem(QString("Ry")));
  ui.tableWidget->setHorizontalHeaderItem(6, new QTableWidgetItem(QString("Rz")));
  ui.tableWidget->setHorizontalHeaderItem(7, new QTableWidgetItem(QString("Rw")));
  ui.tableWidget->setHorizontalHeaderItem(8, new QTableWidgetItem(QString("Source")));
  ui.tableWidget->setHorizontalHeaderItem(9, new QTableWidgetItem(QString("Target")));

  for (int i = 0; i < camera_n_; i++)
  {
    string camera_parameter_text;
    nh_private_.param((QString("camera_%1").arg(i)).toStdString(), camera_parameter_text, string(""));
    ROS_DEBUG("Camera %d parameter: %s", i, camera_parameter_text.c_str());
    QStringList tokens = QString(camera_parameter_text.c_str()).split(' ');
    ROS_DEBUG("Camera %d parameter count: %d", i, tokens.size());
    if(tokens.size() != 10)
    {
      ROS_ERROR("camera %d paparmeter count is not correct", i);
      continue;
    }

    int j = 0;
    foreach(QString text, tokens)
    {
      QTableWidgetItem* item = new QTableWidgetItem(text);
      Qt::ItemFlags f = item->flags() & ~Qt::ItemIsEditable;
      item->setFlags(f);
      ui.tableWidget->setItem(i*2, j, item);
      j++;
    }

    CameraParameter parameter;
    parameter.name = tokens[0].toStdString();
    parameter.tf.setOrigin(tf::Vector3(tokens[1].toDouble(), tokens[2].toDouble(), tokens[3].toDouble()));
    parameter.tf.setRotation(tf::Quaternion(tokens[4].toDouble(), tokens[5].toDouble(), tokens[6].toDouble(), tokens[7].toDouble()));
    parameter.source = tokens[8].toStdString();
    parameter.target = tokens[9].toStdString();
    camera_tf_[i] = parameter.tf;
    camera_parameter_[i] = parameter;

    ui.tableWidget->setItem(i*2 + 1, 1, new QTableWidgetItem("0"));
    ui.tableWidget->setItem(i*2 + 1, 2, new QTableWidgetItem("0"));
    ui.tableWidget->setItem(i*2 + 1, 3, new QTableWidgetItem("0"));
    ui.tableWidget->setItem(i*2 + 1, 4, new QTableWidgetItem("0"));
    ui.tableWidget->setItem(i*2 + 1, 5, new QTableWidgetItem("0"));
    ui.tableWidget->setItem(i*2 + 1, 6, new QTableWidgetItem("0"));
    ui.tableWidget->setItem(i*2 + 1, 7, new QTableWidgetItem());
    ui.tableWidget->setItem(i*2 + 1, 8, new QTableWidgetItem());
    ui.tableWidget->setItem(i*2 + 1, 9, new QTableWidgetItem());
    Qt::ItemFlags f = ui.tableWidget->item(i*2 + 1, 7)->flags() & ~Qt::ItemIsEditable;
    ui.tableWidget->item(i*2 + 1, 7)->setFlags(f);
    ui.tableWidget->item(i*2 + 1, 8)->setFlags(f);
    ui.tableWidget->item(i*2 + 1, 9)->setFlags(f);

    ui.tableWidget->setItemDelegateForRow(i*2+1, new DoubleSpinBoxDelegate());
  }

  connect(ui.tableWidget, SIGNAL(cellChanged(int, int)), this, SLOT(tableWidget_cellChanged(int, int) ));

  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(update()));
  timer_->start(100);
}

CameraSetup::~CameraSetup()
{


}


void CameraSetup::update()
{
  for(int i = 0; i < camera_n_; i++)
  {
    tf::StampedTransform stf(camera_tf_[i], ros::Time::now(), camera_parameter_[i].source, camera_parameter_[i].target);
    broadcaster_[i].sendTransform(stf);
  }
}

void CameraSetup::tableWidget_cellChanged(int row, int column)
{
  double value = ui.tableWidget->item(row, column)->data(Qt::DisplayRole).toDouble();
  ROS_INFO("%d %d %f", row, column, value);

  double tx, ty, tz, roll, pitch, yaw;
  tx = ui.tableWidget->item(row, 1)->data(Qt::DisplayRole).toDouble();
  ty = ui.tableWidget->item(row, 2)->data(Qt::DisplayRole).toDouble();
  tz = ui.tableWidget->item(row, 3)->data(Qt::DisplayRole).toDouble();
  roll = ui.tableWidget->item(row, 4)->data(Qt::DisplayRole).toDouble();
  pitch = ui.tableWidget->item(row, 5)->data(Qt::DisplayRole).toDouble();
  yaw = ui.tableWidget->item(row, 6)->data(Qt::DisplayRole).toDouble();
  tf::Quaternion dq;
  dq.setRPY(roll, pitch, yaw);
  tf::Transform dtf(dq, tf::Vector3(tx, ty, tz));

  ROS_INFO_THROTTLE(1.0, "dtf\t %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f",
                          dtf.getOrigin().x(), dtf.getOrigin().y(), dtf.getOrigin().z(),
                          dtf.getRotation().x(), dtf.getRotation().y(), dtf.getRotation().z(), dtf.getRotation().w());

  tf::Transform result = camera_parameter_[row/2].tf  * dtf;

  ui.tableWidget->blockSignals(true);
  ui.tableWidget->item(row - 1, 1)->setData(Qt::DisplayRole, result.getOrigin().x());
  ui.tableWidget->item(row - 1, 2)->setData(Qt::DisplayRole, result.getOrigin().y());
  ui.tableWidget->item(row - 1, 3)->setData(Qt::DisplayRole, result.getOrigin().z());
  ui.tableWidget->item(row - 1, 4)->setData(Qt::DisplayRole, result.getRotation().x());
  ui.tableWidget->item(row - 1, 5)->setData(Qt::DisplayRole, result.getRotation().y());
  ui.tableWidget->item(row - 1, 6)->setData(Qt::DisplayRole, result.getRotation().z());
  ui.tableWidget->item(row - 1, 7)->setData(Qt::DisplayRole, result.getRotation().w());
  ui.tableWidget->blockSignals(false);

  camera_tf_[row/2] = result;
  ROS_DEBUG_THROTTLE(1.0, "result\t %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f",
                     camera_tf_[row-1].getOrigin().x(), camera_tf_[row-1].getOrigin().y(), camera_tf_[row-1].getOrigin().z(),
                     camera_tf_[row-1].getRotation().x(), camera_tf_[row-1].getRotation().y(), camera_tf_[row-1].getRotation().z(), camera_tf_[row-1].getRotation().w());
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
