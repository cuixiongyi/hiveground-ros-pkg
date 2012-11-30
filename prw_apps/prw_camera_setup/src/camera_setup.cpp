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
  ROS_INFO("haha");
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

  //-0.012478 0.394196 1.361399 -0.696552 0.711558 0.064578 0.065807
  //0.637117 -0.021175 1.265933 -0.696552 0.711558 0.064578 -0.065807
  //tf::Transform tf(tf::Quaternion(-0.669506, 0.70836, -0.0489958, -0.218143), tf::Vector3(0.51795, 0.573112, 0.960851));
  tf::Transform tf(tf::Quaternion(-0.696552, 0.711558, 0.064578, 0.065807), tf::Vector3(-0.012478, 0.394196, 1.361399));
  tf = tf.inverse();

  ROS_INFO_THROTTLE(1.0, "tf\t %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f",
                     tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z(),
                     tf.getRotation().x(), tf.getRotation().y(), tf.getRotation().z(), tf.getRotation().w());


  /*
  model_ = new QStandardItemModel(camera_n_*2, (1 + 3 + 4 + 2), this);
  model_->setHorizontalHeaderItem(0, new QStandardItem(QString("Camera")));
  model_->setHorizontalHeaderItem(1, new QStandardItem(QString("Tx")));
  model_->setHorizontalHeaderItem(2, new QStandardItem(QString("Ty")));
  model_->setHorizontalHeaderItem(3, new QStandardItem(QString("Tz")));
  model_->setHorizontalHeaderItem(4, new QStandardItem(QString("Rx")));
  model_->setHorizontalHeaderItem(5, new QStandardItem(QString("Ry")));
  model_->setHorizontalHeaderItem(6, new QStandardItem(QString("Rz")));
  model_->setHorizontalHeaderItem(7, new QStandardItem(QString("Rw")));
  model_->setHorizontalHeaderItem(8, new QStandardItem(QString("Source")));
  model_->setHorizontalHeaderItem(9, new QStandardItem(QString("Target")));
  //model_->setHorizontalHeaderItem(10, new QStandardItem(QString("Rate")));

  ui.tableView->setModel(model_);
  //ui.tableView->setItemDelegateForColumn(1, new DoubleSpinBoxDelegate());
  //ui.tableView->setItemDelegateForColumn(2, new DoubleSpinBoxDelegate());
  //ui.tableView->setItemDelegateForColumn(3, new DoubleSpinBoxDelegate());
  //ui.tableView->setItemDelegateForColumn(4, new DoubleSpinBoxDelegate());
  //ui.tableView->setItemDelegateForColumn(5, new DoubleSpinBoxDelegate());
  //ui.tableView->setItemDelegateForColumn(6, new DoubleSpinBoxDelegate());
   */
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
      QTableWidgetItem* item = new QTableWidgetItem(text);
      Qt::ItemFlags f = item->flags() & ~Qt::ItemIsEditable;
      item->setFlags(f);
      ui.tableWidget->setItem(i*2, j, item);
      j++;
    }

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

    //ui.tableView->setItemDelegateForRow(i*2+1, new DoubleSpinBoxDelegate());
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
  QString source, target;

  for(int i = 0; i < camera_n_; i++)
  {
    double x, y, z, rx, ry, rz, rw;
    x = ui.tableWidget->item(2*i, 1)->data(Qt::DisplayRole).toDouble();
    y = ui.tableWidget->item(2*i, 2)->data(Qt::DisplayRole).toDouble();
    z = ui.tableWidget->item(2*i, 3)->data(Qt::DisplayRole).toDouble();
    rx = ui.tableWidget->item(2*i, 4)->data(Qt::DisplayRole).toDouble();
    ry = ui.tableWidget->item(2*i, 5)->data(Qt::DisplayRole).toDouble();
    rz = ui.tableWidget->item(2*i, 6)->data(Qt::DisplayRole).toDouble();
    rw = ui.tableWidget->item(2*i, 7)->data(Qt::DisplayRole).toDouble();
    source = ui.tableWidget->item(2*i, 8)->data(Qt::DisplayRole).toString();
    target = ui.tableWidget->item(2*i, 9)->data(Qt::DisplayRole).toString();

    tf::Transform tf(tf::Quaternion(rx, ry, rz, rw), tf::Vector3(x, y, z));

    tf::StampedTransform stf(tf, ros::Time::now(), source.toStdString(), target.toStdString());
    broadcaster_[i].sendTransform(stf);
    ROS_INFO_THROTTLE(1.0, "tf\t %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %s %s",
                            tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z(),
                            tf.getRotation().x(), tf.getRotation().y(), tf.getRotation().z(), tf.getRotation().w(),
                            source.toStdString().c_str(), target.toStdString().c_str());


  }

}

void CameraSetup::tableWidget_cellChanged(int row, int column)
{
  double value = ui.tableWidget->item(row, column)->data(Qt::DisplayRole).toDouble();
  ROS_INFO("%d %d %f", row, column, value);

  double x, y, z, rx, ry, rz, rw;
  x = ui.tableWidget->item(row - 1, 1)->data(Qt::DisplayRole).toDouble();
  y = ui.tableWidget->item(row - 1, 2)->data(Qt::DisplayRole).toDouble();
  z = ui.tableWidget->item(row - 1, 3)->data(Qt::DisplayRole).toDouble();
  rx = ui.tableWidget->item(row - 1, 4)->data(Qt::DisplayRole).toDouble();
  ry = ui.tableWidget->item(row - 1, 5)->data(Qt::DisplayRole).toDouble();
  rz = ui.tableWidget->item(row - 1, 6)->data(Qt::DisplayRole).toDouble();
  rw = ui.tableWidget->item(row - 1, 7)->data(Qt::DisplayRole).toDouble();



  tf::Transform tf(tf::Quaternion(rx, ry, rz, rw), tf::Vector3(x, y, z));
  ROS_DEBUG_THROTTLE(1.0, "tf\t %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f",
                          tf.getOrigin().x(), tf.getOrigin().y(), tf.getOrigin().z(),
                          tf.getRotation().x(), tf.getRotation().y(), tf.getRotation().z(), tf.getRotation().w());


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

  ROS_DEBUG_THROTTLE(1.0, "dtf\t %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f",
                          dtf.getOrigin().x(), dtf.getOrigin().y(), dtf.getOrigin().z(),
                          dtf.getRotation().x(), dtf.getRotation().y(), dtf.getRotation().z(), dtf.getRotation().w());



  tf::Transform result = tf * dtf;
  ROS_DEBUG_THROTTLE(1.0, "result\t %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f",
                          result.getOrigin().x(), result.getOrigin().y(), result.getOrigin().z(),
                          result.getRotation().x(), result.getRotation().y(), result.getRotation().z(), result.getRotation().w());


  ui.tableWidget->blockSignals(true);
  ui.tableWidget->item(row - 1, 1)->setData(Qt::DisplayRole, result.getOrigin().x());
  ui.tableWidget->item(row - 1, 2)->setData(Qt::DisplayRole, result.getOrigin().y());
  ui.tableWidget->item(row - 1, 3)->setData(Qt::DisplayRole, result.getOrigin().z());
  ui.tableWidget->item(row - 1, 4)->setData(Qt::DisplayRole, result.getRotation().x());
  ui.tableWidget->item(row - 1, 5)->setData(Qt::DisplayRole, result.getRotation().y());
  ui.tableWidget->item(row - 1, 6)->setData(Qt::DisplayRole, result.getRotation().z());
  ui.tableWidget->item(row - 1, 7)->setData(Qt::DisplayRole, result.getRotation().w());
  ui.tableWidget->blockSignals(false);

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
