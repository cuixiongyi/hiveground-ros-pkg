#include <ros/ros.h>

#include <qmainwindow.h>
#include "ui_les.h"

namespace rviz
{
class GridDisplay;
class RenderPanel;
class VisualizationManager;
}

class LES : public QMainWindow
{
  Q_OBJECT
public:
  LES(ros::NodeHandle& nh, QWidget *parent = 0, Qt::WFlags flags = 0);
  ~LES();
public:
  ros::NodeHandle& nh_;
  bool quit_threads_;

protected:
  //Qt
  void closeEvent(QCloseEvent *event);

  //ROS
  void setupRviz();
  void cleanUpRviz();

private:
  Ui::LesUi ui;
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::GridDisplay* grid_;
};
