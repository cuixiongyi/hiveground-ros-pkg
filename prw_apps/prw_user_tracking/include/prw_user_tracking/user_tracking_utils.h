#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



#include <boost/thread.hpp>

namespace prw
{

/**
 - inputs
   - calibrated, filtered, and merged color point cloud (ROS message from Kinect)
   - robot model and all other objects inside a working space (ROS message from other module)

 - outputs
   - hand, arm, and body (if it is visible in the scene) positions
     - with collision model






 */

class UserTrackingUtils
{
public:



protected:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);

protected:
  ros::Subscriber cloud_subscriber_;
  ros::Publisher cloud_publisher_;


};



}
