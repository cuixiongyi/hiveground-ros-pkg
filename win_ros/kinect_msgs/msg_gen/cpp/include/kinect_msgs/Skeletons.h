/* Auto-generated by genmsg_cpp for file /home/mahisorn/ros_workspace/hg-ros-pkg/win_ros/kinect_msgs/msg/Skeletons.msg */
#ifndef KINECT_MSGS_MESSAGE_SKELETONS_H
#define KINECT_MSGS_MESSAGE_SKELETONS_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "kinect_msgs/Skeleton.h"

namespace kinect_msgs
{
template <class ContainerAllocator>
struct Skeletons_ {
  typedef Skeletons_<ContainerAllocator> Type;

  Skeletons_()
  : header()
  , skeleton()
  {
  }

  Skeletons_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , skeleton(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector< ::kinect_msgs::Skeleton_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::kinect_msgs::Skeleton_<ContainerAllocator> >::other >  _skeleton_type;
  std::vector< ::kinect_msgs::Skeleton_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::kinect_msgs::Skeleton_<ContainerAllocator> >::other >  skeleton;

  enum { NUI_SKELETON_COUNT = 6 };

  typedef boost::shared_ptr< ::kinect_msgs::Skeletons_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kinect_msgs::Skeletons_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Skeletons
typedef  ::kinect_msgs::Skeletons_<std::allocator<void> > Skeletons;

typedef boost::shared_ptr< ::kinect_msgs::Skeletons> SkeletonsPtr;
typedef boost::shared_ptr< ::kinect_msgs::Skeletons const> SkeletonsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::kinect_msgs::Skeletons_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::kinect_msgs::Skeletons_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace kinect_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::kinect_msgs::Skeletons_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::kinect_msgs::Skeletons_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::kinect_msgs::Skeletons_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bda7ce67dd79e8cf14eda85e22cc84bb";
  }

  static const char* value(const  ::kinect_msgs::Skeletons_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xbda7ce67dd79e8cfULL;
  static const uint64_t static_value2 = 0x14eda85e22cc84bbULL;
};

template<class ContainerAllocator>
struct DataType< ::kinect_msgs::Skeletons_<ContainerAllocator> > {
  static const char* value() 
  {
    return "kinect_msgs/Skeletons";
  }

  static const char* value(const  ::kinect_msgs::Skeletons_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::kinect_msgs::Skeletons_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
Skeleton[] skeleton\n\
\n\
int8 NUI_SKELETON_COUNT = 6\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: kinect_msgs/Skeleton\n\
SkeletonTrackingState skeleton_tracking_state\n\
uint64 tracking_id\n\
uint64 enrollment_index\n\
uint64 user_index\n\
geometry_msgs/Transform position\n\
geometry_msgs/Transform[] skeleton_positions\n\
SkeletonPositionTrackingState[] skeleton_position_tracking_state\n\
uint64 quality_flag\n\
\n\
int8 NUI_SKELETON_POSITION_HIP_CENTER = 0\n\
int8 NUI_SKELETON_POSITION_SPINE = 1\n\
int8 NUI_SKELETON_POSITION_SHOULDER_CENTER = 2\n\
int8 NUI_SKELETON_POSITION_HEAD = 3\n\
int8 NUI_SKELETON_POSITION_SHOULDER_LEFT = 4\n\
int8 NUI_SKELETON_POSITION_ELBOW_LEFT = 5\n\
int8 NUI_SKELETON_POSITION_WRIST_LEFT = 6\n\
int8 NUI_SKELETON_POSITION_HAND_LEFT = 7\n\
int8 NUI_SKELETON_POSITION_SHOULDER_RIGHT = 8\n\
int8 NUI_SKELETON_POSITION_ELBOW_RIGHT = 9\n\
int8 NUI_SKELETON_POSITION_WRIST_RIGHT = 10\n\
int8 NUI_SKELETON_POSITION_HAND_RIGHT = 11\n\
int8 NUI_SKELETON_POSITION_HIP_LEFT = 12\n\
int8 NUI_SKELETON_POSITION_KNEE_LEFT = 13\n\
int8 NUI_SKELETON_POSITION_ANKLE_LEFT = 14\n\
int8 NUI_SKELETON_POSITION_FOOT_LEFT = 15\n\
int8 NUI_SKELETON_POSITION_HIP_RIGHT = 16\n\
int8 NUI_SKELETON_POSITION_KNEE_RIGHT = 17\n\
int8 NUI_SKELETON_POSITION_ANKLE_RIGHT = 18\n\
int8 NUI_SKELETON_POSITION_FOOT_RIGHT = 19\n\
int8 NUI_SKELETON_POSITION_COUNT = 20\n\
\n\
================================================================================\n\
MSG: kinect_msgs/SkeletonTrackingState\n\
int8 NUI_SKELETON_NOT_TRACKED = 0\n\
int8 NUI_SKELETON_POSITION_ONLY = 1\n\
int8 NUI_SKELETON_TRACKED = 2\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Transform\n\
# This represents the transform between two coordinate frames in free space.\n\
\n\
Vector3 translation\n\
Quaternion rotation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: kinect_msgs/SkeletonPositionTrackingState\n\
int8 NUI_SKELETON_POSITION_NOT_TRACKED = 0\n\
int8 NUI_SKELETON_POSITION_INFERRED = 1\n\
int8 NUI_SKELETON_POSITION_TRACKED = 2\n\
\n\
";
  }

  static const char* value(const  ::kinect_msgs::Skeletons_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::kinect_msgs::Skeletons_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::kinect_msgs::Skeletons_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::kinect_msgs::Skeletons_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.skeleton);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Skeletons_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kinect_msgs::Skeletons_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::kinect_msgs::Skeletons_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "skeleton[]" << std::endl;
    for (size_t i = 0; i < v.skeleton.size(); ++i)
    {
      s << indent << "  skeleton[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::kinect_msgs::Skeleton_<ContainerAllocator> >::stream(s, indent + "    ", v.skeleton[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // KINECT_MSGS_MESSAGE_SKELETONS_H

