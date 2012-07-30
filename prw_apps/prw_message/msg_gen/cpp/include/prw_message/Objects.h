/* Auto-generated by genmsg_cpp for file /home/mahisorn/ros_workspace/hg-ros-pkg/prw_apps/prw_message/msg/Objects.msg */
#ifndef PRW_MESSAGE_MESSAGE_OBJECTS_H
#define PRW_MESSAGE_MESSAGE_OBJECTS_H
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

#include "prw_message/Object.h"

namespace prw_message
{
template <class ContainerAllocator>
struct Objects_ {
  typedef Objects_<ContainerAllocator> Type;

  Objects_()
  : objects()
  {
  }

  Objects_(const ContainerAllocator& _alloc)
  : objects(_alloc)
  {
  }

  typedef std::vector< ::prw_message::Object_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::prw_message::Object_<ContainerAllocator> >::other >  _objects_type;
  std::vector< ::prw_message::Object_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::prw_message::Object_<ContainerAllocator> >::other >  objects;


  typedef boost::shared_ptr< ::prw_message::Objects_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::prw_message::Objects_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Objects
typedef  ::prw_message::Objects_<std::allocator<void> > Objects;

typedef boost::shared_ptr< ::prw_message::Objects> ObjectsPtr;
typedef boost::shared_ptr< ::prw_message::Objects const> ObjectsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::prw_message::Objects_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::prw_message::Objects_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace prw_message

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::prw_message::Objects_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::prw_message::Objects_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::prw_message::Objects_<ContainerAllocator> > {
  static const char* value() 
  {
    return "81222f48983e6f437a2912e628e1088c";
  }

  static const char* value(const  ::prw_message::Objects_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x81222f48983e6f43ULL;
  static const uint64_t static_value2 = 0x7a2912e628e1088cULL;
};

template<class ContainerAllocator>
struct DataType< ::prw_message::Objects_<ContainerAllocator> > {
  static const char* value() 
  {
    return "prw_message/Objects";
  }

  static const char* value(const  ::prw_message::Objects_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::prw_message::Objects_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# A representation of all detected object\n\
Object[] objects\n\
\n\
================================================================================\n\
MSG: prw_message/Object\n\
# A pose, reference frame, timestamp, and type\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Vector3 size\n\
sensor_msgs/PointCloud2 cloud\n\
string name\n\
int32 type\n\
\n\
# object definitions\n\
int32 UNKNOW = 0\n\
int32 CUBE = 1\n\
int32 SPHERE = 1\n\
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
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
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
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: sensor_msgs/PointCloud2\n\
# This message holds a collection of N-dimensional points, which may\n\
# contain additional information such as normals, intensity, etc. The\n\
# point data is stored as a binary blob, its layout described by the\n\
# contents of the \"fields\" array.\n\
\n\
# The point cloud data may be organized 2d (image-like) or 1d\n\
# (unordered). Point clouds organized as 2d images may be produced by\n\
# camera depth sensors such as stereo or time-of-flight.\n\
\n\
# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n\
# points).\n\
Header header\n\
\n\
# 2D structure of the point cloud. If the cloud is unordered, height is\n\
# 1 and width is the length of the point cloud.\n\
uint32 height\n\
uint32 width\n\
\n\
# Describes the channels and their layout in the binary data blob.\n\
PointField[] fields\n\
\n\
bool    is_bigendian # Is this data bigendian?\n\
uint32  point_step   # Length of a point in bytes\n\
uint32  row_step     # Length of a row in bytes\n\
uint8[] data         # Actual point data, size is (row_step*height)\n\
\n\
bool is_dense        # True if there are no invalid points\n\
\n\
================================================================================\n\
MSG: sensor_msgs/PointField\n\
# This message holds the description of one point entry in the\n\
# PointCloud2 message format.\n\
uint8 INT8    = 1\n\
uint8 UINT8   = 2\n\
uint8 INT16   = 3\n\
uint8 UINT16  = 4\n\
uint8 INT32   = 5\n\
uint8 UINT32  = 6\n\
uint8 FLOAT32 = 7\n\
uint8 FLOAT64 = 8\n\
\n\
string name      # Name of field\n\
uint32 offset    # Offset from start of point struct\n\
uint8  datatype  # Datatype enumeration, see above\n\
uint32 count     # How many elements in the field\n\
\n\
";
  }

  static const char* value(const  ::prw_message::Objects_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::prw_message::Objects_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.objects);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Objects_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::prw_message::Objects_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::prw_message::Objects_<ContainerAllocator> & v) 
  {
    s << indent << "objects[]" << std::endl;
    for (size_t i = 0; i < v.objects.size(); ++i)
    {
      s << indent << "  objects[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::prw_message::Object_<ContainerAllocator> >::stream(s, indent + "    ", v.objects[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // PRW_MESSAGE_MESSAGE_OBJECTS_H

