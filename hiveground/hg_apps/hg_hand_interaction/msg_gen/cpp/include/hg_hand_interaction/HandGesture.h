/* Auto-generated by genmsg_cpp for file /home/mahisorn/ros_workspace/hg-ros-pkg/hiveground/hg_apps/hg_hand_interaction/msg/HandGesture.msg */
#ifndef HG_HAND_INTERACTION_MESSAGE_HANDGESTURE_H
#define HG_HAND_INTERACTION_MESSAGE_HANDGESTURE_H
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

namespace hg_hand_interaction
{
template <class ContainerAllocator>
struct HandGesture_ {
  typedef HandGesture_<ContainerAllocator> Type;

  HandGesture_()
  : header()
  {
  }

  HandGesture_(const ContainerAllocator& _alloc)
  : header(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;


  typedef boost::shared_ptr< ::hg_hand_interaction::HandGesture_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hg_hand_interaction::HandGesture_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct HandGesture
typedef  ::hg_hand_interaction::HandGesture_<std::allocator<void> > HandGesture;

typedef boost::shared_ptr< ::hg_hand_interaction::HandGesture> HandGesturePtr;
typedef boost::shared_ptr< ::hg_hand_interaction::HandGesture const> HandGestureConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::hg_hand_interaction::HandGesture_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::hg_hand_interaction::HandGesture_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace hg_hand_interaction

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hg_hand_interaction::HandGesture_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hg_hand_interaction::HandGesture_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hg_hand_interaction::HandGesture_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d7be0bb39af8fb9129d5a76e6b63a290";
  }

  static const char* value(const  ::hg_hand_interaction::HandGesture_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd7be0bb39af8fb91ULL;
  static const uint64_t static_value2 = 0x29d5a76e6b63a290ULL;
};

template<class ContainerAllocator>
struct DataType< ::hg_hand_interaction::HandGesture_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hg_hand_interaction/HandGesture";
  }

  static const char* value(const  ::hg_hand_interaction::HandGesture_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hg_hand_interaction::HandGesture_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
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
";
  }

  static const char* value(const  ::hg_hand_interaction::HandGesture_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::hg_hand_interaction::HandGesture_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::hg_hand_interaction::HandGesture_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hg_hand_interaction::HandGesture_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct HandGesture_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hg_hand_interaction::HandGesture_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::hg_hand_interaction::HandGesture_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
  }
};


} // namespace message_operations
} // namespace ros

#endif // HG_HAND_INTERACTION_MESSAGE_HANDGESTURE_H
