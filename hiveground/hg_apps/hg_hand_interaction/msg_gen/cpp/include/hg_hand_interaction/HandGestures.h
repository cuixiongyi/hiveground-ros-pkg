/* Auto-generated by genmsg_cpp for file /home/mahisorn/ros_workspace/hg-ros-pkg/hiveground/hg_apps/hg_hand_interaction/msg/HandGestures.msg */
#ifndef HG_HAND_INTERACTION_MESSAGE_HANDGESTURES_H
#define HG_HAND_INTERACTION_MESSAGE_HANDGESTURES_H
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
#include "hg_hand_interaction/HandGesture.h"

namespace hg_hand_interaction
{
template <class ContainerAllocator>
struct HandGestures_ {
  typedef HandGestures_<ContainerAllocator> Type;

  HandGestures_()
  : header()
  , gestures()
  {
  }

  HandGestures_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , gestures(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector< ::hg_hand_interaction::HandGesture_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::hg_hand_interaction::HandGesture_<ContainerAllocator> >::other >  _gestures_type;
  std::vector< ::hg_hand_interaction::HandGesture_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::hg_hand_interaction::HandGesture_<ContainerAllocator> >::other >  gestures;


  typedef boost::shared_ptr< ::hg_hand_interaction::HandGestures_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hg_hand_interaction::HandGestures_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct HandGestures
typedef  ::hg_hand_interaction::HandGestures_<std::allocator<void> > HandGestures;

typedef boost::shared_ptr< ::hg_hand_interaction::HandGestures> HandGesturesPtr;
typedef boost::shared_ptr< ::hg_hand_interaction::HandGestures const> HandGesturesConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::hg_hand_interaction::HandGestures_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::hg_hand_interaction::HandGestures_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace hg_hand_interaction

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hg_hand_interaction::HandGestures_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hg_hand_interaction::HandGestures_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hg_hand_interaction::HandGestures_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6ec8d74eee60d32f06423a7c0f42baa1";
  }

  static const char* value(const  ::hg_hand_interaction::HandGestures_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6ec8d74eee60d32fULL;
  static const uint64_t static_value2 = 0x06423a7c0f42baa1ULL;
};

template<class ContainerAllocator>
struct DataType< ::hg_hand_interaction::HandGestures_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hg_hand_interaction/HandGestures";
  }

  static const char* value(const  ::hg_hand_interaction::HandGestures_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hg_hand_interaction::HandGestures_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
HandGesture[] gestures\n\
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
MSG: hg_hand_interaction/HandGesture\n\
int8 type\n\
float64 var1\n\
\n\
#request constants\n\
int8 NOT_DETECTED = 0\n\
int8 SWEEP_UP_ONE_HAND = 1\n\
int8 SWEEP_DOWN_ONE_HAND = 2\n\
int8 SWEEP_LEFT_ONE_HAND = 3\n\
int8 SWEEP_RIGHT_ONE_HAND = 4\n\
int8 SWEEP_FORWARD_ONE_HAND = 5\n\
int8 SWEEP_BACKWARD_ONE_HAND = 6\n\
\n\
int8 SWEEP_UP_TWO_HAND = 7\n\
int8 SWEEP_DOWN_TWO_HAND = 8\n\
int8 SWEEP_LEFT_TWO_HAND = 9\n\
int8 SWEEP_RIGHT_TWO_HAND = 10\n\
int8 SWEEP_FORWARD_TWO_HAND = 11\n\
int8 SWEEP_BACKWARD_TWO_HAND = 12\n\
int8 SWEEP_OPEN_TWO_HAND = 13\n\
int8 SWEEP_CLOSE_TWO_HAND = 14\n\
\n\
int8 PUSH_PULL_XP = 15\n\
int8 PUSH_PULL_XN = 16 \n\
int8 PUSH_PULL_YP = 17\n\
int8 PUSH_PULL_YN = 18\n\
int8 PUSH_PULL_ZP = 19\n\
int8 PUSH_PULL_ZN = 20\n\
\n\
int8 PUSH_PULL_RXP = 21\n\
int8 PUSH_PULL_RXN = 22\n\
int8 PUSH_PULL_RYP = 23\n\
int8 PUSH_PULL_RYN = 24\n\
int8 PUSH_PULL_RZP = 25\n\
int8 PUSH_PULL_RZN = 26\n\
";
  }

  static const char* value(const  ::hg_hand_interaction::HandGestures_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::hg_hand_interaction::HandGestures_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::hg_hand_interaction::HandGestures_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hg_hand_interaction::HandGestures_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.gestures);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct HandGestures_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hg_hand_interaction::HandGestures_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::hg_hand_interaction::HandGestures_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "gestures[]" << std::endl;
    for (size_t i = 0; i < v.gestures.size(); ++i)
    {
      s << indent << "  gestures[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::hg_hand_interaction::HandGesture_<ContainerAllocator> >::stream(s, indent + "    ", v.gestures[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // HG_HAND_INTERACTION_MESSAGE_HANDGESTURES_H

