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


namespace hg_hand_interaction
{
template <class ContainerAllocator>
struct HandGesture_ {
  typedef HandGesture_<ContainerAllocator> Type;

  HandGesture_()
  : type(0)
  {
  }

  HandGesture_(const ContainerAllocator& _alloc)
  : type(0)
  {
  }

  typedef int8_t _type_type;
  int8_t type;

  enum { NOT_DETECTED = 0 };
  enum { SWEEP_UP_ONE_HAND = 1 };
  enum { SWEEP_DOWN_ONE_HAND = 2 };
  enum { SWEEP_LEFT_ONE_HAND = 3 };
  enum { SWEEP_RIGHT_ONE_HAND = 4 };
  enum { SWEEP_FORWARD_ONE_HAND = 5 };
  enum { SWEEP_BACKWARD_ONE_HAND = 6 };
  enum { SWEEP_UP_TWO_HAND = 7 };
  enum { SWEEP_DOWN_TWO_HAND = 8 };
  enum { SWEEP_LEFT_TWO_HAND = 9 };
  enum { SWEEP_RIGHT_TWO_HAND = 10 };
  enum { SWEEP_FORWARD_TWO_HAND = 11 };
  enum { SWEEP_BACKWARD_TWO_HAND = 12 };
  enum { SWEEP_OPEN_TWO_HAND = 13 };
  enum { SWEEP_CLOSE_TWO_HAND = 14 };

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
    return "54924b3bfee069bfbfc09b1d80e2254d";
  }

  static const char* value(const  ::hg_hand_interaction::HandGesture_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x54924b3bfee069bfULL;
  static const uint64_t static_value2 = 0xbfc09b1d80e2254dULL;
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
    return "int8 type\n\
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
";
  }

  static const char* value(const  ::hg_hand_interaction::HandGesture_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::hg_hand_interaction::HandGesture_<ContainerAllocator> > : public TrueType {};
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
    stream.next(m.type);
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
    s << indent << "type: ";
    Printer<int8_t>::stream(s, indent + "  ", v.type);
  }
};


} // namespace message_operations
} // namespace ros

#endif // HG_HAND_INTERACTION_MESSAGE_HANDGESTURE_H

