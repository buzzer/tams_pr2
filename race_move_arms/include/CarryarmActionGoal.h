/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-fuerte-common-tutorials-0.2.3/debian/ros-fuerte-common-tutorials/opt/ros/fuerte/stacks/common_tutorials/carryarm_actionlib/msg/CarryarmActionGoal.msg */
#ifndef carryarm_actionlib_MESSAGE_CARRYARMACTIONGOAL_H
#define carryarm_actionlib_MESSAGE_CARRYARMACTIONGOAL_H
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
#include "actionlib_msgs/GoalID.h"
#include "carryarm_actionlib/CarryarmGoal.h"

namespace carryarm_actionlib
{
template <class ContainerAllocator>
struct CarryarmActionGoal_ {
  typedef CarryarmActionGoal_<ContainerAllocator> Type;

  CarryarmActionGoal_()
  : header()
  , goal_id()
  , goal()
  {
  }

  CarryarmActionGoal_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , goal_id(_alloc)
  , goal(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
   ::actionlib_msgs::GoalID_<ContainerAllocator>  goal_id;

  typedef  ::carryarm_actionlib::CarryarmGoal_<ContainerAllocator>  _goal_type;
   ::carryarm_actionlib::CarryarmGoal_<ContainerAllocator>  goal;


  typedef boost::shared_ptr< ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct CarryarmActionGoal
typedef  ::carryarm_actionlib::CarryarmActionGoal_<std::allocator<void> > CarryarmActionGoal;

typedef boost::shared_ptr< ::carryarm_actionlib::CarryarmActionGoal> CarryarmActionGoalPtr;
typedef boost::shared_ptr< ::carryarm_actionlib::CarryarmActionGoal const> CarryarmActionGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace carryarm_actionlib

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "006871c7fa1d0e3d5fe2226bf17b2a94";
  }

  static const char* value(const  ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x006871c7fa1d0e3dULL;
  static const uint64_t static_value2 = 0x5fe2226bf17b2a94ULL;
};

template<class ContainerAllocator>
struct DataType< ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "carryarm_actionlib/CarryarmActionGoal";
  }

  static const char* value(const  ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
CarryarmGoal goal\n\
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
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: carryarm_actionlib/CarryarmGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal definition\n\
int32 carryarm\n\
\n\
";
  }

  static const char* value(const  ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.goal_id);
    stream.next(m.goal);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CarryarmActionGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::carryarm_actionlib::CarryarmActionGoal_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
s << std::endl;
    Printer< ::carryarm_actionlib::CarryarmGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};


} // namespace message_operations
} // namespace ros

#endif // carryarm_actionlib_MESSAGE_CARRYARMACTIONGOAL_H

