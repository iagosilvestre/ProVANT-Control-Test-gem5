// Generated by gencpp from file simulator_msgs/Sensor.msg
// DO NOT EDIT!


#ifndef SIMULATOR_MSGS_MESSAGE_SENSOR_H
#define SIMULATOR_MSGS_MESSAGE_SENSOR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace simulator_msgs
{
template <class ContainerAllocator>
struct Sensor_
{
  typedef Sensor_<ContainerAllocator> Type;

  Sensor_()
    : header()
    , name()
    , values()  {
    }
  Sensor_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , name(_alloc)
    , values(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _values_type;
  _values_type values;




  typedef boost::shared_ptr< ::simulator_msgs::Sensor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::simulator_msgs::Sensor_<ContainerAllocator> const> ConstPtr;

}; // struct Sensor_

typedef ::simulator_msgs::Sensor_<std::allocator<void> > Sensor;

typedef boost::shared_ptr< ::simulator_msgs::Sensor > SensorPtr;
typedef boost::shared_ptr< ::simulator_msgs::Sensor const> SensorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::simulator_msgs::Sensor_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::simulator_msgs::Sensor_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace simulator_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'simulator_msgs': ['/home/brenner/catkin_ws/src/ProVANT-Simulator/source/Structure/simulator_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::simulator_msgs::Sensor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::simulator_msgs::Sensor_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simulator_msgs::Sensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simulator_msgs::Sensor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simulator_msgs::Sensor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simulator_msgs::Sensor_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::simulator_msgs::Sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fa4d9ddb04812e8a8f36b92fd20aeb99";
  }

  static const char* value(const ::simulator_msgs::Sensor_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfa4d9ddb04812e8aULL;
  static const uint64_t static_value2 = 0x8f36b92fd20aeb99ULL;
};

template<class ContainerAllocator>
struct DataType< ::simulator_msgs::Sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "simulator_msgs/Sensor";
  }

  static const char* value(const ::simulator_msgs::Sensor_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::simulator_msgs::Sensor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
string name\n\
float64[] values\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::simulator_msgs::Sensor_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::simulator_msgs::Sensor_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.name);
      stream.next(m.values);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Sensor_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::simulator_msgs::Sensor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::simulator_msgs::Sensor_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "values[]" << std::endl;
    for (size_t i = 0; i < v.values.size(); ++i)
    {
      s << indent << "  values[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.values[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SIMULATOR_MSGS_MESSAGE_SENSOR_H
