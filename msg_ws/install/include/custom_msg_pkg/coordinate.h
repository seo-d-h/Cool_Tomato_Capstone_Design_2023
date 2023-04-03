// Generated by gencpp from file custom_msg_pkg/coordinate.msg
// DO NOT EDIT!


#ifndef CUSTOM_MSG_PKG_MESSAGE_COORDINATE_H
#define CUSTOM_MSG_PKG_MESSAGE_COORDINATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace custom_msg_pkg
{
template <class ContainerAllocator>
struct coordinate_
{
  typedef coordinate_<ContainerAllocator> Type;

  coordinate_()
    : start_time()
    , msg_seq(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  coordinate_(const ContainerAllocator& _alloc)
    : start_time()
    , msg_seq(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef ros::Time _start_time_type;
  _start_time_type start_time;

   typedef uint16_t _msg_seq_type;
  _msg_seq_type msg_seq;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::custom_msg_pkg::coordinate_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::custom_msg_pkg::coordinate_<ContainerAllocator> const> ConstPtr;

}; // struct coordinate_

typedef ::custom_msg_pkg::coordinate_<std::allocator<void> > coordinate;

typedef boost::shared_ptr< ::custom_msg_pkg::coordinate > coordinatePtr;
typedef boost::shared_ptr< ::custom_msg_pkg::coordinate const> coordinateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::custom_msg_pkg::coordinate_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::custom_msg_pkg::coordinate_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::custom_msg_pkg::coordinate_<ContainerAllocator1> & lhs, const ::custom_msg_pkg::coordinate_<ContainerAllocator2> & rhs)
{
  return lhs.start_time == rhs.start_time &&
    lhs.msg_seq == rhs.msg_seq &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::custom_msg_pkg::coordinate_<ContainerAllocator1> & lhs, const ::custom_msg_pkg::coordinate_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace custom_msg_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::custom_msg_pkg::coordinate_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::custom_msg_pkg::coordinate_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msg_pkg::coordinate_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msg_pkg::coordinate_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msg_pkg::coordinate_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msg_pkg::coordinate_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::custom_msg_pkg::coordinate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b83f43423351737928ddd0624ca7902e";
  }

  static const char* value(const ::custom_msg_pkg::coordinate_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb83f434233517379ULL;
  static const uint64_t static_value2 = 0x28ddd0624ca7902eULL;
};

template<class ContainerAllocator>
struct DataType< ::custom_msg_pkg::coordinate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "custom_msg_pkg/coordinate";
  }

  static const char* value(const ::custom_msg_pkg::coordinate_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::custom_msg_pkg::coordinate_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time start_time\n"
"uint16 msg_seq\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::custom_msg_pkg::coordinate_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::custom_msg_pkg::coordinate_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.start_time);
      stream.next(m.msg_seq);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct coordinate_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::custom_msg_pkg::coordinate_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::custom_msg_pkg::coordinate_<ContainerAllocator>& v)
  {
    s << indent << "start_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.start_time);
    s << indent << "msg_seq: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.msg_seq);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CUSTOM_MSG_PKG_MESSAGE_COORDINATE_H
