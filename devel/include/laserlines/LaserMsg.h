/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/nicholas/openrov/src/laserlines/msg/LaserMsg.msg
 *
 */


#ifndef LASERLINES_MESSAGE_LASERMSG_H
#define LASERLINES_MESSAGE_LASERMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace laserlines
{
template <class ContainerAllocator>
struct LaserMsg_
{
  typedef LaserMsg_<ContainerAllocator> Type;

  LaserMsg_()
    : header()
    , angle_min(0.0)
    , angle_max(0.0)
    , angle_increment(0.0)
    , ranges()  {
    }
  LaserMsg_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , angle_min(0.0)
    , angle_max(0.0)
    , angle_increment(0.0)
    , ranges(_alloc)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _angle_min_type;
  _angle_min_type angle_min;

   typedef float _angle_max_type;
  _angle_max_type angle_max;

   typedef float _angle_increment_type;
  _angle_increment_type angle_increment;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _ranges_type;
  _ranges_type ranges;




  typedef boost::shared_ptr< ::laserlines::LaserMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::laserlines::LaserMsg_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct LaserMsg_

typedef ::laserlines::LaserMsg_<std::allocator<void> > LaserMsg;

typedef boost::shared_ptr< ::laserlines::LaserMsg > LaserMsgPtr;
typedef boost::shared_ptr< ::laserlines::LaserMsg const> LaserMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::laserlines::LaserMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::laserlines::LaserMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace laserlines

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'laserlines': ['/home/nicholas/openrov/src/laserlines/msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::laserlines::LaserMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::laserlines::LaserMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::laserlines::LaserMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::laserlines::LaserMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::laserlines::LaserMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::laserlines::LaserMsg_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::laserlines::LaserMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0aa13076bebd983925adfce2780dac6e";
  }

  static const char* value(const ::laserlines::LaserMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0aa13076bebd9839ULL;
  static const uint64_t static_value2 = 0x25adfce2780dac6eULL;
};

template<class ContainerAllocator>
struct DataType< ::laserlines::LaserMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "laserlines/LaserMsg";
  }

  static const char* value(const ::laserlines::LaserMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::laserlines::LaserMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float32 angle_min\n\
float32 angle_max\n\
float32 angle_increment\n\
float32[] ranges\n\
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
";
  }

  static const char* value(const ::laserlines::LaserMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::laserlines::LaserMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.angle_min);
      stream.next(m.angle_max);
      stream.next(m.angle_increment);
      stream.next(m.ranges);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct LaserMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::laserlines::LaserMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::laserlines::LaserMsg_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "angle_min: ";
    Printer<float>::stream(s, indent + "  ", v.angle_min);
    s << indent << "angle_max: ";
    Printer<float>::stream(s, indent + "  ", v.angle_max);
    s << indent << "angle_increment: ";
    Printer<float>::stream(s, indent + "  ", v.angle_increment);
    s << indent << "ranges[]" << std::endl;
    for (size_t i = 0; i < v.ranges.size(); ++i)
    {
      s << indent << "  ranges[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.ranges[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // LASERLINES_MESSAGE_LASERMSG_H
