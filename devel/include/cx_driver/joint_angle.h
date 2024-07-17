// Generated by gencpp from file cx_driver/joint_angle.msg
// DO NOT EDIT!


#ifndef CX_DRIVER_MESSAGE_JOINT_ANGLE_H
#define CX_DRIVER_MESSAGE_JOINT_ANGLE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cx_driver
{
template <class ContainerAllocator>
struct joint_angle_
{
  typedef joint_angle_<ContainerAllocator> Type;

  joint_angle_()
    : left_arm_joint()
    , right_arm_joint()  {
      left_arm_joint.assign(0.0);

      right_arm_joint.assign(0.0);
  }
  joint_angle_(const ContainerAllocator& _alloc)
    : left_arm_joint()
    , right_arm_joint()  {
  (void)_alloc;
      left_arm_joint.assign(0.0);

      right_arm_joint.assign(0.0);
  }



   typedef boost::array<double, 6>  _left_arm_joint_type;
  _left_arm_joint_type left_arm_joint;

   typedef boost::array<double, 6>  _right_arm_joint_type;
  _right_arm_joint_type right_arm_joint;





  typedef boost::shared_ptr< ::cx_driver::joint_angle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cx_driver::joint_angle_<ContainerAllocator> const> ConstPtr;

}; // struct joint_angle_

typedef ::cx_driver::joint_angle_<std::allocator<void> > joint_angle;

typedef boost::shared_ptr< ::cx_driver::joint_angle > joint_anglePtr;
typedef boost::shared_ptr< ::cx_driver::joint_angle const> joint_angleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cx_driver::joint_angle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cx_driver::joint_angle_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cx_driver::joint_angle_<ContainerAllocator1> & lhs, const ::cx_driver::joint_angle_<ContainerAllocator2> & rhs)
{
  return lhs.left_arm_joint == rhs.left_arm_joint &&
    lhs.right_arm_joint == rhs.right_arm_joint;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cx_driver::joint_angle_<ContainerAllocator1> & lhs, const ::cx_driver::joint_angle_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cx_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cx_driver::joint_angle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cx_driver::joint_angle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cx_driver::joint_angle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cx_driver::joint_angle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cx_driver::joint_angle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cx_driver::joint_angle_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cx_driver::joint_angle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "990341802786d12533e60a9263f3d7c1";
  }

  static const char* value(const ::cx_driver::joint_angle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x990341802786d125ULL;
  static const uint64_t static_value2 = 0x33e60a9263f3d7c1ULL;
};

template<class ContainerAllocator>
struct DataType< ::cx_driver::joint_angle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cx_driver/joint_angle";
  }

  static const char* value(const ::cx_driver::joint_angle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cx_driver::joint_angle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[6] left_arm_joint\n"
"float64[6] right_arm_joint\n"
"\n"
;
  }

  static const char* value(const ::cx_driver::joint_angle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cx_driver::joint_angle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.left_arm_joint);
      stream.next(m.right_arm_joint);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct joint_angle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cx_driver::joint_angle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cx_driver::joint_angle_<ContainerAllocator>& v)
  {
    s << indent << "left_arm_joint[]" << std::endl;
    for (size_t i = 0; i < v.left_arm_joint.size(); ++i)
    {
      s << indent << "  left_arm_joint[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.left_arm_joint[i]);
    }
    s << indent << "right_arm_joint[]" << std::endl;
    for (size_t i = 0; i < v.right_arm_joint.size(); ++i)
    {
      s << indent << "  right_arm_joint[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.right_arm_joint[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CX_DRIVER_MESSAGE_JOINT_ANGLE_H
