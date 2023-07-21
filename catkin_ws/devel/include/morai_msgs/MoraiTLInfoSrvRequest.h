// Generated by gencpp from file morai_msgs/MoraiTLInfoSrvRequest.msg
// DO NOT EDIT!


#ifndef MORAI_MSGS_MESSAGE_MORAITLINFOSRVREQUEST_H
#define MORAI_MSGS_MESSAGE_MORAITLINFOSRVREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <morai_msgs/MoraiTLIndex.h>

namespace morai_msgs
{
template <class ContainerAllocator>
struct MoraiTLInfoSrvRequest_
{
  typedef MoraiTLInfoSrvRequest_<ContainerAllocator> Type;

  MoraiTLInfoSrvRequest_()
    : request()  {
    }
  MoraiTLInfoSrvRequest_(const ContainerAllocator& _alloc)
    : request(_alloc)  {
  (void)_alloc;
    }



   typedef  ::morai_msgs::MoraiTLIndex_<ContainerAllocator>  _request_type;
  _request_type request;





  typedef boost::shared_ptr< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> const> ConstPtr;

}; // struct MoraiTLInfoSrvRequest_

typedef ::morai_msgs::MoraiTLInfoSrvRequest_<std::allocator<void> > MoraiTLInfoSrvRequest;

typedef boost::shared_ptr< ::morai_msgs::MoraiTLInfoSrvRequest > MoraiTLInfoSrvRequestPtr;
typedef boost::shared_ptr< ::morai_msgs::MoraiTLInfoSrvRequest const> MoraiTLInfoSrvRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator1> & lhs, const ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator2> & rhs)
{
  return lhs.request == rhs.request;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator1> & lhs, const ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace morai_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c5e0616a2a9276c5d4842fa9a4a59f3f";
  }

  static const char* value(const ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc5e0616a2a9276c5ULL;
  static const uint64_t static_value2 = 0xd4842fa9a4a59f3fULL;
};

template<class ContainerAllocator>
struct DataType< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "morai_msgs/MoraiTLInfoSrvRequest";
  }

  static const char* value(const ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "MoraiTLIndex request\n"
"\n"
"================================================================================\n"
"MSG: morai_msgs/MoraiTLIndex\n"
"string idx\n"
;
  }

  static const char* value(const ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.request);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoraiTLInfoSrvRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::morai_msgs::MoraiTLInfoSrvRequest_<ContainerAllocator>& v)
  {
    s << indent << "request: ";
    s << std::endl;
    Printer< ::morai_msgs::MoraiTLIndex_<ContainerAllocator> >::stream(s, indent + "  ", v.request);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MORAI_MSGS_MESSAGE_MORAITLINFOSRVREQUEST_H