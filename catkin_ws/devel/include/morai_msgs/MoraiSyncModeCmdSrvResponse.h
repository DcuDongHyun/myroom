// Generated by gencpp from file morai_msgs/MoraiSyncModeCmdSrvResponse.msg
// DO NOT EDIT!


#ifndef MORAI_MSGS_MESSAGE_MORAISYNCMODECMDSRVRESPONSE_H
#define MORAI_MSGS_MESSAGE_MORAISYNCMODECMDSRVRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <morai_msgs/SyncModeCmdResponse.h>

namespace morai_msgs
{
template <class ContainerAllocator>
struct MoraiSyncModeCmdSrvResponse_
{
  typedef MoraiSyncModeCmdSrvResponse_<ContainerAllocator> Type;

  MoraiSyncModeCmdSrvResponse_()
    : response()  {
    }
  MoraiSyncModeCmdSrvResponse_(const ContainerAllocator& _alloc)
    : response(_alloc)  {
  (void)_alloc;
    }



   typedef  ::morai_msgs::SyncModeCmdResponse_<ContainerAllocator>  _response_type;
  _response_type response;





  typedef boost::shared_ptr< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> const> ConstPtr;

}; // struct MoraiSyncModeCmdSrvResponse_

typedef ::morai_msgs::MoraiSyncModeCmdSrvResponse_<std::allocator<void> > MoraiSyncModeCmdSrvResponse;

typedef boost::shared_ptr< ::morai_msgs::MoraiSyncModeCmdSrvResponse > MoraiSyncModeCmdSrvResponsePtr;
typedef boost::shared_ptr< ::morai_msgs::MoraiSyncModeCmdSrvResponse const> MoraiSyncModeCmdSrvResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator1> & lhs, const ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator2> & rhs)
{
  return lhs.response == rhs.response;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator1> & lhs, const ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace morai_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f5aeea6ec21f5f08b319408453475055";
  }

  static const char* value(const ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf5aeea6ec21f5f08ULL;
  static const uint64_t static_value2 = 0xb319408453475055ULL;
};

template<class ContainerAllocator>
struct DataType< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "morai_msgs/MoraiSyncModeCmdSrvResponse";
  }

  static const char* value(const ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "SyncModeCmdResponse response\n"
"\n"
"\n"
"================================================================================\n"
"MSG: morai_msgs/SyncModeCmdResponse\n"
"string user_id\n"
"uint64 frame\n"
"bool result\n"
"uint32 time_step\n"
;
  }

  static const char* value(const ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.response);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoraiSyncModeCmdSrvResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::morai_msgs::MoraiSyncModeCmdSrvResponse_<ContainerAllocator>& v)
  {
    s << indent << "response: ";
    s << std::endl;
    Printer< ::morai_msgs::SyncModeCmdResponse_<ContainerAllocator> >::stream(s, indent + "  ", v.response);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MORAI_MSGS_MESSAGE_MORAISYNCMODECMDSRVRESPONSE_H
