// Generated by gencpp from file morai_msgs/MoraiSyncModeSLSrv.msg
// DO NOT EDIT!


#ifndef MORAI_MSGS_MESSAGE_MORAISYNCMODESLSRV_H
#define MORAI_MSGS_MESSAGE_MORAISYNCMODESLSRV_H

#include <ros/service_traits.h>


#include <morai_msgs/MoraiSyncModeSLSrvRequest.h>
#include <morai_msgs/MoraiSyncModeSLSrvResponse.h>


namespace morai_msgs
{

struct MoraiSyncModeSLSrv
{

typedef MoraiSyncModeSLSrvRequest Request;
typedef MoraiSyncModeSLSrvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct MoraiSyncModeSLSrv
} // namespace morai_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::morai_msgs::MoraiSyncModeSLSrv > {
  static const char* value()
  {
    return "892af6bb455c083a88752e9286fb2b85";
  }

  static const char* value(const ::morai_msgs::MoraiSyncModeSLSrv&) { return value(); }
};

template<>
struct DataType< ::morai_msgs::MoraiSyncModeSLSrv > {
  static const char* value()
  {
    return "morai_msgs/MoraiSyncModeSLSrv";
  }

  static const char* value(const ::morai_msgs::MoraiSyncModeSLSrv&) { return value(); }
};


// service_traits::MD5Sum< ::morai_msgs::MoraiSyncModeSLSrvRequest> should match
// service_traits::MD5Sum< ::morai_msgs::MoraiSyncModeSLSrv >
template<>
struct MD5Sum< ::morai_msgs::MoraiSyncModeSLSrvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::morai_msgs::MoraiSyncModeSLSrv >::value();
  }
  static const char* value(const ::morai_msgs::MoraiSyncModeSLSrvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::morai_msgs::MoraiSyncModeSLSrvRequest> should match
// service_traits::DataType< ::morai_msgs::MoraiSyncModeSLSrv >
template<>
struct DataType< ::morai_msgs::MoraiSyncModeSLSrvRequest>
{
  static const char* value()
  {
    return DataType< ::morai_msgs::MoraiSyncModeSLSrv >::value();
  }
  static const char* value(const ::morai_msgs::MoraiSyncModeSLSrvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::morai_msgs::MoraiSyncModeSLSrvResponse> should match
// service_traits::MD5Sum< ::morai_msgs::MoraiSyncModeSLSrv >
template<>
struct MD5Sum< ::morai_msgs::MoraiSyncModeSLSrvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::morai_msgs::MoraiSyncModeSLSrv >::value();
  }
  static const char* value(const ::morai_msgs::MoraiSyncModeSLSrvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::morai_msgs::MoraiSyncModeSLSrvResponse> should match
// service_traits::DataType< ::morai_msgs::MoraiSyncModeSLSrv >
template<>
struct DataType< ::morai_msgs::MoraiSyncModeSLSrvResponse>
{
  static const char* value()
  {
    return DataType< ::morai_msgs::MoraiSyncModeSLSrv >::value();
  }
  static const char* value(const ::morai_msgs::MoraiSyncModeSLSrvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MORAI_MSGS_MESSAGE_MORAISYNCMODESLSRV_H