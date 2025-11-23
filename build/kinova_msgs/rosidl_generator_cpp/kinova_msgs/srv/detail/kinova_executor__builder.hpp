// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from kinova_msgs:srv/KinovaExecutor.idl
// generated code does not contain a copyright notice

#ifndef KINOVA_MSGS__SRV__DETAIL__KINOVA_EXECUTOR__BUILDER_HPP_
#define KINOVA_MSGS__SRV__DETAIL__KINOVA_EXECUTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "kinova_msgs/srv/detail/kinova_executor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace kinova_msgs
{

namespace srv
{

namespace builder
{

class Init_KinovaExecutor_Request_coordinate
{
public:
  explicit Init_KinovaExecutor_Request_coordinate(::kinova_msgs::srv::KinovaExecutor_Request & msg)
  : msg_(msg)
  {}
  ::kinova_msgs::srv::KinovaExecutor_Request coordinate(::kinova_msgs::srv::KinovaExecutor_Request::_coordinate_type arg)
  {
    msg_.coordinate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kinova_msgs::srv::KinovaExecutor_Request msg_;
};

class Init_KinovaExecutor_Request_frame
{
public:
  Init_KinovaExecutor_Request_frame()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KinovaExecutor_Request_coordinate frame(::kinova_msgs::srv::KinovaExecutor_Request::_frame_type arg)
  {
    msg_.frame = std::move(arg);
    return Init_KinovaExecutor_Request_coordinate(msg_);
  }

private:
  ::kinova_msgs::srv::KinovaExecutor_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::kinova_msgs::srv::KinovaExecutor_Request>()
{
  return kinova_msgs::srv::builder::Init_KinovaExecutor_Request_frame();
}

}  // namespace kinova_msgs


namespace kinova_msgs
{

namespace srv
{

namespace builder
{

class Init_KinovaExecutor_Response_message
{
public:
  explicit Init_KinovaExecutor_Response_message(::kinova_msgs::srv::KinovaExecutor_Response & msg)
  : msg_(msg)
  {}
  ::kinova_msgs::srv::KinovaExecutor_Response message(::kinova_msgs::srv::KinovaExecutor_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kinova_msgs::srv::KinovaExecutor_Response msg_;
};

class Init_KinovaExecutor_Response_success
{
public:
  Init_KinovaExecutor_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KinovaExecutor_Response_message success(::kinova_msgs::srv::KinovaExecutor_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_KinovaExecutor_Response_message(msg_);
  }

private:
  ::kinova_msgs::srv::KinovaExecutor_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::kinova_msgs::srv::KinovaExecutor_Response>()
{
  return kinova_msgs::srv::builder::Init_KinovaExecutor_Response_success();
}

}  // namespace kinova_msgs

#endif  // KINOVA_MSGS__SRV__DETAIL__KINOVA_EXECUTOR__BUILDER_HPP_
