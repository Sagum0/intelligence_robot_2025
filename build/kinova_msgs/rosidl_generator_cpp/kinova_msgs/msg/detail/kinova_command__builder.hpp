// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from kinova_msgs:msg/KinovaCommand.idl
// generated code does not contain a copyright notice

#ifndef KINOVA_MSGS__MSG__DETAIL__KINOVA_COMMAND__BUILDER_HPP_
#define KINOVA_MSGS__MSG__DETAIL__KINOVA_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "kinova_msgs/msg/detail/kinova_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace kinova_msgs
{

namespace msg
{

namespace builder
{

class Init_KinovaCommand_coordinate
{
public:
  explicit Init_KinovaCommand_coordinate(::kinova_msgs::msg::KinovaCommand & msg)
  : msg_(msg)
  {}
  ::kinova_msgs::msg::KinovaCommand coordinate(::kinova_msgs::msg::KinovaCommand::_coordinate_type arg)
  {
    msg_.coordinate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kinova_msgs::msg::KinovaCommand msg_;
};

class Init_KinovaCommand_frame
{
public:
  Init_KinovaCommand_frame()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KinovaCommand_coordinate frame(::kinova_msgs::msg::KinovaCommand::_frame_type arg)
  {
    msg_.frame = std::move(arg);
    return Init_KinovaCommand_coordinate(msg_);
  }

private:
  ::kinova_msgs::msg::KinovaCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::kinova_msgs::msg::KinovaCommand>()
{
  return kinova_msgs::msg::builder::Init_KinovaCommand_frame();
}

}  // namespace kinova_msgs

#endif  // KINOVA_MSGS__MSG__DETAIL__KINOVA_COMMAND__BUILDER_HPP_
