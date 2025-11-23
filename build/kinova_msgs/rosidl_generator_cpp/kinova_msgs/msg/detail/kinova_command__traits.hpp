// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from kinova_msgs:msg/KinovaCommand.idl
// generated code does not contain a copyright notice

#ifndef KINOVA_MSGS__MSG__DETAIL__KINOVA_COMMAND__TRAITS_HPP_
#define KINOVA_MSGS__MSG__DETAIL__KINOVA_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "kinova_msgs/msg/detail/kinova_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace kinova_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const KinovaCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: frame
  {
    out << "frame: ";
    rosidl_generator_traits::value_to_yaml(msg.frame, out);
    out << ", ";
  }

  // member: coordinate
  {
    if (msg.coordinate.size() == 0) {
      out << "coordinate: []";
    } else {
      out << "coordinate: [";
      size_t pending_items = msg.coordinate.size();
      for (auto item : msg.coordinate) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const KinovaCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame: ";
    rosidl_generator_traits::value_to_yaml(msg.frame, out);
    out << "\n";
  }

  // member: coordinate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coordinate.size() == 0) {
      out << "coordinate: []\n";
    } else {
      out << "coordinate:\n";
      for (auto item : msg.coordinate) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const KinovaCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace kinova_msgs

namespace rosidl_generator_traits
{

[[deprecated("use kinova_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const kinova_msgs::msg::KinovaCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  kinova_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kinova_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const kinova_msgs::msg::KinovaCommand & msg)
{
  return kinova_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<kinova_msgs::msg::KinovaCommand>()
{
  return "kinova_msgs::msg::KinovaCommand";
}

template<>
inline const char * name<kinova_msgs::msg::KinovaCommand>()
{
  return "kinova_msgs/msg/KinovaCommand";
}

template<>
struct has_fixed_size<kinova_msgs::msg::KinovaCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<kinova_msgs::msg::KinovaCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<kinova_msgs::msg::KinovaCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // KINOVA_MSGS__MSG__DETAIL__KINOVA_COMMAND__TRAITS_HPP_
