// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from kinova_msgs:srv/KinovaExecutor.idl
// generated code does not contain a copyright notice

#ifndef KINOVA_MSGS__SRV__DETAIL__KINOVA_EXECUTOR__TRAITS_HPP_
#define KINOVA_MSGS__SRV__DETAIL__KINOVA_EXECUTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "kinova_msgs/srv/detail/kinova_executor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace kinova_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const KinovaExecutor_Request & msg,
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
  const KinovaExecutor_Request & msg,
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

inline std::string to_yaml(const KinovaExecutor_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace kinova_msgs

namespace rosidl_generator_traits
{

[[deprecated("use kinova_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const kinova_msgs::srv::KinovaExecutor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  kinova_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kinova_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const kinova_msgs::srv::KinovaExecutor_Request & msg)
{
  return kinova_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<kinova_msgs::srv::KinovaExecutor_Request>()
{
  return "kinova_msgs::srv::KinovaExecutor_Request";
}

template<>
inline const char * name<kinova_msgs::srv::KinovaExecutor_Request>()
{
  return "kinova_msgs/srv/KinovaExecutor_Request";
}

template<>
struct has_fixed_size<kinova_msgs::srv::KinovaExecutor_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<kinova_msgs::srv::KinovaExecutor_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<kinova_msgs::srv::KinovaExecutor_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace kinova_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const KinovaExecutor_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const KinovaExecutor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const KinovaExecutor_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace kinova_msgs

namespace rosidl_generator_traits
{

[[deprecated("use kinova_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const kinova_msgs::srv::KinovaExecutor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  kinova_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kinova_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const kinova_msgs::srv::KinovaExecutor_Response & msg)
{
  return kinova_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<kinova_msgs::srv::KinovaExecutor_Response>()
{
  return "kinova_msgs::srv::KinovaExecutor_Response";
}

template<>
inline const char * name<kinova_msgs::srv::KinovaExecutor_Response>()
{
  return "kinova_msgs/srv/KinovaExecutor_Response";
}

template<>
struct has_fixed_size<kinova_msgs::srv::KinovaExecutor_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<kinova_msgs::srv::KinovaExecutor_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<kinova_msgs::srv::KinovaExecutor_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<kinova_msgs::srv::KinovaExecutor>()
{
  return "kinova_msgs::srv::KinovaExecutor";
}

template<>
inline const char * name<kinova_msgs::srv::KinovaExecutor>()
{
  return "kinova_msgs/srv/KinovaExecutor";
}

template<>
struct has_fixed_size<kinova_msgs::srv::KinovaExecutor>
  : std::integral_constant<
    bool,
    has_fixed_size<kinova_msgs::srv::KinovaExecutor_Request>::value &&
    has_fixed_size<kinova_msgs::srv::KinovaExecutor_Response>::value
  >
{
};

template<>
struct has_bounded_size<kinova_msgs::srv::KinovaExecutor>
  : std::integral_constant<
    bool,
    has_bounded_size<kinova_msgs::srv::KinovaExecutor_Request>::value &&
    has_bounded_size<kinova_msgs::srv::KinovaExecutor_Response>::value
  >
{
};

template<>
struct is_service<kinova_msgs::srv::KinovaExecutor>
  : std::true_type
{
};

template<>
struct is_service_request<kinova_msgs::srv::KinovaExecutor_Request>
  : std::true_type
{
};

template<>
struct is_service_response<kinova_msgs::srv::KinovaExecutor_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // KINOVA_MSGS__SRV__DETAIL__KINOVA_EXECUTOR__TRAITS_HPP_
