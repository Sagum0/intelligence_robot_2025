// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kinova_msgs:srv/KinovaExecutor.idl
// generated code does not contain a copyright notice

#ifndef KINOVA_MSGS__SRV__DETAIL__KINOVA_EXECUTOR__STRUCT_H_
#define KINOVA_MSGS__SRV__DETAIL__KINOVA_EXECUTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'frame'
#include "rosidl_runtime_c/string.h"
// Member 'coordinate'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/KinovaExecutor in the package kinova_msgs.
typedef struct kinova_msgs__srv__KinovaExecutor_Request
{
  rosidl_runtime_c__String frame;
  rosidl_runtime_c__float__Sequence coordinate;
} kinova_msgs__srv__KinovaExecutor_Request;

// Struct for a sequence of kinova_msgs__srv__KinovaExecutor_Request.
typedef struct kinova_msgs__srv__KinovaExecutor_Request__Sequence
{
  kinova_msgs__srv__KinovaExecutor_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kinova_msgs__srv__KinovaExecutor_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/KinovaExecutor in the package kinova_msgs.
typedef struct kinova_msgs__srv__KinovaExecutor_Response
{
  bool success;
  rosidl_runtime_c__String message;
} kinova_msgs__srv__KinovaExecutor_Response;

// Struct for a sequence of kinova_msgs__srv__KinovaExecutor_Response.
typedef struct kinova_msgs__srv__KinovaExecutor_Response__Sequence
{
  kinova_msgs__srv__KinovaExecutor_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kinova_msgs__srv__KinovaExecutor_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KINOVA_MSGS__SRV__DETAIL__KINOVA_EXECUTOR__STRUCT_H_
