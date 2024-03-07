// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/Bias.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Bias in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__Bias
{
  double needchangeyaw;
  bool is_can_hit;
} auto_aim_interfaces__msg__Bias;

// Struct for a sequence of auto_aim_interfaces__msg__Bias.
typedef struct auto_aim_interfaces__msg__Bias__Sequence
{
  auto_aim_interfaces__msg__Bias * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__Bias__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__STRUCT_H_
