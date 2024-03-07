// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from auto_aim_interfaces:msg/Bias.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/bias__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "auto_aim_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "auto_aim_interfaces/msg/detail/bias__struct.h"
#include "auto_aim_interfaces/msg/detail/bias__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _Bias__ros_msg_type = auto_aim_interfaces__msg__Bias;

static bool _Bias__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Bias__ros_msg_type * ros_message = static_cast<const _Bias__ros_msg_type *>(untyped_ros_message);
  // Field name: needchangeyaw
  {
    cdr << ros_message->needchangeyaw;
  }

  // Field name: is_can_hit
  {
    cdr << (ros_message->is_can_hit ? true : false);
  }

  return true;
}

static bool _Bias__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Bias__ros_msg_type * ros_message = static_cast<_Bias__ros_msg_type *>(untyped_ros_message);
  // Field name: needchangeyaw
  {
    cdr >> ros_message->needchangeyaw;
  }

  // Field name: is_can_hit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_can_hit = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auto_aim_interfaces
size_t get_serialized_size_auto_aim_interfaces__msg__Bias(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Bias__ros_msg_type * ros_message = static_cast<const _Bias__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name needchangeyaw
  {
    size_t item_size = sizeof(ros_message->needchangeyaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_can_hit
  {
    size_t item_size = sizeof(ros_message->is_can_hit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Bias__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_auto_aim_interfaces__msg__Bias(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_auto_aim_interfaces
size_t max_serialized_size_auto_aim_interfaces__msg__Bias(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: needchangeyaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: is_can_hit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = auto_aim_interfaces__msg__Bias;
    is_plain =
      (
      offsetof(DataType, is_can_hit) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Bias__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_auto_aim_interfaces__msg__Bias(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Bias = {
  "auto_aim_interfaces::msg",
  "Bias",
  _Bias__cdr_serialize,
  _Bias__cdr_deserialize,
  _Bias__get_serialized_size,
  _Bias__max_serialized_size
};

static rosidl_message_type_support_t _Bias__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Bias,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, auto_aim_interfaces, msg, Bias)() {
  return &_Bias__type_support;
}

#if defined(__cplusplus)
}
#endif
