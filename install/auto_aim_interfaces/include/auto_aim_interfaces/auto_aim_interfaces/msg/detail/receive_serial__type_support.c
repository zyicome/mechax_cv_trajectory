// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from auto_aim_interfaces:msg/ReceiveSerial.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "auto_aim_interfaces/msg/detail/receive_serial__rosidl_typesupport_introspection_c.h"
#include "auto_aim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "auto_aim_interfaces/msg/detail/receive_serial__functions.h"
#include "auto_aim_interfaces/msg/detail/receive_serial__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  auto_aim_interfaces__msg__ReceiveSerial__init(message_memory);
}

void auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_fini_function(void * message_memory)
{
  auto_aim_interfaces__msg__ReceiveSerial__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__ReceiveSerial, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_shoot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__ReceiveSerial, is_shoot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__ReceiveSerial, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__ReceiveSerial, pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_message_members = {
  "auto_aim_interfaces__msg",  // message namespace
  "ReceiveSerial",  // message name
  4,  // number of fields
  sizeof(auto_aim_interfaces__msg__ReceiveSerial),
  auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_message_member_array,  // message members
  auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_init_function,  // function to initialize message memory (memory has to be allocated)
  auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_message_type_support_handle = {
  0,
  &auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_auto_aim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, auto_aim_interfaces, msg, ReceiveSerial)() {
  auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_message_type_support_handle.typesupport_identifier) {
    auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &auto_aim_interfaces__msg__ReceiveSerial__rosidl_typesupport_introspection_c__ReceiveSerial_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
