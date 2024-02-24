// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from auto_aim_interfaces:msg/DebugArmor.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMOR__FUNCTIONS_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMOR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "auto_aim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "auto_aim_interfaces/msg/detail/debug_armor__struct.h"

/// Initialize msg/DebugArmor message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * auto_aim_interfaces__msg__DebugArmor
 * )) before or use
 * auto_aim_interfaces__msg__DebugArmor__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__DebugArmor__init(auto_aim_interfaces__msg__DebugArmor * msg);

/// Finalize msg/DebugArmor message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__DebugArmor__fini(auto_aim_interfaces__msg__DebugArmor * msg);

/// Create msg/DebugArmor message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * auto_aim_interfaces__msg__DebugArmor__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
auto_aim_interfaces__msg__DebugArmor *
auto_aim_interfaces__msg__DebugArmor__create();

/// Destroy msg/DebugArmor message.
/**
 * It calls
 * auto_aim_interfaces__msg__DebugArmor__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__DebugArmor__destroy(auto_aim_interfaces__msg__DebugArmor * msg);

/// Check for msg/DebugArmor message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__DebugArmor__are_equal(const auto_aim_interfaces__msg__DebugArmor * lhs, const auto_aim_interfaces__msg__DebugArmor * rhs);

/// Copy a msg/DebugArmor message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__DebugArmor__copy(
  const auto_aim_interfaces__msg__DebugArmor * input,
  auto_aim_interfaces__msg__DebugArmor * output);

/// Initialize array of msg/DebugArmor messages.
/**
 * It allocates the memory for the number of elements and calls
 * auto_aim_interfaces__msg__DebugArmor__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__DebugArmor__Sequence__init(auto_aim_interfaces__msg__DebugArmor__Sequence * array, size_t size);

/// Finalize array of msg/DebugArmor messages.
/**
 * It calls
 * auto_aim_interfaces__msg__DebugArmor__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__DebugArmor__Sequence__fini(auto_aim_interfaces__msg__DebugArmor__Sequence * array);

/// Create array of msg/DebugArmor messages.
/**
 * It allocates the memory for the array and calls
 * auto_aim_interfaces__msg__DebugArmor__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
auto_aim_interfaces__msg__DebugArmor__Sequence *
auto_aim_interfaces__msg__DebugArmor__Sequence__create(size_t size);

/// Destroy array of msg/DebugArmor messages.
/**
 * It calls
 * auto_aim_interfaces__msg__DebugArmor__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
void
auto_aim_interfaces__msg__DebugArmor__Sequence__destroy(auto_aim_interfaces__msg__DebugArmor__Sequence * array);

/// Check for msg/DebugArmor message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__DebugArmor__Sequence__are_equal(const auto_aim_interfaces__msg__DebugArmor__Sequence * lhs, const auto_aim_interfaces__msg__DebugArmor__Sequence * rhs);

/// Copy an array of msg/DebugArmor messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_auto_aim_interfaces
bool
auto_aim_interfaces__msg__DebugArmor__Sequence__copy(
  const auto_aim_interfaces__msg__DebugArmor__Sequence * input,
  auto_aim_interfaces__msg__DebugArmor__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMOR__FUNCTIONS_H_
