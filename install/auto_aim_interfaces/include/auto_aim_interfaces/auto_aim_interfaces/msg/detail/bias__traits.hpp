// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/Bias.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/bias__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Bias & msg,
  std::ostream & out)
{
  out << "{";
  // member: needchangeyaw
  {
    out << "needchangeyaw: ";
    rosidl_generator_traits::value_to_yaml(msg.needchangeyaw, out);
    out << ", ";
  }

  // member: is_can_hit
  {
    out << "is_can_hit: ";
    rosidl_generator_traits::value_to_yaml(msg.is_can_hit, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Bias & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: needchangeyaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "needchangeyaw: ";
    rosidl_generator_traits::value_to_yaml(msg.needchangeyaw, out);
    out << "\n";
  }

  // member: is_can_hit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_can_hit: ";
    rosidl_generator_traits::value_to_yaml(msg.is_can_hit, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Bias & msg, bool use_flow_style = false)
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

}  // namespace auto_aim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use auto_aim_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const auto_aim_interfaces::msg::Bias & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::Bias & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::Bias>()
{
  return "auto_aim_interfaces::msg::Bias";
}

template<>
inline const char * name<auto_aim_interfaces::msg::Bias>()
{
  return "auto_aim_interfaces/msg/Bias";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::Bias>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::Bias>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<auto_aim_interfaces::msg::Bias>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__TRAITS_HPP_
