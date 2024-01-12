// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Serial.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__SERIAL__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__SERIAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/serial__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Serial_yaw
{
public:
  explicit Init_Serial_yaw(::auto_aim_interfaces::msg::Serial & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Serial yaw(::auto_aim_interfaces::msg::Serial::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Serial msg_;
};

class Init_Serial_pitch
{
public:
  explicit Init_Serial_pitch(::auto_aim_interfaces::msg::Serial & msg)
  : msg_(msg)
  {}
  Init_Serial_yaw pitch(::auto_aim_interfaces::msg::Serial::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_Serial_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Serial msg_;
};

class Init_Serial_is_shoot
{
public:
  explicit Init_Serial_is_shoot(::auto_aim_interfaces::msg::Serial & msg)
  : msg_(msg)
  {}
  Init_Serial_pitch is_shoot(::auto_aim_interfaces::msg::Serial::_is_shoot_type arg)
  {
    msg_.is_shoot = std::move(arg);
    return Init_Serial_pitch(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Serial msg_;
};

class Init_Serial_header
{
public:
  Init_Serial_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Serial_is_shoot header(::auto_aim_interfaces::msg::Serial::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Serial_is_shoot(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Serial msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Serial>()
{
  return auto_aim_interfaces::msg::builder::Init_Serial_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__SERIAL__BUILDER_HPP_
