// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/ReceiveSerial.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__RECEIVE_SERIAL__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__RECEIVE_SERIAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/receive_serial__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_ReceiveSerial_pitch
{
public:
  explicit Init_ReceiveSerial_pitch(::auto_aim_interfaces::msg::ReceiveSerial & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::ReceiveSerial pitch(::auto_aim_interfaces::msg::ReceiveSerial::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::ReceiveSerial msg_;
};

class Init_ReceiveSerial_yaw
{
public:
  explicit Init_ReceiveSerial_yaw(::auto_aim_interfaces::msg::ReceiveSerial & msg)
  : msg_(msg)
  {}
  Init_ReceiveSerial_pitch yaw(::auto_aim_interfaces::msg::ReceiveSerial::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_ReceiveSerial_pitch(msg_);
  }

private:
  ::auto_aim_interfaces::msg::ReceiveSerial msg_;
};

class Init_ReceiveSerial_is_shoot
{
public:
  explicit Init_ReceiveSerial_is_shoot(::auto_aim_interfaces::msg::ReceiveSerial & msg)
  : msg_(msg)
  {}
  Init_ReceiveSerial_yaw is_shoot(::auto_aim_interfaces::msg::ReceiveSerial::_is_shoot_type arg)
  {
    msg_.is_shoot = std::move(arg);
    return Init_ReceiveSerial_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::ReceiveSerial msg_;
};

class Init_ReceiveSerial_header
{
public:
  Init_ReceiveSerial_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReceiveSerial_is_shoot header(::auto_aim_interfaces::msg::ReceiveSerial::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ReceiveSerial_is_shoot(msg_);
  }

private:
  ::auto_aim_interfaces::msg::ReceiveSerial msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::ReceiveSerial>()
{
  return auto_aim_interfaces::msg::builder::Init_ReceiveSerial_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__RECEIVE_SERIAL__BUILDER_HPP_
