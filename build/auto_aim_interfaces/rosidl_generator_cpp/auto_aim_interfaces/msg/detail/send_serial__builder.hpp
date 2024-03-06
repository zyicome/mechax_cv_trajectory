// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/SendSerial.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_SERIAL__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_SERIAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/send_serial__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_SendSerial_pitch
{
public:
  explicit Init_SendSerial_pitch(::auto_aim_interfaces::msg::SendSerial & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::SendSerial pitch(::auto_aim_interfaces::msg::SendSerial::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendSerial msg_;
};

class Init_SendSerial_yaw
{
public:
  explicit Init_SendSerial_yaw(::auto_aim_interfaces::msg::SendSerial & msg)
  : msg_(msg)
  {}
  Init_SendSerial_pitch yaw(::auto_aim_interfaces::msg::SendSerial::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_SendSerial_pitch(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendSerial msg_;
};

class Init_SendSerial_is_tracking
{
public:
  explicit Init_SendSerial_is_tracking(::auto_aim_interfaces::msg::SendSerial & msg)
  : msg_(msg)
  {}
  Init_SendSerial_yaw is_tracking(::auto_aim_interfaces::msg::SendSerial::_is_tracking_type arg)
  {
    msg_.is_tracking = std::move(arg);
    return Init_SendSerial_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendSerial msg_;
};

class Init_SendSerial_header
{
public:
  Init_SendSerial_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendSerial_is_tracking header(::auto_aim_interfaces::msg::SendSerial::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SendSerial_is_tracking(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendSerial msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::SendSerial>()
{
  return auto_aim_interfaces::msg::builder::Init_SendSerial_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_SERIAL__BUILDER_HPP_
