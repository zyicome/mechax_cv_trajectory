// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Bias.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/bias__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Bias_is_can_hit
{
public:
  explicit Init_Bias_is_can_hit(::auto_aim_interfaces::msg::Bias & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Bias is_can_hit(::auto_aim_interfaces::msg::Bias::_is_can_hit_type arg)
  {
    msg_.is_can_hit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Bias msg_;
};

class Init_Bias_needchangeyaw
{
public:
  Init_Bias_needchangeyaw()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Bias_is_can_hit needchangeyaw(::auto_aim_interfaces::msg::Bias::_needchangeyaw_type arg)
  {
    msg_.needchangeyaw = std::move(arg);
    return Init_Bias_is_can_hit(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Bias msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Bias>()
{
  return auto_aim_interfaces::msg::builder::Init_Bias_needchangeyaw();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__BUILDER_HPP_
