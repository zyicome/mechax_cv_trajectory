// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Armors.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__ARMORS__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__ARMORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/armors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Armors_armors
{
public:
  explicit Init_Armors_armors(::auto_aim_interfaces::msg::Armors & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Armors armors(::auto_aim_interfaces::msg::Armors::_armors_type arg)
  {
    msg_.armors = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armors msg_;
};

class Init_Armors_header
{
public:
  Init_Armors_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Armors_armors header(::auto_aim_interfaces::msg::Armors::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Armors_armors(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Armors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Armors>()
{
  return auto_aim_interfaces::msg::builder::Init_Armors_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__ARMORS__BUILDER_HPP_
