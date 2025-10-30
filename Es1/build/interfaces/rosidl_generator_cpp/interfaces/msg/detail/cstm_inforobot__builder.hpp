// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/CstmInforobot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/cstm_inforobot.hpp"


#ifndef INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/cstm_inforobot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_CstmInforobot_battery_level
{
public:
  explicit Init_CstmInforobot_battery_level(::interfaces::msg::CstmInforobot & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::CstmInforobot battery_level(::interfaces::msg::CstmInforobot::_battery_level_type arg)
  {
    msg_.battery_level = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::CstmInforobot msg_;
};

class Init_CstmInforobot_room_name
{
public:
  explicit Init_CstmInforobot_room_name(::interfaces::msg::CstmInforobot & msg)
  : msg_(msg)
  {}
  Init_CstmInforobot_battery_level room_name(::interfaces::msg::CstmInforobot::_room_name_type arg)
  {
    msg_.room_name = std::move(arg);
    return Init_CstmInforobot_battery_level(msg_);
  }

private:
  ::interfaces::msg::CstmInforobot msg_;
};

class Init_CstmInforobot_room_id
{
public:
  Init_CstmInforobot_room_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CstmInforobot_room_name room_id(::interfaces::msg::CstmInforobot::_room_id_type arg)
  {
    msg_.room_id = std::move(arg);
    return Init_CstmInforobot_room_name(msg_);
  }

private:
  ::interfaces::msg::CstmInforobot msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::CstmInforobot>()
{
  return interfaces::msg::builder::Init_CstmInforobot_room_id();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__BUILDER_HPP_
