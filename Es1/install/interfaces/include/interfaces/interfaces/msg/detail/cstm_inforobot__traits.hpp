// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/CstmInforobot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/cstm_inforobot.hpp"


#ifndef INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/cstm_inforobot__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CstmInforobot & msg,
  std::ostream & out)
{
  out << "{";
  // member: room_id
  {
    out << "room_id: ";
    rosidl_generator_traits::value_to_yaml(msg.room_id, out);
    out << ", ";
  }

  // member: room_name
  {
    out << "room_name: ";
    rosidl_generator_traits::value_to_yaml(msg.room_name, out);
    out << ", ";
  }

  // member: battery_level
  {
    out << "battery_level: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_level, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CstmInforobot & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: room_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "room_id: ";
    rosidl_generator_traits::value_to_yaml(msg.room_id, out);
    out << "\n";
  }

  // member: room_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "room_name: ";
    rosidl_generator_traits::value_to_yaml(msg.room_name, out);
    out << "\n";
  }

  // member: battery_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_level: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_level, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CstmInforobot & msg, bool use_flow_style = false)
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

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::msg::CstmInforobot & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::CstmInforobot & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::CstmInforobot>()
{
  return "interfaces::msg::CstmInforobot";
}

template<>
inline const char * name<interfaces::msg::CstmInforobot>()
{
  return "interfaces/msg/CstmInforobot";
}

template<>
struct has_fixed_size<interfaces::msg::CstmInforobot>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::msg::CstmInforobot>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::msg::CstmInforobot>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__TRAITS_HPP_
