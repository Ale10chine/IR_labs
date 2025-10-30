// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/CstmInforobot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/cstm_inforobot.hpp"


#ifndef INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__CstmInforobot __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__CstmInforobot __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CstmInforobot_
{
  using Type = CstmInforobot_<ContainerAllocator>;

  explicit CstmInforobot_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->room_id = 0l;
      this->room_name = "";
      this->battery_level = 0.0;
    }
  }

  explicit CstmInforobot_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : room_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->room_id = 0l;
      this->room_name = "";
      this->battery_level = 0.0;
    }
  }

  // field types and members
  using _room_id_type =
    int32_t;
  _room_id_type room_id;
  using _room_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _room_name_type room_name;
  using _battery_level_type =
    double;
  _battery_level_type battery_level;

  // setters for named parameter idiom
  Type & set__room_id(
    const int32_t & _arg)
  {
    this->room_id = _arg;
    return *this;
  }
  Type & set__room_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->room_name = _arg;
    return *this;
  }
  Type & set__battery_level(
    const double & _arg)
  {
    this->battery_level = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::CstmInforobot_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::CstmInforobot_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::CstmInforobot_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::CstmInforobot_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::CstmInforobot_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::CstmInforobot_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::CstmInforobot_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::CstmInforobot_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::CstmInforobot_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::CstmInforobot_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__CstmInforobot
    std::shared_ptr<interfaces::msg::CstmInforobot_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__CstmInforobot
    std::shared_ptr<interfaces::msg::CstmInforobot_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CstmInforobot_ & other) const
  {
    if (this->room_id != other.room_id) {
      return false;
    }
    if (this->room_name != other.room_name) {
      return false;
    }
    if (this->battery_level != other.battery_level) {
      return false;
    }
    return true;
  }
  bool operator!=(const CstmInforobot_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CstmInforobot_

// alias to use template instance with default allocator
using CstmInforobot =
  interfaces::msg::CstmInforobot_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__STRUCT_HPP_
