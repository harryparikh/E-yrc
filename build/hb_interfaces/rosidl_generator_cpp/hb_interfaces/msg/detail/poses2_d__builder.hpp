// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hb_interfaces:msg/Poses2D.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__POSES2_D__BUILDER_HPP_
#define HB_INTERFACES__MSG__DETAIL__POSES2_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hb_interfaces/msg/detail/poses2_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hb_interfaces
{

namespace msg
{

namespace builder
{

class Init_Poses2D_poses
{
public:
  Init_Poses2D_poses()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hb_interfaces::msg::Poses2D poses(::hb_interfaces::msg::Poses2D::_poses_type arg)
  {
    msg_.poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hb_interfaces::msg::Poses2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hb_interfaces::msg::Poses2D>()
{
  return hb_interfaces::msg::builder::Init_Poses2D_poses();
}

}  // namespace hb_interfaces

#endif  // HB_INTERFACES__MSG__DETAIL__POSES2_D__BUILDER_HPP_
