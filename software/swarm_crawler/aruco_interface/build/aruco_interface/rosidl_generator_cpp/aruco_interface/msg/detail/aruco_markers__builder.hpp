// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from aruco_interface:msg/ArucoMarkers.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACE__MSG__DETAIL__ARUCO_MARKERS__BUILDER_HPP_
#define ARUCO_INTERFACE__MSG__DETAIL__ARUCO_MARKERS__BUILDER_HPP_

#include "aruco_interface/msg/detail/aruco_markers__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace aruco_interface
{

namespace msg
{

namespace builder
{

class Init_ArucoMarkers_poses
{
public:
  explicit Init_ArucoMarkers_poses(::aruco_interface::msg::ArucoMarkers & msg)
  : msg_(msg)
  {}
  ::aruco_interface::msg::ArucoMarkers poses(::aruco_interface::msg::ArucoMarkers::_poses_type arg)
  {
    msg_.poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::aruco_interface::msg::ArucoMarkers msg_;
};

class Init_ArucoMarkers_marker_ids
{
public:
  explicit Init_ArucoMarkers_marker_ids(::aruco_interface::msg::ArucoMarkers & msg)
  : msg_(msg)
  {}
  Init_ArucoMarkers_poses marker_ids(::aruco_interface::msg::ArucoMarkers::_marker_ids_type arg)
  {
    msg_.marker_ids = std::move(arg);
    return Init_ArucoMarkers_poses(msg_);
  }

private:
  ::aruco_interface::msg::ArucoMarkers msg_;
};

class Init_ArucoMarkers_header
{
public:
  Init_ArucoMarkers_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArucoMarkers_marker_ids header(::aruco_interface::msg::ArucoMarkers::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ArucoMarkers_marker_ids(msg_);
  }

private:
  ::aruco_interface::msg::ArucoMarkers msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::aruco_interface::msg::ArucoMarkers>()
{
  return aruco_interface::msg::builder::Init_ArucoMarkers_header();
}

}  // namespace aruco_interface

#endif  // ARUCO_INTERFACE__MSG__DETAIL__ARUCO_MARKERS__BUILDER_HPP_
