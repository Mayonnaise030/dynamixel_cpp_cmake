// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef READ_WRITE_NODE2_HPP_
#define READ_WRITE_NODE2_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


class ReadWriteNode2 : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

  ReadWriteNode2();
  virtual ~ReadWriteNode2();

private:
  rclcpp::Publisher<SetPosition>::SharedPtr set_position_publisher_;
  rclcpp::Client<GetPosition>::SharedPtr get_position_client_;
  rclcpp::TimerBase::SharedPtr publish_timer;
  rclcpp::TimerBase::SharedPtr client_timer;
  void publish_callback();
  void client_callback();
  size_t count_;

  int present_position;
};

#endif  // READ_WRITE_NODE_HPP_