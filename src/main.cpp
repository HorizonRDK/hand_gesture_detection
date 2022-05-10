// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <memory>
#include <string>

#include "include/hand_gesture_det_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_WARN(rclcpp::get_logger("hand gesture det pkg"),
              "This is hand gesture det pkg!");

  rclcpp::spin(
      std::make_shared<inference::HandGestureDetNode>("hand_gesture_det"));

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("hand gesture det pkg"), "Pkg exit.");
  return 0;
}
