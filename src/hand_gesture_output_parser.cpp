// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/hand_gesture_output_parser.h"

#include <memory>
#include <vector>

#include "include/hand_gesture_det_node.h"
#include "rclcpp/rclcpp.hpp"

namespace inference {
int32_t HandGestureOutputParser::Parse(
    std::shared_ptr<DNNResult>& output,
    std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
    std::shared_ptr<OutputDescription>& output_description,
    std::shared_ptr<DNNTensor>& output_tensor) {
  RCLCPP_DEBUG(rclcpp::get_logger("hand gesture parser"),
               "HandGestureOutputParser parse start");

  if (!input_descriptions.empty()) {
    RCLCPP_DEBUG(rclcpp::get_logger("hand lmk parser"),
                 "empty input_descriptions");
  }

  std::shared_ptr<HandGestureResult> hand_gesture_output = nullptr;
  if (!output) {
    hand_gesture_output = std::make_shared<HandGestureResult>();
    hand_gesture_output->Reset();
    output = hand_gesture_output;
  } else {
    hand_gesture_output = std::dynamic_pointer_cast<HandGestureResult>(output);
    hand_gesture_output->Reset();
  }

  auto hand_gesture_output_desc =
      std::dynamic_pointer_cast<HandGestureOutDesc>(output_description);
  if (!hand_gesture_output || !hand_gesture_output_desc ||
      !gesture_post_process_) {
    RCLCPP_WARN(rclcpp::get_logger("parser"), "%s:%d", __FILE__, __LINE__);
    return -1;
  }

  if (hand_gesture_output_desc->input_tensor) {
    std::vector<DNNTensor> tensors{*hand_gesture_output_desc->input_tensor};
    if (FreeTensor(tensors) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("parser"), "free tensor fail!");
    }
  }

  auto track_id = hand_gesture_output_desc->track_id;
  hand_gesture_output->gesture_res = gesture_post_process_->Execute(
      output_tensor, track_id, hand_gesture_output_desc->timestamp);

  return 0;
}

}  // namespace inference
