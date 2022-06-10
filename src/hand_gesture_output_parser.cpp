// Copyright (c) 2022，Horizon Robotics.
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
