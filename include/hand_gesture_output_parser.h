// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef HAND_GESTURE_OUTPUT_PARSER_H
#define HAND_GESTURE_OUTPUT_PARSER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dnn_node/dnn_node_data.h"
#include "include/gesture_postprocess.h"

namespace inference {
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::InputDescription;
using hobot::dnn_node::Model;
using hobot::dnn_node::OutputDescription;
using hobot::dnn_node::SingleBranchOutputParser;

class HandGestureOutDesc : public OutputDescription {
 public:
  HandGestureOutDesc(Model* mode, int index, std::string type = "detection")
      : OutputDescription(mode, index, type) {}
  uint64_t track_id;
  uint64_t timestamp;
  std::shared_ptr<DNNTensor> input_tensor = nullptr;
};

class HandGestureResult : public DNNResult {
 public:
  std::shared_ptr<GestureRes> gesture_res = nullptr;
};

class HandGestureOutputParser : public SingleBranchOutputParser {
 public:
  HandGestureOutputParser() {
    gesture_post_process_ = std::make_shared<GesturePostProcess>("");
  }
  ~HandGestureOutputParser() {}

  int32_t Parse(
      std::shared_ptr<DNNResult>& output,
      std::vector<std::shared_ptr<InputDescription>>& input_descriptions,
      std::shared_ptr<OutputDescription>& output_description,
      std::shared_ptr<DNNTensor>& output_tensor) override;

 private:
  int i_o_stride_ = 4;

  std::shared_ptr<GesturePostProcess> gesture_post_process_ = nullptr;
};
}  // namespace inference
#endif  // HAND_GESTURE_OUTPUT_PARSER_H
