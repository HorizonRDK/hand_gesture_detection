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
