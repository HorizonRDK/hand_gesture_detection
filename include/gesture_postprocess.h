// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef GESTURE_POSTPROCESS_H_
#define GESTURE_POSTPROCESS_H_

#include <memory>
#include <string>
#include <vector>

#include "include/gesture_data.h"
#include "include/gesture_postprocess_util.h"

namespace inference {

class GesturePostProcess {
 public:
  explicit GesturePostProcess(std::string json_str) { Init(json_str); }

  ~GesturePostProcess() {}

  int Init(const std::string &json_str);

  std::shared_ptr<GestureRes> Execute(
      const std::shared_ptr<DNNTensor> &output_tensor,
      const uint64_t &track_id,
      const uint64_t &timestamp);

 private:
  void InitThresholds(std::string &thresholds_str);

  void InitMergeGroups(std::string &merge_groups_str);

  GestureRes ActPostPro(std::vector<FloatTensor> &float_tensors,
                        uint64_t timestamp,
                        uint64_t track_id);

  void OutputTensors2FloatTensors(const DNNTensor &tensor,
                                  FloatTensor &float_tensor,
                                  int batch);

 private:
  int en_score_avg_ = 0;
  float threshold_ = 0.5;  // 0.95;
  std::vector<float> thresholds_ = {0.95};
  std::vector<std::vector<int>> merge_groups_ = {
      {0, 1, 2, 3, 6, 12, 13, 14, 15, 16},
      {},
      {4},
      {5},
      {7},
      {8, 9, 10},
      {},
      {},
      {},
      {},
      {},
      {17},
      {18},
      {19},
      {20},
      {11},
      {21},
      {22}};

  float window_size_ = 0.5;
};

}  // namespace inference

#endif  // GESTURE_POSTPROCESS_H_
