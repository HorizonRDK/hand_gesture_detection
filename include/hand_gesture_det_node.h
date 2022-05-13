// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <memory>
#include <string>
#include <vector>

#include "ai_msgs/msg/capture_targets.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "include/gesture_preprocess.h"
#include "rclcpp/rclcpp.hpp"

#ifndef HAND_GESTURE_DET_NODE_H_
#define HAND_GESTURE_DET_NODE_H_

namespace inference {

using rclcpp::NodeOptions;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;
using hobot::dnn_node::DNNResult;
using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::Model;
using hobot::dnn_node::ModelInferTask;
using hobot::dnn_node::ModelManager;
using hobot::dnn_node::ModelRoiInferTask;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::NV12PyramidInput;
using hobot::dnn_node::OutputDescription;
using hobot::dnn_node::OutputParser;
using hobot::dnn_node::TaskId;

using ai_msgs::msg::PerceptionTargets;

struct HandGestureRes {
  std::promise<int> prom_;
  std::atomic_bool is_promised_;
  std::vector<gesture_type> gesture_res_;
};

struct HandGestureOutput : public DnnNodeOutput {
  uint64_t track_id;
  uint64_t timestamp;
  std::shared_ptr<HandGestureRes> gesture_res = nullptr;
};

class HandGestureDetNode : public DnnNode {
 public:
  HandGestureDetNode(const std::string &node_name,
                     const NodeOptions &options = NodeOptions());
  ~HandGestureDetNode() override;

 protected:
  int SetNodePara() override;
  int SetOutputParser() override;

  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  std::string model_file_name_ = "config/gestureDet_8x21.hbm";
  std::string model_name_ = "gestureDet_8x21";
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;

  int model_input_width_ = -1;
  int model_input_height_ = -1;
  int32_t model_output_count_ = 1;
  const int32_t output_index_ = 0;

  int is_sync_mode_ = 0;

  std::string ai_msg_pub_topic_name = "/hobot_hand_gesture_detection";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ =
      nullptr;

  std::string gesture_preprocess_config_{""};
  std::shared_ptr<GesturePreProcess> gesture_preprocess_ = nullptr;

  // 模型结构信息, PreProcess需要
  std::vector<hbDNNTensorProperties> input_model_info_;

  std::chrono::high_resolution_clock::time_point output_tp_;
  int output_frameCount_ = 0;
  int smart_fps_ = -1;
  std::mutex frame_stat_mtx_;

  int Predict(std::vector<std::shared_ptr<DNNTensor>> &inputs,
              std::vector<std::shared_ptr<OutputDescription>> &output_descs,
              std::shared_ptr<DnnNodeOutput> dnn_output);

  std::string ai_msg_sub_topic_name_ = "/hobot_hand_lmk_detection";
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
      ai_msg_subscription_ = nullptr;
  void AiImgProcess(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

  int GetModelIOInfo();

  int Render(const std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> &pyramid,
             std::string result_image,
             std::vector<std::shared_ptr<hbDNNRoi>> rois,
             std::vector<std::shared_ptr<inference::Landmarks>> lmkses);
};
}  // namespace inference
#endif  // HAND_GESTURE_DET_NODE_H_
