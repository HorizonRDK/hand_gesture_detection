// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/hand_gesture_det_node.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "include/hand_gesture_output_parser.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"

namespace inference {
HandGestureDetNode::HandGestureDetNode(const std::string& node_name,
                                       const NodeOptions& options)
    : DnnNode(node_name, options) {
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<std::string>("model_file_name", model_file_name_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name",
                                       ai_msg_pub_topic_name);
  this->declare_parameter<std::string>("ai_msg_sub_topic_name",
                                       ai_msg_sub_topic_name_);

  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<std::string>("model_file_name", model_file_name_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name",
                                   ai_msg_pub_topic_name);
  this->get_parameter<std::string>("ai_msg_sub_topic_name",
                                   ai_msg_sub_topic_name_);

  std::stringstream ss;
  ss << "Parameter:"
     << "\n is_sync_mode_: " << is_sync_mode_
     << "\n model_file_name_: " << model_file_name_
     << "\n ai_msg_sub_topic_name_: " << ai_msg_sub_topic_name_
     << "\n ai_msg_pub_topic_name: " << ai_msg_pub_topic_name;
  RCLCPP_WARN(
      rclcpp::get_logger("hand gesture det node"), "%s", ss.str().c_str());

  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hand gesture det node"), "Init failed!");
  }

  if (GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("hand gesture det node"),
                 "Get model input size fail!");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("hand gesture det node"),
                "The model input width is %d and height is %d",
                model_input_width_,
                model_input_height_);
  }

  GetModelIOInfo();

  gesture_preprocess_ =
      std::make_shared<GesturePreProcess>(gesture_preprocess_config_);

  RCLCPP_WARN(rclcpp::get_logger("hand gesture det node"),
              "Create subscription with topic_name: %s",
              ai_msg_sub_topic_name_.c_str());
  ai_msg_subscription_ =
      this->create_subscription<ai_msgs::msg::PerceptionTargets>(
          ai_msg_sub_topic_name_,
          10,
          std::bind(
              &HandGestureDetNode::AiImgProcess, this, std::placeholders::_1));

  RCLCPP_WARN(rclcpp::get_logger("hand gesture det node"),
              "ai_msg_pub_topic_name: %s",
              ai_msg_pub_topic_name.data());
  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      ai_msg_pub_topic_name, 10);
}

HandGestureDetNode::~HandGestureDetNode() {}

int HandGestureDetNode::SetNodePara() {
  RCLCPP_INFO(rclcpp::get_logger("hand gesture det node"), "Set node para.");
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 2;
  return 0;
}

int HandGestureDetNode::SetOutputParser() {
  RCLCPP_INFO(rclcpp::get_logger("mono2d_body_det"), "Set output parser.");
  // set output parser
  auto model_manage = GetModel();
  if (!model_manage || !dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Invalid model");
    return -1;
  }

  std::shared_ptr<OutputParser> out_parser =
      std::make_shared<HandGestureOutputParser>();
  model_manage->SetOutputParser(output_index_, out_parser);

  return 0;
}

int HandGestureDetNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput>& node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }

  if (!msg_publisher_) {
    RCLCPP_ERROR(rclcpp::get_logger("hand gesture det node"),
                 "Invalid msg_publisher_");
    return -1;
  }

  if (!node_output ||
      output_index_ >= static_cast<int32_t>(node_output->outputs.size())) {
    RCLCPP_ERROR(rclcpp::get_logger("hand gesture det node"),
                 "Invalid node output");
    return -1;
  }

  auto hand_gesture_output =
      std::dynamic_pointer_cast<HandGestureOutput>(node_output);
  if (!hand_gesture_output) {
    return -1;
  }

  auto gesture_res_ptr = std::dynamic_pointer_cast<HandGestureResult>(
      hand_gesture_output->outputs.front());
  if (!gesture_res_ptr) {
    return -1;
  }
  const auto& res = gesture_res_ptr->gesture_res;
  if (res->value_ < static_cast<int>(gesture_type::Background) ||
      res->value_ > static_cast<int>(gesture_type::Awesome)) {
    hand_gesture_output->gesture_res->gesture_res_.push_back(
        gesture_type::Background);
  } else {
    hand_gesture_output->gesture_res->gesture_res_.push_back(
        static_cast<gesture_type>(res->value_));
  }

  hand_gesture_output->gesture_res->prom_.set_value(0);
  hand_gesture_output->gesture_res->is_promised_ = true;

  return 0;
}

int HandGestureDetNode::Predict(
    std::vector<std::shared_ptr<DNNTensor>>& inputs,
    std::vector<std::shared_ptr<OutputDescription>>& output_descs,
    std::shared_ptr<DnnNodeOutput> dnn_output) {
  RCLCPP_DEBUG(rclcpp::get_logger("hand gesture det node"),
               "task_num: %d",
               dnn_node_para_ptr_->task_num);

  return Run(
      inputs, output_descs, dnn_output, is_sync_mode_ == 1 ? true : false);
}

void HandGestureDetNode::AiImgProcess(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  if (!msg || !rclcpp::ok()) {
    return;
  }

  std::stringstream ss;
  ss << "Recved ai msg"
     << ", frame_id: " << msg->header.frame_id
     << ", stamp: " << msg->header.stamp.sec << "_"
     << msg->header.stamp.nanosec;
  RCLCPP_INFO(
      rclcpp::get_logger("hand gesture det node"), "%s", ss.str().c_str());

  auto model_manage = GetModel();
  if (!model_manage) {
    RCLCPP_ERROR(rclcpp::get_logger("mono2d_body_det"), "Invalid model");
    return;
  }

  std::vector<std::vector<DNNTensor>> input_tensors;
  std::vector<uint64_t> track_ids;

  ai_msgs::msg::PerceptionTargets::UniquePtr ai_msg(
      new ai_msgs::msg::PerceptionTargets());
  ai_msg->set__header(msg->header);
  ai_msg->set__fps(msg->fps);
  ai_msg->set__targets(msg->targets);
  ai_msg->set__disappeared_targets(msg->disappeared_targets);
  ai_msg->set__perfs(msg->perfs);
  uint64_t timestamp;
  if (!gesture_preprocess_ ||
      gesture_preprocess_->Execute(
          msg, input_model_info_, input_tensors, track_ids, timestamp) != 0 ||
      input_tensors.empty()) {
    msg_publisher_->publish(std::move(ai_msg));
    return;
  }

  std::unordered_map<uint64_t, std::shared_ptr<HandGestureRes>> gesture_outputs;
  for (size_t idx = 0; idx < input_tensors.size(); idx++) {
    uint64_t track_id = track_ids.at(idx);
    auto handgesture_output_desc = std::make_shared<HandGestureOutDesc>(
        model_manage, output_index_, "gesture_branch");
    handgesture_output_desc->timestamp = timestamp;
    handgesture_output_desc->track_id = track_id;
    handgesture_output_desc->input_tensor =
        std::make_shared<DNNTensor>(input_tensors.at(idx).front());

    std::vector<std::shared_ptr<OutputDescription>> output_descs{
        std::dynamic_pointer_cast<OutputDescription>(handgesture_output_desc)};

    auto gesture_output = std::make_shared<HandGestureRes>();
    gesture_outputs[track_id] = gesture_output;

    auto dnn_output = std::make_shared<HandGestureOutput>();
    dnn_output->gesture_res = gesture_output;
    dnn_output->gesture_res->is_promised_ = false;
    dnn_output->timestamp = timestamp;

    std::vector<std::shared_ptr<DNNTensor>> inputs{
        handgesture_output_desc->input_tensor};

    uint32_t ret = 0;
    // 3. 开始预测
    ret = Predict(inputs, output_descs, dnn_output);

    // 4. 处理预测结果，如渲染到图片或者发布预测结果
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("hand gesture det node"),
                   "Run predict failed!");
      return;
    }
  }

  // todo 20220428 move to thread pool
  // 等待所有input_tensor都推理完成
  // 一帧中可能包含多个target，即多个input_tensor，每个input_tensor对应一个PostProcess，PostProcess中设置推理完成标志
  // 所有input_tensor都推理完成后才会将此帧数据pub出去
  bool has_waited = false;
  for (auto& res : gesture_outputs) {
    std::shared_ptr<HandGestureRes>& gesture_info = res.second;
    if (!gesture_info) {
      continue;
    }
    if (!gesture_info->is_promised_) {
      if (has_waited) {
        // 已经等待过了，所有roi只需要等待一次
        continue;
      } else {
        has_waited = true;
        auto fut_ = gesture_info->prom_.get_future();
        int time_out_ms = 50;
        if (fut_.wait_for(std::chrono::milliseconds(time_out_ms)) ==
            std::future_status::ready) {
          gesture_info->is_promised_ = true;
        } else {
          gesture_info->is_promised_ = false;
          continue;
        }
      }
    } else {
      // do nothing
    }
  }

  if (msg_publisher_) {
    // append gesture to ai msg and publish ai msg
    ai_msgs::msg::PerceptionTargets::UniquePtr pub_ai_msg(
        new ai_msgs::msg::PerceptionTargets());
    pub_ai_msg->set__header(msg->header);
    pub_ai_msg->set__fps(msg->fps);
    pub_ai_msg->set__perfs(msg->perfs);
    pub_ai_msg->set__disappeared_targets(msg->disappeared_targets);

    std::stringstream ss;
    ss << "publish msg"
       << ", frame_id: " << pub_ai_msg->header.frame_id
       << ", stamp: " << pub_ai_msg->header.stamp.sec << "_"
       << msg->header.stamp.nanosec << "\n";
    for (const auto& in_target : msg->targets) {
      ai_msgs::msg::Target target;
      target.set__type(in_target.type);
      target.set__rois(in_target.rois);
      target.set__captures(in_target.captures);
      target.set__track_id(in_target.track_id);
      target.set__points(in_target.points);

      // 缓存target的attr
      std::vector<ai_msgs::msg::Attribute> tar_attributes;
      for (const auto& attr : in_target.attributes) {
        tar_attributes.push_back(attr);
      }

      if (gesture_outputs.find(in_target.track_id) != gesture_outputs.end()) {
        const auto& gesture_res = gesture_outputs.at(in_target.track_id);
        if (gesture_res && !gesture_res->gesture_res_.empty()) {
          for (const gesture_type& res : gesture_res->gesture_res_) {
            ai_msgs::msg::Attribute attr;
            attr.set__type("gesture");
            attr.set__value(static_cast<int>(res));
            tar_attributes.push_back(attr);

            ss << "\t target id: " << in_target.track_id
               << ", attr type: " << attr.type.data() << ", val: " << attr.value
               << "\n";
          }
        }
      }
      target.set__attributes(tar_attributes);
      pub_ai_msg->targets.emplace_back(target);
    }
    RCLCPP_WARN(
        rclcpp::get_logger("hand gesture det node"), "%s", ss.str().c_str());

    msg_publisher_->publish(std::move(pub_ai_msg));
  }
}

int HandGestureDetNode::GetModelIOInfo() {
  auto model_manage = GetModel();
  if (!model_manage) {
    RCLCPP_ERROR(rclcpp::get_logger("hand gesture det node"), "Invalid model");
    return -1;
  }

  hbDNNHandle_t dnn_model_handle = model_manage->GetDNNHandle();
  int input_num = model_manage->GetInputCount();
  input_model_info_.resize(input_num);
  for (int input_idx = 0; input_idx < input_num; input_idx++) {
    hbDNNGetInputTensorProperties(
        &input_model_info_[input_idx], dnn_model_handle, input_idx);

    std::stringstream ss;
    ss << "input_idx: " << input_idx
       << ", tensorType = " << input_model_info_[input_idx].tensorType
       << ", tensorLayout = " << input_model_info_[input_idx].tensorLayout;
    RCLCPP_WARN(
        rclcpp::get_logger("hand gesture det node"), "%s", ss.str().c_str());
  }

  return 0;
}

int HandGestureDetNode::Render(
    const std::shared_ptr<hobot::easy_dnn::NV12PyramidInput>& pyramid,
    std::string result_image,
    std::vector<std::shared_ptr<hbDNNRoi>> rois,
    std::vector<std::shared_ptr<inference::Landmarks>> lmkses) {
  static cv::Scalar colors[] = {
      cv::Scalar(255, 0, 0),    // red
      cv::Scalar(255, 255, 0),  // yellow
      cv::Scalar(0, 255, 0),    // green
      cv::Scalar(0, 0, 255),    // blue
  };

  char* y_img = reinterpret_cast<char*>(pyramid->y_vir_addr);
  char* uv_img = reinterpret_cast<char*>(pyramid->uv_vir_addr);
  auto height = pyramid->height;
  auto width = pyramid->width;
  auto img_y_size = height * width;
  auto img_uv_size = img_y_size / 2;
  char* buf = new char[img_y_size + img_uv_size];
  memcpy(buf, y_img, img_y_size);
  memcpy(buf + img_y_size, uv_img, img_uv_size);
  cv::Mat nv12(height * 3 / 2, width, CV_8UC1, buf);
  cv::Mat bgr;
  cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);
  auto& mat = bgr;

  RCLCPP_INFO(rclcpp::get_logger("hand lmk det node"),
              "h w: %d %d,  mat: %d %d",
              height,
              width,
              mat.cols,
              mat.rows);

  for (auto& roi : rois) {
    auto rect = *roi;
    cv::rectangle(mat,
                  cv::Point(rect.left, rect.top),
                  cv::Point(rect.right, rect.bottom),
                  cv::Scalar(255, 0, 0),
                  3);
  }

  size_t lmk_num = lmkses.size();
  for (size_t idx = 0; idx < lmk_num; idx++) {
    const auto& lmk = *lmkses.at(idx);
    auto& color = colors[idx % 4];
    for (const auto& point : lmk) {
      cv::circle(mat, cv::Point(point.x, point.y), 3, color, 3);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("hand lmk det node"),
              "Draw result to file: %s",
              result_image.c_str());
  cv::imwrite(result_image, mat);

  return 0;
}

}  // namespace inference