// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef ACT_POSTPROCESS_UTIL_H_
#define ACT_POSTPROCESS_UTIL_H_

#include <deque>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace inference {

class CachedScoreEntry {
 public:
  explicit CachedScoreEntry(float timestamp) : timestamp_(timestamp) {}
  void Clean() { score_.clear(); }
  std::vector<float> score_;
  float timestamp_;
};

class GesturePostProcessUtil {
 public:
  GesturePostProcessUtil() { inited_ = false; }

  static GesturePostProcessUtil *GetInstance();

  int Init(float window_size, int score_size);

  // timestamp单位秒
  std::vector<float> GetCachedAvgScore(float timestamp,
                                       int track_id,
                                       std::vector<float> cur_score);

  void Clean(std::vector<int64_t> disappeared_track_ids);

 private:
  std::vector<float> CalcAvg(std::deque<CachedScoreEntry> data);

 private:
  std::unordered_map<int, std::deque<CachedScoreEntry>> cached_scores_map_;
  // 单位秒，表示window时间内的数据
  float window_size_;
  int score_size_;
  bool inited_ = false;
  std::mutex map_mutex_;
  static GesturePostProcessUtil *util_instance_;
  static std::mutex instance_mutex_;
};

}  // namespace inference

#endif  // ACT_POSTPROCESS_UTIL_H_
