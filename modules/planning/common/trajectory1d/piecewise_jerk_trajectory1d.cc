/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"

#include <algorithm>

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

PiecewiseJerkTrajectory1d::PiecewiseJerkTrajectory1d(const double p,
                                                     const double v,
                                                     const double a) {
  last_p_ = p;
  last_v_ = v;
  last_a_ = a;
  param_.push_back(0.0); // param_ 代表分段的时间轴，储存每一段匀加加速度运动
  //的终点时间，第一个元素是 0，然后
}

// 往 PiecewiseJerkTrajectory1d 类对象里增加一段匀加加速度运动的信息
// 参数：保持不变的这个加加速度 jerk
// 参数: param 代表这一段匀加加速度运动持续的时间
void PiecewiseJerkTrajectory1d::AppendSegment(const double jerk,
                                              const double param) {
  // 检查时间 param > 1e-6 ，否的话则报错
  // CHECK_GT 是 GLOG 库的里定义的函数
  CHECK_GT(param, FLAGS_numerical_epsilon);

  // 往时间轴 param_ 添加增加的这一段匀加加速运动段的终点时间
  param_.push_back(param_.back() + param);

  segments_.emplace_back(last_p_, last_v_, last_a_, jerk, param);
  // 注：segments_ 是一个 vector，里面每个元素都是 ConstantJerkTrajectory1d。
  //    用上一段匀加加速度运动的终点的 pos，v，a，以及这一段匀加加速度运动的 jerk，t 
  //    初始化一个 ConstantJerkTrajectory1d 对象加入容器。
  // 注：上一段的终点其实就是这一段的起点。
  
  // 更新类数成员 last_p_, last_v_, last_a_
  last_p_ = segments_.back().end_position();
  last_v_ = segments_.back().end_velocity();
  last_a_ = segments_.back().end_acceleration();
}

// 插值函数，order 为阶数，0 代表纵向位置插值，1 代表速度插值，2 代表加速度插值，
// 3 代表加加速度插值；param 代表时间，就是去分段匀加加速度轨迹上根据时间 t 
// 插值出对应的量
double PiecewiseJerkTrajectory1d::Evaluate(const std::uint32_t order,
                                           const double param) const {
  // 找到数据成员 param_ 里刚好小于给定时间 param 的那个元素的迭代器
  auto it_lower = std::lower_bound(param_.begin(), param_.end(), param);

  if (it_lower == param_.begin()) {
    return segments_[0].Evaluate(order, param);
  }

  if (it_lower == param_.end()) {
    auto index = std::max(0, static_cast<int>(param_.size() - 2));
    return segments_.back().Evaluate(order, param - param_[index]);
  }
  // 确定给定时间 param 在数据成员 vector param_ 里夹在哪个区间内，也就是夹在哪两个下标之
  // 间，这里的 index 代表所处区间的下边界
  auto index = std::distance(param_.begin(), it_lower);
  // 根据通过访问 ConstantJerkTrajectory1d 的成员函数插值出给
  // 定时间的对应的 pos，v，a，jerk 等
  return segments_[index - 1].Evaluate(order, param - param_[index - 1]);
}

// 返回 PiecewiseJerkTrajectory1d 所覆盖的时间的总长度
double PiecewiseJerkTrajectory1d::ParamLength() const { return param_.back(); }

std::string PiecewiseJerkTrajectory1d::ToString() const { return ""; }

}  // namespace planning
}  // namespace apollo
