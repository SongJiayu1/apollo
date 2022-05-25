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

#pragma once

#include <string>
#include <vector>

#include "modules/planning/common/trajectory1d/constant_jerk_trajectory1d.h"
#include "modules/planning/math/curve1d/curve1d.h"

namespace apollo {
namespace planning {

class PiecewiseJerkTrajectory1d : public Curve1d {
 public:
  PiecewiseJerkTrajectory1d(const double p, const double v, const double a);
  // 参数 p 上一段匀加加速度运动的终点位置
  // 参数 v 上一段匀加加速度运动的终点速度
  // 参数 a 上一段匀加加速度运动的终点加速度

  virtual ~PiecewiseJerkTrajectory1d() = default;

  // 插值函数，Apollo 里见到 Evaluate 基本都是插值
  // order 表示插值的量的阶数，param 为参数时间
  double Evaluate(const std::uint32_t order, const double param) const;

  // 返回多段匀加加速度运动 1 维轨迹类所覆盖的时间的总长度
  double ParamLength() const;
   
  // 返回这条多段匀加加速度运动 1 维轨迹类的名称，默认为空
  std::string ToString() const;
  
  // 往 PiecewiseJerkTrajectory1d 类对象里增加一段 ConstantJerkTrajectory1d
  void AppendSegment(const double jerk, const double param);
  // 参数：保持不变的这个加加速度 jerk
  // 参数: param 代表这一段匀加加速度运动持续的时间

 private:
  std::vector<ConstantJerkTrajectory1d> segments_;

  double last_p_;

  double last_v_;

  double last_a_;
  // 储存每一段 ConstantJerkTrajectory1d 的终点时间，上一段的终点即这一段的起点时间
  std::vector<double> param_;
};

}  // namespace planning
}  // namespace apollo
