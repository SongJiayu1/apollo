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

#include "modules/planning/math/curve1d/curve1d.h"

namespace apollo {
namespace planning {

class ConstantJerkTrajectory1d : public Curve1d {
 public:
  // ConstantJerkTrajectory1d 类带参构造函数
  ConstantJerkTrajectory1d(const double p0, const double v0, const double a0,
                           const double jerk, const double param);
  // p0 -初始纵向位置
  // v0 - 初速度，a0 - 初始加速度
  // jerk - 加加速度
  // param - 匀加加速运动持续总时间

  virtual ~ConstantJerkTrajectory1d() = default;

  // 根据时间 param 即阶数 order，实现匀加加速过程中对位移
  // 速度，加速度及加加速度的插值
  double Evaluate(const std::uint32_t order, const double param) const;

  double ParamLength() const;

  std::string ToString() const;

  double start_position() const;

  double start_velocity() const;

  double start_acceleration() const;

  double end_position() const;

  double end_velocity() const;

  double end_acceleration() const;

  double jerk() const;

 private:
  double p0_;
  double v0_;
  double a0_;

  double p1_;
  double v1_;
  double a1_;

  double param_;

  double jerk_;
};

}  // namespace planning
}  // namespace apollo
