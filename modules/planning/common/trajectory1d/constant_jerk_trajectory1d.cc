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

#include "modules/planning/common/trajectory1d/constant_jerk_trajectory1d.h"

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

ConstantJerkTrajectory1d::ConstantJerkTrajectory1d(const double p0,
                                                   const double v0,
                                                   const double a0,
                                                   const double j,
                                                   const double param)
    : p0_(p0), v0_(v0), a0_(a0), param_(param), jerk_(j) {
  // CHECK_GT 判断 param > FLAGS_numerical_epsilon?
  // FLAGS_numerical_epsilon 去 modules\planning\common\planning_gflags.cc中
  // 取出 numerical_epsilon的值 1e-6，否则报错
  CHECK_GT(param, FLAGS_numerical_epsilon);

  // 计算匀加加速运动的终点的相对纵向位置
  p1_ = Evaluate(0, param_); // 根据时间零阶插值即为位移
  // 计算匀加加速运动的终点的速度
  v1_ = Evaluate(1, param_); // 根据时间 1 阶插值即为速度
  // 计算匀加加速运动的终点的加速度
  a1_ = Evaluate(2, param_); // 根据时间 2 阶插值即为加速度
}

// 根据时间 param 及阶数 order，实现匀加加速过程中对位移
// 速度，加速度及加加速度的插值
double ConstantJerkTrajectory1d::Evaluate(const std::uint32_t order,
                                          const double param) const {
  switch (order) {
    case 0: {
      return p0_ + v0_ * param + 0.5 * a0_ * param * param +
             jerk_ * param * param * param / 6.0;
    }
    case 1: {
      return v0_ + a0_ * param + 0.5 * jerk_ * param * param;
    }
    case 2: {
      return a0_ + jerk_ * param;
    }
    case 3: {
      return jerk_;
    }
    default:
      return 0.0;
  }
}
// 下面都是一些返回数据成员的函数，包括匀加加速度的时间，位移，速度，加速度，加加速度等
double ConstantJerkTrajectory1d::start_position() const { return p0_; }

double ConstantJerkTrajectory1d::start_velocity() const { return v0_; }

double ConstantJerkTrajectory1d::start_acceleration() const { return a0_; }

double ConstantJerkTrajectory1d::end_position() const { return p1_; }

double ConstantJerkTrajectory1d::end_velocity() const { return v1_; }

double ConstantJerkTrajectory1d::end_acceleration() const { return a1_; }

double ConstantJerkTrajectory1d::ParamLength() const { return param_; }

std::string ConstantJerkTrajectory1d::ToString() const { return ""; }

double ConstantJerkTrajectory1d::jerk() const { return jerk_; }

}  // namespace planning
}  // namespace apollo
