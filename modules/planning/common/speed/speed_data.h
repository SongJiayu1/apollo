/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/common/proto/pnc_point.pb.h"

namespace apollo {
namespace planning {

// SpeedData 类是继承基类 SpeedPoint 速度点的 vector 类，本质上是一个 vector 容器类
// SpeedPoint 为 Protolbuf 数据类型，同时使用 std::sort() 按照时间 t 排序。
class SpeedData : public std::vector<common::SpeedPoint> {
 public:
  SpeedData() = default; // 默认构造函数

  virtual ~SpeedData() = default; // 默认析构函数
  // explicit 修饰的构造函数不能被隐式调用；禁止类对象之间的隐式转换
  explicit SpeedData(std::vector<common::SpeedPoint> speed_points);

  void AppendSpeedPoint(const double s, const double time, const double v,
                        const double a, const double da);

  bool EvaluateByTime(const double time,
                      common::SpeedPoint* const speed_point) const;

  // Assuming spatial traversed distance is monotonous, which is the case for
  // current usage on city driving scenario
  bool EvaluateByS(const double s, common::SpeedPoint* const speed_point) const;

  double TotalTime() const;

  // Assuming spatial traversed distance is monotonous
  double TotalLength() const;

  virtual std::string DebugString() const;
};

}  // namespace planning
}  // namespace apollo
