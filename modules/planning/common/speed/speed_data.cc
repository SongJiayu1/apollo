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

#include "modules/planning/common/speed/speed_data.h"

#include <algorithm>
#include <mutex>
#include <utility>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::SpeedPoint;

// 带参构造函数，输入参数是一个由多个 SpeedPoint 构成的 vector 容器对象 speed_points
// 冒号后面为类对象初始化方式，直接将参数 speed_points move 到 SpeedData 类对象容器中
SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
    : std::vector<SpeedPoint>(std::move(speed_points)) {
  // 然后构造函数对 SpeedData 这个容器里的对象进行排序
  // sort 函数的第 3 个参数代表排序的条件，即按照速度点的时间升序进行排序    
  std::sort(begin(), end(), [](const SpeedPoint& p1, const SpeedPoint& p2) {
      return p1.t() < p2.t();}
    );
}
// 在 SpeedData 类对象末尾插入一个速度点
// 用 s, t, v, a, da 来构造一个 SpeedPoint，然后加入 SpeedData 
void SpeedData::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a,
                                 const double da) {
  // 使用了 std::mutex，UNIQUE_LOCK_MULTITHREAD 线程锁
  static std::mutex mutex_speedpoint;
  UNIQUE_LOCK_MULTITHREAD(mutex_speedpoint);

  if (!empty()) {
    ACHECK(back().t() < time);
  }
  push_back(common::util::PointFactory::ToSpeedPoint(s, time, v, a, da));
}

// 通过 对应的时间 t， 查找速度信息（线性插值）
// Evaluate 通常指插值，用时间去速度规划数组 SpeedData 类对象里插值出 s, v, a, da
// 插值结果存放到函数输入的最后一个参数 speed_point 里
bool SpeedData::EvaluateByTime(const double t,
                               common::SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  // 这里要 t 在 front().t() 和 back().t() 之间
  // 注：这里的 1.0e-6 是为了消除浮点型数据运算的误差。
  if (!(front().t() < t + 1.0e-6 && t - 1.0e-6 < back().t())) {
    return false;
  }

  auto comp = [](const common::SpeedPoint& sp, const double t) {
    return sp.t() < t;
  };
  // 返回大于等于 t 的第一个 SpeedPoint（这里实际上是返回第一个不符合 comp 规则的元素）
  auto it_lower = std::lower_bound(begin(), end(), t, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1); // 
    const auto& p1 = *it_lower; // 第一个大于 t 的 SpeedPoint 的迭代器为 it_lower
    double t0 = p0.t();
    double t1 = p1.t();
    
    speed_point->Clear(); 
    speed_point->set_s(common::math::lerp(p0.s(), t0, p1.s(), t1, t)); // 线性插值
    // 注：这里的 lerp 是一个函数模板，可以自动类型推导（类模板必须显示的指定类型）。
    // 注：这里的 set_xxx 函数，是 protobuf 的默认访问方式。
    speed_point->set_t(t);
    if (p0.has_v() && p1.has_v()) {
      speed_point->set_v(common::math::lerp(p0.v(), t0, p1.v(), t1, t));
    }
    if (p0.has_a() && p1.has_a()) {
      speed_point->set_a(common::math::lerp(p0.a(), t0, p1.a(), t1, t));
    }
    if (p0.has_da() && p1.has_da()) {
      speed_point->set_da(common::math::lerp(p0.da(), t0, p1.da(), t1, t));
    }
  }
  return true;
}
// Evaluate通常指插值，用纵向位置去速度规划数组SpeedData类对象里插值出 t, v, a, da
// 插值结果存放到函数输入的最后一个参数 speed_point 里
bool SpeedData::EvaluateByS(const double s,
                            common::SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().s() < s + 1.0e-6 && s - 1.0e-6 < back().s())) {
    return false;
  }

  auto comp = [](const common::SpeedPoint& sp, const double s) {
    return sp.s() < s;
  };

  auto it_lower = std::lower_bound(begin(), end(), s, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double s0 = p0.s();
    double s1 = p1.s();

    speed_point->Clear();
    speed_point->set_s(s);
    speed_point->set_t(common::math::lerp(p0.t(), s0, p1.t(), s1, s));
    if (p0.has_v() && p1.has_v()) {
      speed_point->set_v(common::math::lerp(p0.v(), s0, p1.v(), s1, s));
    }
    if (p0.has_a() && p1.has_a()) {
      speed_point->set_a(common::math::lerp(p0.a(), s0, p1.a(), s1, s));
    }
    if (p0.has_da() && p1.has_da()) {
      speed_point->set_da(common::math::lerp(p0.da(), s0, p1.da(), s1, s));
    }
  }
  return true;
}
// 返回速度规划的总时间
// 就是 SpeedData 类对象里 速度点数组 最后一个点的时间减第一个点的时间。
double SpeedData::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().t() - front().t();
}
// 返回速度规划的纵向总位移（就是 delta s），
// 就是 SpeedData 类对象里速度点数组最后一个点的 s 值减第一个点的 s 值。
double SpeedData::TotalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}
// 返回 debug 字符串
// 其实就是获取速度点数组容器 SpeedData 中前 11 个点的信息构成字符串来 debug
std::string SpeedData::DebugString() const {
  const auto limit = std::min(
      size(), static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
  return absl::StrCat(
      "[\n",
      absl::StrJoin(begin(), begin() + limit, ",\n",
                    apollo::common::util::DebugStringFormatter()),
      "]\n");
}

}  // namespace planning
}  // namespace apollo
