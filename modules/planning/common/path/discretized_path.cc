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
 * @file path.cc
 **/

#include "modules/planning/common/path/discretized_path.h"

#include <algorithm>

#include "cyber/common/log.h"
#include "modules/common/math/linear_interpolation.h"

namespace apollo {
namespace planning {


// 使用路径点类，一种数据结构由.proto 文件生成的一种数据结构类
//详细参见 proto 文件定义，modules\common\proto\pnc_point.proto
using apollo::common::PathPoint;


// DiscretizedPath 类构造函数，参数就是路径点的 vector 容器，
// 其实就是将一系列路径点加载进来
DiscretizedPath::DiscretizedPath(std::vector<PathPoint> path_points)
    : std::vector<PathPoint>(std::move(path_points)) {}

// 获取离散点路径的总长度，如果离散点容器为空返回 0，
// 否则返回最后一个点的纵向位置 station 减去第一个点的纵向位置 station，
// 就是路径的总长度
double DiscretizedPath::Length() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

// 在离散的路径点之间正向线性插值得到给定 s 的点的信息，
// 输入：某个指定的纵向位置 path_s
// 输出：位置 path_s 处的 PathPoint
PathPoint DiscretizedPath::Evaluate(const double path_s) const {
  // 检查是否非空
  ACHECK(!empty());
  // 找到刚好总想位置小于 path_s 的点，也就是 path_s 的下边界点，返回指向它的迭代器
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin()) { // 如果下边界点就是起始点，返回起始点
    return front();
  }
  if (it_lower == end()) { // 如果下边界点就是最后一个点就返回最后一个点
    return back();
  }
  // 线性插值，用下边界点和下边界点的前一个点，线性插值得到 path_s 处的路径点信息
  // (x, y, theta, kappa 等)。
  return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1),
                                                           *it_lower, path_s);
}

// 查询下边界函数
// 输入参数是某个给定的纵向位置 path_s，找到路径点容器中刚好位置小于 path_s 的点
std::vector<PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
    const double path_s) const {
  auto func = [](const PathPoint &tp, const double path_s) {
    return tp.s() < path_s;
  };
  // 正向遍历路径点，返回大于等于 path_s 的第一个路径点的 const_iterator
  return std::lower_bound(begin(), end(), path_s, func);
}
// 在离散的路径点之间逆向线性插值得到给定 s 的点的信息，
// 输入参数也是某个指定的纵向位置 path_s
PathPoint DiscretizedPath::EvaluateReverse(const double path_s) const {
  ACHECK(!empty());
  auto it_upper = QueryUpperBound(path_s);
  if (it_upper == begin()) {
    return front();
  }
  if (it_upper == end()) {
    return back();
  }
  return common::math::InterpolateUsingLinearApproximation(*(it_upper - 1),
                                                           *it_upper, path_s);
}

// 查询上边界函数，输入参数是某个特定的纵向位置 path_s，
// 找到路径点容器中刚好大于 path_s 的点
std::vector<PathPoint>::const_iterator DiscretizedPath::QueryUpperBound(
    const double path_s) const {
  auto func = [](const double path_s, const PathPoint &tp) {
    return tp.s() < path_s;
  };
  return std::upper_bound(begin(), end(), path_s, func);
}

}  // namespace planning
}  // namespace apollo
