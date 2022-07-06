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

#include <memory>
#include <vector>

#include "modules/common/math/box2d.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/discretized_trajectory.h"
#include "modules/planning/lattice/behavior/path_time_graph.h"

namespace apollo {
namespace planning {

class CollisionChecker {
 public:
  // 带参构造函数
  CollisionChecker(
      const std::vector<const Obstacle*>& obstacles, // 障碍物列表
      const double ego_vehicle_s, // 自车在 s 方向的位置
      const double ego_vehicle_d, // 自车的横向偏移（就是自车在 Frenet 坐标系下的坐标）
      const std::vector<common::PathPoint>& discretized_reference_line,
      const ReferenceLineInfo* ptr_reference_line_info,
      const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph);

  // Check if there are overlaps between obstacles and ego vehicle according
  // to the predicted obstacles environment.
  // 这个函数是根据 BuildPredictedEnvironment() 这个函数输出的预测环境信息进行检测。
  bool InCollision(const DiscretizedTrajectory& discretized_trajectory);

  // Check if there are overlaps between obstacles' predicted trajectory 
  // and ego vehicle's planning trajectory.
  static bool InCollision(const std::vector<const Obstacle*>& obstacles,
                          const DiscretizedTrajectory& ego_trajectory,
                          const double ego_length, const double ego_width,
                          const double ego_edge_to_center);

 private:
  // 建立预测环境函数：其实就是把待考虑障碍物的 0-8s 预测轨迹上的二维边界盒
  // 都塞入类成员 predicted_bounding_rectangles_ 了
  // 输入参数：障碍物列表，自车横纵向坐标，离散参考线
  void BuildPredictedEnvironment(
      const std::vector<const Obstacle*>& obstacles, 
      const double ego_vehicle_s,
      const double ego_vehicle_d,
      const std::vector<common::PathPoint>& discretized_reference_line);
  // 判断自车是否在参考线的车道上
  bool IsEgoVehicleInLane(const double ego_vehicle_s,
                          const double ego_vehicle_d);
  // 判断障碍物是否在自车后面的同一车道上
  bool IsObstacleBehindEgoVehicle(
      const Obstacle* obstacle, const double ego_vehicle_s,
      const std::vector<apollo::common::PathPoint>& discretized_reference_line);

 private:
  const ReferenceLineInfo* ptr_reference_line_info_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
  std::vector<std::vector<common::math::Box2d>> predicted_bounding_rectangles_;
};

}  // namespace planning
}  // namespace apollo
