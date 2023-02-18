/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/st_bounds_decider/st_bounds_decider.h"

#include <limits>
#include <memory>

#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::planning_internal::StGraphBoundaryDebug;
using apollo::planning_internal::STGraphDebug;

namespace {
// STBoundPoint contains (t, s_min, s_max)
using STBoundPoint = std::tuple<double, double, double>;
// STBound is a vector of STBoundPoints
using STBound = std::vector<STBoundPoint>;
// ObsDecSet is a set of decision for new obstacles.
using ObsDecSet = std::vector<std::pair<std::string, ObjectDecisionType>>;
}  // namespace

STBoundsDecider::STBoundsDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  ACHECK(config.has_st_bounds_decider_config());
  st_bounds_config_ = config.st_bounds_decider_config();
}

Status STBoundsDecider::Process(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  // 1. Initialize the related helper classes.
  // 初始化相关的辅助类，主要是初始化 STObstaclesProcessor。
  InitSTBoundsDecider(*frame, reference_line_info);

  // 2. Sweep the t-axis, and determine the s-boundaries step by step.
  // 遍历单帧规划轨迹的时间轴 t（7.0s 总长度，0.1s 间隔），调用 GenerateRegularSTBound() 函数，
  // 一步一步确定每个离散时刻的 st，vt bound 和 st guide line。
  STBound regular_st_bound;
  STBound regular_vt_bound;
  std::vector<std::pair<double, double>> st_guide_line;
  Status ret = GenerateRegularSTBound(&regular_st_bound, &regular_vt_bound,
                                      &st_guide_line);
  if (!ret.ok()) {
    ADEBUG << "Cannot generate a regular ST-boundary.";
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }
  if (regular_st_bound.empty()) {
    const std::string msg = "Generated regular ST-boundary is empty.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // 3. 计算 st_drivable_boundaries。
  StGraphData* st_graph_data = reference_line_info_->mutable_st_graph_data();
  st_graph_data->SetSTDrivableBoundary(regular_st_bound, regular_vt_bound);

  // 4. Record the ST-Graph for good visualization and easy debugging.
  auto all_st_boundaries = st_obstacles_processor_.GetAllSTBoundaries();
  std::vector<STBoundary> st_boundaries;
  for (const auto& st_boundary : all_st_boundaries) {
    st_boundaries.push_back(st_boundary.second);
  }
  ADEBUG << "Total ST boundaries = " << st_boundaries.size();
  STGraphDebug* st_graph_debug = reference_line_info->mutable_debug()
                                     ->mutable_planning_data()
                                     ->add_st_graph();
  RecordSTGraphDebug(st_boundaries, regular_st_bound, st_guide_line,
                     st_graph_debug);

  return Status::OK();
}

void STBoundsDecider::InitSTBoundsDecider(
    const Frame& frame, ReferenceLineInfo* const reference_line_info) {
  const PathData& path_data = reference_line_info->path_data();
  PathDecision* path_decision = reference_line_info->path_decision();

  // 1. Map all related obstacles onto ST-Graph. 将所有障碍物投射到 ST 图上。
  auto time1 = std::chrono::system_clock::now();
  // 1.1 初始化 STObstaclesProcessor 类，把 ST 图的最大 Length，最长时间传进去。
  st_obstacles_processor_.Init(path_data.discretized_path().Length(),
                               st_bounds_config_.total_time(), path_data,
                               path_decision, injector_->history());
  // 1.2 将障碍物投影到 ST 图上。
  // 注：path_decision 里面包含了障碍物列表。
  st_obstacles_processor_.MapObstaclesToSTBoundaries(path_decision);
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Obstacles Processing = " << diff.count() * 1000
         << " msec.";

  // 2. Initialize Guide-Line and Driving-Limits.
  static constexpr double desired_speed = 15.0;
  // If the path_data optimization is guided from a reference path of a
  // reference trajectory, use its reference speed profile to select the st
  // bounds in LaneFollow Hybrid Mode
  // 2.1 初始化 st_guild_line_。
  if (path_data.is_optimized_towards_trajectory_reference()) { // 默认为 false。
    st_guide_line_.Init(desired_speed,
                        injector_->learning_based_data()
                            ->learning_data_adc_future_trajectory_points());
    // st_guide_line_.Init() 函数的第二个参数是 std::vector<common::TrajectoryPoint>，作为 speed_reference 输入。
    // st_guide_line_.Init() 函数给 guideline_speed_data_ 赋值，它会在 GetGuideSFromT 被使用。
  } else {
    st_guide_line_.Init(desired_speed);
  }
  // 2.2 初始化 st_driving_limits_。 
  static constexpr double max_acc = 2.5;
  static constexpr double max_dec = 5.0;
  static constexpr double max_v = desired_speed * 1.5;
  st_driving_limits_.Init(max_acc, max_dec, max_v,
                          frame.PlanningStartPoint().v());
  // 根据车辆最大加减速的限制计算出相应时间可以到达的 s。
  // 这部分会在 GenerateRegularSTBound 时候使用：
  // 它会调用 st_driving_limits_.GetVehicleDynamicsLimits() 函数，计算 s 的范围。
}

Status STBoundsDecider::GenerateFallbackSTBound(STBound* const st_bound,
                                                STBound* const vt_bound) {
  // Initialize st-boundary.
  for (double curr_t = 0.0; curr_t <= st_bounds_config_.total_time();
       curr_t += kSTBoundsDeciderResolution) {
    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
    vt_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
  }

  // Sweep-line to get detailed ST-boundary.
  for (size_t i = 0; i < st_bound->size(); ++i) {
    double t, s_lower, s_upper, lower_obs_v, upper_obs_v;
    std::tie(t, s_lower, s_upper) = st_bound->at(i);
    std::tie(t, lower_obs_v, upper_obs_v) = vt_bound->at(i);
    ADEBUG << "Processing st-boundary at t = " << t;

    // Get Boundary due to driving limits
    auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);
    s_lower = std::fmax(s_lower, driving_limits_bound.first);
    s_upper = std::fmin(s_upper, driving_limits_bound.second);
    ADEBUG << "Bounds for s due to driving limits are "
           << "s_upper = " << s_upper << ", s_lower = " << s_lower;

    // Get Boundary due to obstacles
    std::vector<std::pair<double, double>> available_s_bounds;
    std::vector<ObsDecSet> available_obs_decisions;
    if (!st_obstacles_processor_.GetSBoundsFromDecisions(
            t, &available_s_bounds, &available_obs_decisions)) {
      const std::string msg =
          "Failed to find a proper boundary due to obstacles.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    std::vector<std::pair<STBoundPoint, ObsDecSet>> available_choices;
    ADEBUG << "Available choices are:";
    for (int j = 0; j < static_cast<int>(available_s_bounds.size()); ++j) {
      ADEBUG << "  (" << available_s_bounds[j].first << ", "
             << available_s_bounds[j].second << ")";
      available_choices.emplace_back(
          std::make_tuple(0.0, available_s_bounds[j].first,
                          available_s_bounds[j].second),
          available_obs_decisions[j]);
    }
    RemoveInvalidDecisions(driving_limits_bound, &available_choices);

    // Always go for the most conservative option.
    if (!available_choices.empty()) {
      // Select the most conservative decision.
      auto top_choice_s_range = available_choices.front().first;
      auto top_choice_decision = available_choices.front().second;
      for (size_t j = 1; j < available_choices.size(); ++j) {
        if (std::get<1>(available_choices[j].first) <
            std::get<1>(top_choice_s_range)) {
          top_choice_s_range = available_choices[j].first;
          top_choice_decision = available_choices[j].second;
        }
      }

      // Set decision for obstacles without decisions.
      bool is_limited_by_upper_obs = false;
      bool is_limited_by_lower_obs = false;
      if (s_lower < std::get<1>(top_choice_s_range)) {
        s_lower = std::get<1>(top_choice_s_range);
        is_limited_by_lower_obs = true;
      }
      if (s_upper > std::get<2>(top_choice_s_range)) {
        s_upper = std::get<2>(top_choice_s_range);
        is_limited_by_upper_obs = true;
      }
      st_obstacles_processor_.SetObstacleDecision(top_choice_decision);

      // Update st-guide-line, st-driving-limit info, and v-limits.
      std::pair<double, double> limiting_speed_info;
      if (st_obstacles_processor_.GetLimitingSpeedInfo(t,
                                                       &limiting_speed_info)) {
        st_driving_limits_.UpdateBlockingInfo(
            t, s_lower, limiting_speed_info.first, s_upper,
            limiting_speed_info.second);
        st_guide_line_.UpdateBlockingInfo(t, s_lower, true);
        st_guide_line_.UpdateBlockingInfo(t, s_upper, false);
        if (is_limited_by_lower_obs) {
          lower_obs_v = limiting_speed_info.first;
        }
        if (is_limited_by_upper_obs) {
          upper_obs_v = limiting_speed_info.second;
        }
      }
    } else {
      const std::string msg = "No valid st-boundary exists.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // Update into st_bound
    st_bound->at(i) = std::make_tuple(t, s_lower, s_upper);
    vt_bound->at(i) = std::make_tuple(t, lower_obs_v, upper_obs_v);
  }

  return Status::OK();
}

Status STBoundsDecider::GenerateRegularSTBound(
    STBound* const st_bound, STBound* const vt_bound,
    std::vector<std::pair<double, double>>* const st_guide_line) {
  // 1. Initialize st-boundary.
  // 遍历整个 st_bounds_config_ 的 total_time，在 planning_config.pb.txt 中被定义为 7.0s，
  // 离散采样，每隔 0.1s 采样一次。
  for (double curr_t = 0.0; curr_t <= st_bounds_config_.total_time();
       curr_t += kSTBoundsDeciderResolution) { 
    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
    vt_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
    // 此时 st_bound 里应该塞了 71 个点（共 7.0s，0.1s 一个点）。
  }

  // 2. Sweep-line to get detailed ST-boundary.
  // 沿着时间轴 t 遍历每个离散采样点：
  for (size_t i = 0; i < st_bound->size(); ++i) {
    // 通过 std::tie 取出 tuple 中的数。
    double t, s_lower, s_upper, lower_obs_v, upper_obs_v;
    std::tie(t, s_lower, s_upper) = st_bound->at(i);
    std::tie(t, lower_obs_v, upper_obs_v) = vt_bound->at(i);
    ADEBUG << "Processing st-boundary at t = " << t;

    // 2.1 Get Boundary due to driving limits
    // 根据车辆最大的加减速度及最大速度限制，计算车辆可以到达的 s 的上下边界。
    // 注：在 InitSTBoundsDecider() 中已经初始化了 st_driving_limits_，即确定了车辆的最大加速度，
    // 最大减速度以及最大速度（max_v = 1.5 * desired_v）。
    auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);
    s_lower = std::fmax(s_lower, driving_limits_bound.first);
    s_upper = std::fmin(s_upper, driving_limits_bound.second);
    ADEBUG << "Bounds for s due to driving limits are "
           << "s_upper = " << s_upper << ", s_lower = " << s_lower;

    // 2.2 Get Boundary due to obstacles
    // 2.2.1 初始化 available_s_bounds。
    // 注：在 available_s_bounds 中存储当前时刻所有可能的 s 上下界
    //（可能夹在多个障碍物的 ST 边界之间，因此有多种可能）。
    std::vector<std::pair<double, double>> available_s_bounds;
    // 2.2.2 初始化 available_obs_decisions。
    // 在 available_obs_decisions 是一个存储障碍物列表及其决策的 vector 数组，
    // 里面的每一个元素都是可能的决策列表，用来存储 t 时刻对所有障碍物的决策。
    // 如果取不同障碍物之间得区域可能有不同得决策，因此有多套决策列表和上面的 
    // available_s_bounds 对应，一个 bound 对应着一个决策列表。
    std::vector<ObsDecSet> available_obs_decisions;
    // 2.2.3 之前 init 初始化过的 st_obstacle_processor_，障碍物的 ST 图投影即 ST 边界全部得到了，
    // 调用 GetSBoundsFromDecisions()，得到自车所有可能 s 边界 available_s_bounds 
    // 及其对应得决策列表 available_obs_decisions。
    if (!st_obstacles_processor_.GetSBoundsFromDecisions(
            t, &available_s_bounds, &available_obs_decisions)) {
      const std::string msg =
          "Failed to find a proper boundary due to obstacles.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    // 2.2.4 最后把上下界和决策都塞进 available_choices 中。
    std::vector<std::pair<STBoundPoint, ObsDecSet>> available_choices;
    ADEBUG << "Available choices are:";
    for (int j = 0; j < static_cast<int>(available_s_bounds.size()); ++j) {
      ADEBUG << "  (" << available_s_bounds[j].first << ", "
             << available_s_bounds[j].second << ")";
      available_choices.emplace_back(
          std::make_tuple(0.0, available_s_bounds[j].first,
                          available_s_bounds[j].second), // t=0.0, s_min, s_max
          available_obs_decisions[j]);
          // std::vector.emplace_back() 可以直接塞 std::pair：
          // 第一个是 STBoundPoint = std::tuple<double, double, double>，第二个是 ObsDecSet。
    }
    // 2.3 根据自车运动学约束，剔除不可达的障碍物 decision。
    RemoveInvalidDecisions(driving_limits_bound, &available_choices);

    if (!available_choices.empty()) {
      ADEBUG << "One decision needs to be made among "
             << available_choices.size() << " choices.";
      // 2.4 计算 guide_line_s。
      double guide_line_s = st_guide_line_.GetGuideSFromT(t);
      st_guide_line->emplace_back(t, guide_line_s);
      // 2.5 对 available_choices 中的 s bound 按照从大到小（或者包含引导线）进行排序。
      RankDecisions(guide_line_s, driving_limits_bound, &available_choices);
      // 2.6 Select the top decision.
      auto top_choice_s_range = available_choices.front().first;
      bool is_limited_by_upper_obs = false;
      bool is_limited_by_lower_obs = false;
      // 2.6.1 如果自车可行驶的 s_lower 小于 top choice 的 s_min，说明被障碍物限制了 s_lower，
      // 用障碍物生成的 s bound 的下界更新 s_lower。
      if (s_lower < std::get<1>(top_choice_s_range)) {
        s_lower = std::get<1>(top_choice_s_range);
        is_limited_by_lower_obs = true;
      }
      // 2.6.2 如果自车可行驶的 s_upper 大于 top choice 的 s_max，说明被障碍物限制了 s_upper，
      // 用障碍物生成的 s bound 的上界更新 s_upper。
      if (s_upper > std::get<2>(top_choice_s_range)) {
        s_upper = std::get<2>(top_choice_s_range);
        is_limited_by_upper_obs = true;
      }

      // 2.7 Set decision for obstacles without decisions.
      // 把最优的决策赋给全部的障碍物。
      auto top_choice_decision = available_choices.front().second;
      st_obstacles_processor_.SetObstacleDecision(top_choice_decision);

      // 2.8 Update st-guide-line, st-driving-limit info, and v-limits.
      std::pair<double, double> limiting_speed_info;
      // 2.8.1 根据障碍物的 yield，stop，overtake 决策算出自车的在每个时刻的速度限制上下限。
      // 得到的自车速度限制放在 limiting_speed_info 里。
      if (st_obstacles_processor_.GetLimitingSpeedInfo(t,
                                                       &limiting_speed_info)) {
        // 2.8.2 把当前的 s 的上下边界，速度的上下边界及对应的时间更新到 st_driving_limits 的类成员里。
        st_driving_limits_.UpdateBlockingInfo(
            t, s_lower, limiting_speed_info.first, s_upper,
            limiting_speed_info.second);
        // 2.8.3 更新 t 时刻的引导线，其实就是用求出的 s 上下界去限制t时刻引导线 s 不能超出这个范围。
        st_guide_line_.UpdateBlockingInfo(t, s_lower, true);
        st_guide_line_.UpdateBlockingInfo(t, s_upper, false);
        if (is_limited_by_lower_obs) {
          lower_obs_v = limiting_speed_info.first;
        }
        if (is_limited_by_upper_obs) {
          upper_obs_v = limiting_speed_info.second;
        }
      }
    } else {
      const std::string msg = "No valid st-boundary exists.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // Update into st_bound
    st_bound->at(i) = std::make_tuple(t, s_lower, s_upper);
    vt_bound->at(i) = std::make_tuple(t, lower_obs_v, upper_obs_v);
  }

  return Status::OK();
}

void STBoundsDecider::RemoveInvalidDecisions(
    std::pair<double, double> driving_limit,
    std::vector<std::pair<STBoundPoint, ObsDecSet>>* available_choices) {
  // Remove those choices that don't even fall within driving-limits.
  size_t i = 0;
  while (i < available_choices->size()) {
    double s_lower = 0.0;
    double s_upper = 0.0;
    std::tie(std::ignore, s_lower, s_upper) = available_choices->at(i).first;
    if (s_lower > driving_limit.second || s_upper < driving_limit.first) {
      // 注：driving_limit.second 是 ADV 能够到达的 s_max；first 是 ADV 能够到达的 s_min。
      // 也就是说，如果可通行区域的 s_lower 比自车可到达的 s_max 还要大，则说明自车无法到达，舍弃；
      // 同理，如果可通行区域的 s_upper 比自车可到达的 s_min 还要小，则说明自车无法到达，舍弃。

      // Invalid bound, should be removed.
      // 如果是无效的 bound，则将其置于最后，然后 pop_back() 舍弃。
      if (i != available_choices->size() - 1) {
        swap(available_choices->at(i),
             available_choices->at(available_choices->size() - 1));
      }
      available_choices->pop_back();
    } else {
      // Valid bound, proceed to the next one.
      ++i;
    }
  }
}

void STBoundsDecider::RankDecisions(
    double s_guide_line, std::pair<double, double> driving_limit,
    std::vector<std::pair<STBoundPoint, ObsDecSet>>* available_choices) {
  // Perform sorting of the existing decisions.
  bool has_swaps = true;
  while (has_swaps) {
    has_swaps = false;
    for (int i = 0; i < static_cast<int>(available_choices->size()) - 1; ++i) {
      // 1. 拿到第 i 个 bound 的 s 上下界。
      double A_s_lower = 0.0;
      double A_s_upper = 0.0;
      std::tie(std::ignore, A_s_lower, A_s_upper) =
          available_choices->at(i).first;
      // 2. 拿到第 i+1 个 bound 的 s 上下界。
      double B_s_lower = 0.0;
      double B_s_upper = 0.0;
      std::tie(std::ignore, B_s_lower, B_s_upper) =
          available_choices->at(i + 1).first;

      ADEBUG << "    Range ranking: A has s_upper = " << A_s_upper
             << ", s_lower = " << A_s_lower;
      ADEBUG << "    Range ranking: B has s_upper = " << B_s_upper
             << ", s_lower = " << B_s_lower;

      // 3. If not both are larger than passable-threshold, should select
      // the one with larger room.
      double A_room = std::fmin(driving_limit.second, A_s_upper) -
                      std::fmax(driving_limit.first, A_s_lower);
      double B_room = std::fmin(driving_limit.second, B_s_upper) -
                      std::fmax(driving_limit.first, B_s_lower);
      // 如果两个可通行区域至少有一个小于 kSTPassableThreshold = 3m，则把较大的放在前面。
      if (A_room < kSTPassableThreshold || B_room < kSTPassableThreshold) {
        if (A_room < B_room) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true; 
          // 如果发生了交换，说明相邻的两个元素不满足从大到小的顺序，因此在 for 循环后，
          // 需要再次执行一次 while 循环，直到整个 for 循环过程中不发生任何交换，说明从大到小排序完成。
          ADEBUG << "Swapping to favor larger room.";
        }
        continue;
      }

      // 4. Should select the one with overlap to guide-line.
      // 如果两个可通行区域都大于 3m，则把包含引导线的前置，优先级更高。
      bool A_contains_guideline =
          A_s_upper >= s_guide_line && A_s_lower <= s_guide_line;
      bool B_contains_guideline =
          B_s_upper >= s_guide_line && B_s_lower <= s_guide_line;
      if (A_contains_guideline != B_contains_guideline) {
        if (!A_contains_guideline) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true;
          ADEBUG << "Swapping to favor overlapping with guide-line.";
        }
        continue;
      }
    }
  }
}

void STBoundsDecider::RecordSTGraphDebug(
    const std::vector<STBoundary>& st_graph_data, const STBound& st_bound,
    const std::vector<std::pair<double, double>>& st_guide_line,
    planning_internal::STGraphDebug* const st_graph_debug) {
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }

  // Plot ST-obstacle boundaries.
  for (const auto& boundary : st_graph_data) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary.id());
    if (boundary.boundary_type() == STBoundary::BoundaryType::YIELD) {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = YIELD";
    } else if (boundary.boundary_type() == STBoundary::BoundaryType::OVERTAKE) {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = OVERTAKE";
    } else {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = UNKNOWN";
    }

    for (const auto& point : boundary.points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  // Plot the chosen ST boundary.
  auto boundary_debug = st_graph_debug->add_boundary();
  boundary_debug->set_name("Generated ST-Boundary");
  boundary_debug->set_type(
      StGraphBoundaryDebug::ST_BOUNDARY_TYPE_DRIVABLE_REGION);
  for (const auto& st_bound_pt : st_bound) {
    auto point_debug = boundary_debug->add_point();
    double t = 0.0;
    double s_lower = 0.0;
    std::tie(t, s_lower, std::ignore) = st_bound_pt;
    point_debug->set_t(t);
    point_debug->set_s(s_lower);
    ADEBUG << "(" << t << ", " << s_lower << ")";
  }
  for (int i = static_cast<int>(st_bound.size()) - 1; i >= 0; --i) {
    auto point_debug = boundary_debug->add_point();
    double t = 0.0;
    double s_upper = 0.0;
    std::tie(t, std::ignore, s_upper) = st_bound[i];
    point_debug->set_t(t);
    point_debug->set_s(s_upper);
    ADEBUG << "(" << t << ", " << s_upper << ")";
  }

  // Plot the used st_guide_line when generating the st_bounds
  for (const auto& st_points : st_guide_line) {
    auto* speed_point = st_graph_debug->add_speed_profile();
    speed_point->set_t(st_points.first);
    speed_point->set_s(st_points.second);
  }
}

}  // namespace planning
}  // namespace apollo
