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

/*
 * @file
 */

#include "modules/planning/open_space/coarse_trajectory_generator/hybrid_a_star.h"

#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;

HybridAStar::HybridAStar(const PlannerOpenSpaceConfig& open_space_conf) {
  planner_open_space_config_.CopyFrom(open_space_conf);
  reed_shepp_generator_ =
      std::make_unique<ReedShepp>(vehicle_param_, planner_open_space_config_);
  grid_a_star_heuristic_generator_ =
      std::make_unique<GridSearch>(planner_open_space_config_);
  next_node_num_ =
      planner_open_space_config_.warm_start_config().next_node_num(); // next_node_num = 10
  max_steer_angle_ =
      vehicle_param_.max_steer_angle() / vehicle_param_.steer_ratio(); // max_steer_angle: 8.20 弧度，约等于 30° 角度
  step_size_ = planner_open_space_config_.warm_start_config().step_size(); // step_size_ = 0.5
  xy_grid_resolution_ =
      planner_open_space_config_.warm_start_config().xy_grid_resolution(); // xy_grid_resolution_ = 0.3
  delta_t_ = planner_open_space_config_.delta_t();
  traj_forward_penalty_ =
      planner_open_space_config_.warm_start_config().traj_forward_penalty();
  traj_back_penalty_ =
      planner_open_space_config_.warm_start_config().traj_back_penalty();
  traj_gear_switch_penalty_ =
      planner_open_space_config_.warm_start_config().traj_gear_switch_penalty();
  traj_steer_penalty_ =
      planner_open_space_config_.warm_start_config().traj_steer_penalty();
  traj_steer_change_penalty_ = planner_open_space_config_.warm_start_config()
                                   .traj_steer_change_penalty();
}

bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
  // 调用 ShortestRSP 函数，得到弧长最短的 RS 曲线，返回 reeds_shepp_to_check 指针
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          reeds_shepp_to_check)) {
    ADEBUG << "ShortestRSP failed";
    return false;
  }
  // RS 曲线的碰撞与越界检测
  if (!RSPCheck(reeds_shepp_to_check)) {
    return false;
  }

  ADEBUG << "Reach the end configuration with Reed Sharp";
  // load the whole RSP as nodes and add to the close set
  // 将 RS 曲线的最后一个节点赋给 final_node_，并且把 current_node 作为 final_node 的父节点。
  final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
  return true;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
  std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));
  return ValidityCheck(node);
}

bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node) {
  CHECK_NOTNULL(node);
  CHECK_GT(node->GetStepSize(), 0);

  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }

  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      return false;
    }
    Box2d bounding_box = Node3d::GetBoundingBox(
        vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i]);
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const common::math::LineSegment2d& linesegment :
           obstacle_linesegments) {
        if (bounding_box.HasOverlap(linesegment)) {
          ADEBUG << "collision start at x: " << linesegment.start().x();
          ADEBUG << "collision start at y: " << linesegment.start().y();
          ADEBUG << "collision end at x: " << linesegment.end().x();
          ADEBUG << "collision end at y: " << linesegment.end().y();
          return false;
        }
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {

  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_));

  end_node->SetPre(current_node);
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}

std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index) {
  // 这里主要是从 current_node 扩展到 next_node，是一步一步扩展的：
  // 新的 next_node 有自己的 traversed_x_， y， phi，里面保存的第一个元素是 current_node 的 x，y，phi，
  // 最后一个元素就是 next_node 自己的坐标信息，中间保存了每一步扩展的中间节点的坐标信息。
  double steering = 0.0;
  double traveled_distance = 0.0;

  // 1. 计算 steering 和 traveled_distance 用于符合运动学约束的扩展
  if (next_node_index < static_cast<double>(next_node_num_) / 2) { // 前进
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(next_node_index); // 这里的 max_steer_angle_ = 0.51，约等于 30 度。
    traveled_distance = step_size_; // step_size_ = 0.5, Grid 的分辨率为 0.3 米
  } else { // 后退
    size_t index = next_node_index - next_node_num_ / 2;
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(index);
    traveled_distance = -step_size_;
  } // steering 可能取的值为：-30，-15，0，15，30

  // take above motion primitive to generate a curve driving the car to a
  // different grid - 确保了新的节点与当前节点不在同一个栅格中，避免了共占栅格问题
  double arc = std::sqrt(2) * xy_grid_resolution_; // arc = 0.424 
  // 2 定义数组用来存放 current_node 到 next_node 之间，用来构成圆弧的中间点（包括这两个节点本身）
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  // 拿到当前节点的坐标信息
  double last_x = current_node->GetX(); 
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();

  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);

  for (size_t i = 0; i < arc / step_size_; ++i) { // arc = 0.424, step_size_ = 0.5, 实际上扩展了一次，就到达下一个栅格了。
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    // 这里使用了以后轴为参考点的自行车模型
    const double next_phi = common::math::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base() * std::tan(steering));
        // traveled_distance = 0.5 m, wheel_base = 2.8
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }
  // check if the vehicle runs outside of XY boundary
  if (intermediate_x.back() > XYbounds_[1] ||
      intermediate_x.back() < XYbounds_[0] ||
      intermediate_y.back() > XYbounds_[3] ||
      intermediate_y.back() < XYbounds_[2]) {
    return nullptr;
  }
  // 根据上面扩展的信息，构造 next_node 并返回
  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 planner_open_space_config_)); // intermediate_x 就是 traversed_x_
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  return next_node;
}

void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node)); 
  // TrajCost 函数计算了从当前节点到下一个节点状态转换的 cost
  // evaluate heuristic cost
  double optimal_path_cost = 0.0;
  optimal_path_cost += HoloObstacleHeuristic(next_node);
  // 这里只用了 dp_map 作为启发 cost，而没有考虑 RS 曲线的弧长
  next_node->SetHeuCost(optimal_path_cost);
}

double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_forward_penalty_;
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_back_penalty_;
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node) {
  return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                      next_node->GetY());
}

bool HybridAStar::GetResult(HybridAStartResult* result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  // hybrid_a_x，y，phi 用来存放全部的节点（起始节点，终止节点，全部的中间扩展节点）
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;

  // 通过 while 循环，从目标节点回溯到起始节点
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs(); // get traversed_x_
    std::vector<double> y = current_node->GetYs(); // get traversed_y_
    std::vector<double> phi = current_node->GetPhis(); // get traversed_phi_
    if (x.empty() || y.empty() || phi.empty()) {
      AERROR << "result size check failed";
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      AERROR << "states sizes are not equal";
      return false;
    }
    // 注意：这里的 x，y，就是 traversed_x，y，里面的第一个元素是上一个节点的坐标，最后一个元素是当前节点的坐标。
    // 这里先 reverse 再 pop_back，就是将上一个节点的坐标 pop 出去了，保留了中间节点和当前节点的信息。
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  } // while 循环之后得到的是从终止节点到起始节点（包含全部的中间扩展节点，不包含起始节点本身）的坐标信息。
  // 注：当循环到最后一个节点，即 current_node 此时已经是起始节点，它的 preNode = nullptr，退出循环。
  //   此时，hybrid_a_x 中的序列是：目标节点 -> 起始节点的前一个节点。

  // 将起始节点压入 hybrid_a_x
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());

  // 将 hybrid_a_x, y, phi 全部 reverse，得到从起始节点到终止节点的 vector
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;
  
  // 将 Hybrid A* 计算的轨迹结果，按照行驶的正反方向切换，分割为数段，分别逆向翻转轨迹点
  // 然后重新拼接在一起，进行速度规划，就是最终可以发布供车行驶的轨迹
  if (!GetTemporalProfile(result)) { // 对 result 进行时序排序，这里传递的是指针
    AERROR << "GetSpeedProfile from Hybrid Astar path fails";
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->v.size() ||
      result->x.size() != result->phi.size()) {
    AERROR << "state sizes not equal, "
           << "result->x.size(): " << result->x.size() << "result->y.size()"
           << result->y.size() << "result->phi.size()" << result->phi.size()
           << "result->v.size()" << result->v.size();
    return false;
  }
  if (result->a.size() != result->steer.size() ||
      result->x.size() - result->a.size() != 1) {
    AERROR << "control sizes not equal or not right";
    AERROR << " acceleration size: " << result->a.size();
    AERROR << " steer size: " << result->steer.size();
    AERROR << " x size: " << result->x.size();
    return false;
  }
  return true;
}

bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult* result) {
  // Sanity Check
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR << "result size check when generating speed and acceleration fail";
    return false;
  }
  const size_t x_size = result->x.size();

  // load velocity from position
  // initial and end speed are set to be zeros
  result->v.push_back(0.0);
  for (size_t i = 1; i + 1 < x_size; ++i) {
    double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) *
                             std::cos(result->phi[i]) +
                         ((result->x[i] - result->x[i - 1]) / delta_t_) *
                             std::cos(result->phi[i])) /
                            2.0 +
                        (((result->y[i + 1] - result->y[i]) / delta_t_) *
                             std::sin(result->phi[i]) +
                         ((result->y[i] - result->y[i - 1]) / delta_t_) *
                             std::sin(result->phi[i])) /
                            2.0;
    result->v.push_back(discrete_v);
  }
  result->v.push_back(0.0);

  // load acceleration from velocity
  for (size_t i = 0; i + 1 < x_size; ++i) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }

  // load steering from phi
  for (size_t i = 0; i + 1 < x_size; ++i) {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            vehicle_param_.wheel_base() / step_size_;
    if (result->v[i] > 0.0) {
      discrete_steer = std::atan(discrete_steer);
    } else {
      discrete_steer = std::atan(-discrete_steer);
    }
    result->steer.push_back(discrete_steer);
  }
  return true;
}

bool HybridAStar::GenerateSCurveSpeedAcceleration(HybridAStartResult* result) {
  /*
    使用 QP 的方法，进行速度规划，求 v，a，steer
    这里的 result 指针指向的是分段后的某一段轨迹的首地址
  */
  // sanity check
  CHECK_NOTNULL(result);
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    AERROR << "result size check when generating speed and acceleration fail";
    return false;
  }
  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    AERROR << "result sizes not equal";
    return false;
  }

  // get gear info - 计算起始点的档位状态
  double init_heading = result->phi.front();
  const Vec2d init_tracking_vector(result->x[1] - result->x[0],
                                   result->y[1] - result->y[0]);
  const double gear =
      std::abs(common::math::NormalizeAngle(
          init_heading - init_tracking_vector.Angle())) < M_PI_2; // 起始点的档位

  // get path lengh
  size_t path_points_size = result->x.size();

  double accumulated_s = 0.0; // 用于存放累加的位移
  result->accumulated_s.clear();

  auto last_x = result->x.front();
  auto last_y = result->y.front();

  for (size_t i = 0; i < path_points_size; ++i) {
    double x_diff = result->x[i] - last_x;
    double y_diff = result->y[i] - last_y;
    accumulated_s += std::sqrt(x_diff * x_diff + y_diff * y_diff); // 计算每个节点处的位移
    result->accumulated_s.push_back(accumulated_s);
    last_x = result->x[i];
    last_y = result->y[i];
  }
  // assume static initial state
  const double init_v = 0.0;
  const double init_a = 0.0;

  // minimum time speed optimization
  // TODO(Jinyun): move to confs - 确定约束
  const double max_forward_v = 2.0;
  const double max_reverse_v = 1.0;
  const double max_forward_acc = 2.0;
  const double max_reverse_acc = 1.0;
  const double max_acc_jerk = 0.5;
  const double delta_t = 0.2;

  SpeedData speed_data;

  // TODO(Jinyun): explore better time horizon heuristic - 根据这段轨迹的总位移，计算所需要的总时间
  const double path_length = result->accumulated_s.back();
  const double total_t = std::max(gear ? 1.5 *
                                             (max_forward_v * max_forward_v +
                                              path_length * max_forward_acc) /
                                             (max_forward_acc * max_forward_v)
                                       : 1.5 *
                                             (max_reverse_v * max_reverse_v +
                                              path_length * max_reverse_acc) /
                                             (max_reverse_acc * max_reverse_v),
                                  10.0); // 计算总时间，上限设定为 10 s

  const size_t num_of_knots = static_cast<size_t>(total_t / delta_t) + 1;

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(
      num_of_knots, delta_t, {0.0, std::abs(init_v), std::abs(init_a)});

  // 初始化 x_bounds, dx_bounds, ddx_bounds
  std::vector<std::pair<double, double>> x_bounds(num_of_knots,
                                                  {0.0, path_length});

  const double max_v = gear ? max_forward_v : max_reverse_v;
  const double max_acc = gear ? max_forward_acc : max_reverse_acc;

  const auto upper_dx = std::fmax(max_v, std::abs(init_v));
  std::vector<std::pair<double, double>> dx_bounds(num_of_knots,
                                                   {0.0, upper_dx});
  std::vector<std::pair<double, double>> ddx_bounds(num_of_knots,
                                                    {-max_acc, max_acc});
  // 初始化末状态
  x_bounds[num_of_knots - 1] = std::make_pair(path_length, path_length);
  dx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);
  ddx_bounds[num_of_knots - 1] = std::make_pair(0.0, 0.0);

  // TODO(Jinyun): move to confs
  std::vector<double> x_ref(num_of_knots, path_length);
  piecewise_jerk_problem.set_x_ref(10000.0, std::move(x_ref));
  piecewise_jerk_problem.set_weight_ddx(10.0);
  piecewise_jerk_problem.set_weight_dddx(10.0);
  piecewise_jerk_problem.set_x_bounds(std::move(x_bounds));
  piecewise_jerk_problem.set_dx_bounds(std::move(dx_bounds));
  piecewise_jerk_problem.set_ddx_bounds(std::move(ddx_bounds));
  piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);

  // solve the problem
  if (!piecewise_jerk_problem.Optimize()) {
    AERROR << "Piecewise jerk speed optimizer failed!";
    return false;
  }

  // extract output - 输出平滑后的位置，速度，加速度
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

  // assign speed point by gear
  speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  const double kEpislon = 1.0e-6;
  const double sEpislon = 1.0e-6;
  for (size_t i = 1; i < num_of_knots; ++i) {
    if (s[i - 1] - s[i] > kEpislon) {
      ADEBUG << "unexpected decreasing s in speed smoothing at time "
             << static_cast<double>(i) * delta_t << "with total time "
             << total_t;
      break;
    }
    speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i), ds[i],
                                dds[i], (dds[i] - dds[i - 1]) / delta_t);
    // cut the speed data when it is about to meet end condition
    if (path_length - s[i] < sEpislon) {
      break;
    }
  }

  // 构造 path_data
  DiscretizedPath path_data;
  for (size_t i = 0; i < path_points_size; ++i) {
    common::PathPoint path_point;
    path_point.set_x(result->x[i]);
    path_point.set_y(result->y[i]);
    path_point.set_theta(result->phi[i]);
    path_point.set_s(result->accumulated_s[i]);
    path_data.push_back(std::move(path_point));
  }

  HybridAStartResult combined_result;

  // TODO(Jinyun): move to confs
  const double kDenseTimeResoltuion = 0.5;
  const double time_horizon =
      speed_data.TotalTime() + kDenseTimeResoltuion * 1.0e-6;
  if (path_data.empty()) {
    AERROR << "path data is empty";
    return false;
  }
  
  for (double cur_rel_time = 0.0; cur_rel_time < time_horizon;
       cur_rel_time += kDenseTimeResoltuion) {
    common::SpeedPoint speed_point;
    if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data.Length()) {
      break;
    }

    common::PathPoint path_point = path_data.Evaluate(speed_point.s());
    // combine speed and path profile
    combined_result.x.push_back(path_point.x());
    combined_result.y.push_back(path_point.y());
    combined_result.phi.push_back(path_point.theta());
    combined_result.accumulated_s.push_back(path_point.s());
    if (!gear) {
      combined_result.v.push_back(-speed_point.v());
      combined_result.a.push_back(-speed_point.a());
    } else {
      combined_result.v.push_back(speed_point.v());
      combined_result.a.push_back(speed_point.a());
    }
  }

  combined_result.a.pop_back();

  // recalc step size
  path_points_size = combined_result.x.size();

  // load steering from phi
  for (size_t i = 0; i + 1 < path_points_size; ++i) {
    double discrete_steer =
        (combined_result.phi[i + 1] - combined_result.phi[i]) *
        vehicle_param_.wheel_base() /
        (combined_result.accumulated_s[i + 1] -
         combined_result.accumulated_s[i]);
    discrete_steer =
        gear ? std::atan(discrete_steer) : std::atan(-discrete_steer);
    combined_result.steer.push_back(discrete_steer);
  }

  *result = combined_result;
  return true;
}

// 轨迹切割 -> 速度规划 -> Piesewies jerk speed 规划
// 轨迹切割是按照挡位变化（前进、后退）进行切割
bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult& result, // const 左值引用 - 可以接收一个左值，也可以接受一个右值
    std::vector<HybridAStartResult>* partitioned_result) { // 这里传递的是一个左值，可以被赋值
  // 输出：partitioned_result

  // 此时 result 已经是 起始节点 -> 目标节点 的信息
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.size() != y.size() || x.size() != phi.size()) {
    AERROR << "states sizes are not equal when do trajectory partitioning of "
              "Hybrid A Star result";
    return false;
  }

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back(); // 调用 HybridAStartResult 的默认构造函数

  auto* current_traj = &(partitioned_result->back());
  // 根据档位变化进行轨迹切割
  // 档位是通过判断某个节点处自车航向角和 这个节点和下一个节点构成的 tracking_vector 的角度关系来确定的
  double heading_angle = phi.front(); // 起始节点的 heading_angle
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]); // 计算起始节点处的 tracking_angle
  double tracking_angle = init_tracking_vector.Angle();
  // 判断起始节点处的档位：current_gear = true - 前进；false，后退
  bool current_gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
      (M_PI_2);
    
  // for 循环计算每个节点的 heading_angle 和 trakcing_angle，来判断档位；
  // 如果和起始档位不同，说明行驶方向发生变化，需要进行轨迹切割。
  for (size_t i = 0; i < horizon - 1; ++i) { // ++i 是左值
    heading_angle = phi[i];
    const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.Angle();
    bool gear =
        std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
        (M_PI_2);
    
    // 如果在某个节点处的档位与起始节点处的档位不一致：
    // 这里的处理需要注意：假设所有节点为 A(forward)->B(forward)->C(backward)->D(backward)
    // 由 B 到 C 发生了档位变化，需要将 B 加入当前轨迹中（作为当前轨迹的最后一个元素），
    // 然后 partitioned_result->emplace_back() 会新加入一个类型为 HybridAStarResult 的元素（里面都是空的）
    // 作为新的轨迹段，让 current_traj 指针重新指向新的轨迹段的首地址，
    // 同时，也需要将 B 节点加入新的轨迹段，作为新的轨迹段的第一个元素，之后继续循环，将后续节点加入这条轨迹段中。

    if (gear != current_gear) {
      // 这里相当于把 B 节点加入当前的轨迹段中，作为当前轨迹段的最后一个元素
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);

      // 在容器末尾加入一个 HybridAStarResult 类型的元素，作为新的轨迹段
      partitioned_result->emplace_back();

      // 让 current_traj 指针指向新的轨迹段的首地址
      current_traj = &(partitioned_result->back());
      current_gear = gear; // 更新档位状态
    }
    // 这里对应把 B 节点加入新的轨迹段，作为新轨迹段的第一个节点
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
  }
  // 最后，把目标节点的信息存入当前的轨迹段中
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());
  
  // 在上面对轨迹进行分段后，需要对每段轨迹进行速度规划，即对轨迹点的 a，v，steer 进行赋值
  // 这里采用了两种方法：
  //  1. 数值优化，使用 QP 进行速度规划
  //  2. 使用相邻点的静态信息进行规划
  const auto start_timestamp = std::chrono::system_clock::now();

  // Retrieve v, a and steer from path
  for (auto& result : *partitioned_result) {
    if (FLAGS_use_s_curve_speed_smooth) {
      // 对分割后的每段轨迹进行速度规划
      if (!GenerateSCurveSpeedAcceleration(&result)) { // Piecewise jerk 速度规划
        AERROR << "GenerateSCurveSpeedAcceleration fail";
        return false;
      }
    } else { // 根据 result 中的静态信息 x，y，phi，利用相邻点，逐个点求动态信息 v，a，steer
      if (!GenerateSpeedAcceleration(&result)) { 
        AERROR << "GenerateSpeedAcceleration fail";
        return false;
      }
    }
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  ADEBUG << "speed profile total time: " << diff.count() * 1000.0 << " ms.";
  return true;
}

bool HybridAStar::GetTemporalProfile(HybridAStartResult* result) {
  std::vector<HybridAStartResult> partitioned_results;
  // 轨迹分割 - 按照行驶方向对轨迹进行分割，给每段轨迹添加 v，a，steer 信息（速度规划）
  if (!TrajectoryPartition(*result, &partitioned_results)) {
    AERROR << "TrajectoryPartition fail";
    return false;
  }

  // 将分段的轨迹拼接起来
  HybridAStartResult stitched_result;
  for (const auto& result : partitioned_results) {
    std::copy(result.x.begin(), result.x.end() - 1,
              std::back_inserter(stitched_result.x));
    std::copy(result.y.begin(), result.y.end() - 1,
              std::back_inserter(stitched_result.y));
    std::copy(result.phi.begin(), result.phi.end() - 1,
              std::back_inserter(stitched_result.phi));
    std::copy(result.v.begin(), result.v.end() - 1,
              std::back_inserter(stitched_result.v));
    std::copy(result.a.begin(), result.a.end(),
              std::back_inserter(stitched_result.a));
    std::copy(result.steer.begin(), result.steer.end(),
              std::back_inserter(stitched_result.steer));
  }
  stitched_result.x.push_back(partitioned_results.back().x.back());
  stitched_result.y.push_back(partitioned_results.back().y.back());
  stitched_result.phi.push_back(partitioned_results.back().phi.back());
  stitched_result.v.push_back(partitioned_results.back().v.back());
  *result = stitched_result; // 输出，此时 result 指向的轨迹已经完成了轨迹分割，速度规划
  return true;
}
// HybridAStar 的核心规划函数
bool HybridAStar::Plan(
    double sx, double sy, double sphi, double ex, double ey, double ephi,
    const std::vector<double>& XYbounds,
    const std::vector<std::vector<common::math::Vec2d>>& obstacles_vertices_vec, 
    HybridAStartResult* result) {
  // 1 数据准备，清空之前的缓存数据 - 1.1 clear containers 
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)(); // 注意这里的用法，用 decltype 清空一种容器
  final_node_ = nullptr;
  // 1.2 初始化存放障碍物边界的 vector
  std::vector<std::vector<common::math::LineSegment2d>>
      obstacles_linesegments_vec;
  // 根据障碍物顶点信息，构造障碍物的轮廓线段
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    std::vector<common::math::LineSegment2d> obstacle_linesegments; // 存放每个障碍物的边界线段信息，一个障碍物有多条边界线段
    for (size_t i = 0; i < vertices_num - 1; ++i) { // 假设一个障碍物有 4 个顶点，则构造的线段为 0-1，1-2，2-3
      common::math::LineSegment2d line_segment = common::math::LineSegment2d(
          obstacle_vertices[i], obstacle_vertices[i + 1]); // 构造每条线段
      obstacle_linesegments.emplace_back(line_segment);
    }
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec); // 构造结束后，存入类的成员变量

  // load XYbounds
  XYbounds_ = XYbounds;
  // 1.3 构造起始节点和目标节点 - 注意：这里使用智能指针的 reset 函数
  // load nodes and obstacles
  start_node_.reset(
      new Node3d({sx}, {sy}, {sphi}, XYbounds_, planner_open_space_config_));
  end_node_.reset(
      new Node3d({ex}, {ey}, {ephi}, XYbounds_, planner_open_space_config_));
  if (!ValidityCheck(start_node_)) {
    ADEBUG << "start_node in collision with obstacles";
    return false;
  }
  if (!ValidityCheck(end_node_)) {
    ADEBUG << "end_node in collision with obstacles";
    return false;
  }
  double map_time = Clock::NowInSeconds();
  // 1.5 生成 dp_map_
  // 使用动态规划 DP 来计算目标点到某点的启发代价（以目标点为 DP 的起点），即 heuristic cost 中的 holonomic with obstacle 部分。
  // 生成 graph 的同时获得了目标点到图中任一点的 cost，作为缓存，这就是 DPMap 的用处
  grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_,
                                                  obstacles_linesegments_vec_);
  ADEBUG << "map time " << Clock::NowInSeconds() - map_time;
  // 1.6 将 start_node_ 压入 open_set_ 和 priority queue - load open set, pq
  // 注：这里用 open_set_ 和 open_pq 共同实现了 open list 的作用；
  //  open_set_ 用来存放 index 和 node 的指针 - 由于是哈希表结构，可以快速查找；
  //  open_pq_ 用来对 node 按照 cost 进行排序，可以快速排序
  // 这种设计方式，实际上是用空间换时间的思想
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());
  // 注：这里 open_pq_ 里面的每个元素都是 std::pair<std::string, double>

  size_t explored_node_num = 0;
  double astar_start_time = Clock::NowInSeconds();
  double heuristic_time = 0.0;
  double rs_time = 0.0;

  // 2 HybridAStar 主循环
  while (!open_pq_.empty()) {
    // 2.1 take out the lowest cost neighboring node
    const std::string current_id = open_pq_.top().first; 
    // 这里处理的思路：用优先队列查找 cost 最小的节点的 id，然后去 open_set_ 里面找对应的节点的指针
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];
    // 2.2 check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so, search
    // ends. - 尝试直接用 RS 曲线连接当前节点到目标节点，如果无碰撞，返回 RS 曲线，结束循环
    const double rs_start_time = Clock::NowInSeconds();
    if (AnalyticExpansion(current_node)) {
      break; // 主循环结束的条件
    }
    const double rs_end_time = Clock::NowInSeconds();
    rs_time += rs_end_time - rs_start_time;

    // 2.3 将当前节点加入 close_set_
    close_set_.emplace(current_node->GetIndex(), current_node);
    // 2.4 由当前节点扩展下一个节点
    // 对前轮转角进行采样，向前 5 个点，向后 5 个点
    for (size_t i = 0; i < next_node_num_; ++i) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      // collision check
      if (!ValidityCheck(next_node)) {
        continue;
      }
      // 如果 next_node_ 不在 open_set_ 中，则计算它的 cost，并将其添加到 open_set_ 中
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        explored_node_num++;
        const double start_time = Clock::NowInSeconds();
        CalculateNodeCost(current_node, next_node);
        const double end_time = Clock::NowInSeconds();
        heuristic_time += end_time - start_time;
        open_set_.emplace(next_node->GetIndex(), next_node);
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
      }
    }
  } // HybridAStar 的主循环会一直循环，直到 AnalyticExpansion 函数击中到目标点，此时 final_node_ 指针会被赋值。
  if (final_node_ == nullptr) {
    ADEBUG << "Hybrid A searching return null ptr(open_set ran out)";
    return false;
  }
  // 初步路径的每个node都指向上一个node，HybridAStar::GetResult() 在把这些 node 反向后
  //（由当前node指向终点node），得到了顺序正确的node集合。
  // 注意，此时形成了一个挨一个的node，还不是一个挨一个的轨迹点。
  // 因此，要调用 GetTemporalProfile(result) 来完成大部分的后处理，得到最终结果。
  if (!GetResult(result)) { // 这里的 result 是 HybridAStar::Plan 函数的输出，是一个类型为 HybridAStartResult 的指针
    ADEBUG << "GetResult failed";
    return false;
  }
  ADEBUG << "explored node num is " << explored_node_num;
  ADEBUG << "heuristic time is " << heuristic_time;
  ADEBUG << "reed shepp time is " << rs_time;
  ADEBUG << "hybrid astar total time is "
         << Clock::NowInSeconds() - astar_start_time;
  return true;
}
}  // namespace planning
}  // namespace apollo
