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

#include "modules/planning/math/piecewise_jerk/piecewise_jerk_path_problem.h"

#include "cyber/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

PiecewiseJerkPathProblem::PiecewiseJerkPathProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init)
    : PiecewiseJerkProblem(num_of_knots, delta_s, x_init) {}

// 构建 P 矩阵
void PiecewiseJerkPathProblem::CalculateKernel(std::vector<c_float>* P_data,
                                               std::vector<c_int>* P_indices,
                                               std::vector<c_int>* P_indptr) {
  const int n = static_cast<int>(num_of_knots_); // 待平滑的路径点的个数
  const int num_of_variables = 3 * n; // 每个点处要优化 x，x'，x'' 三个变量 - 就是 l 方向的 l，l'，l''
  const int num_of_nonzeros = num_of_variables + (n - 1);
  // 这里定义的 columns，里面存放的就是 P 矩阵的信息
  // 假设需要优化 4 和路径点，即 n = 4,，则 number_of_variables = 12，相当于 columns 有 12 行，
  // 每一行是一个 vector，有 12 个元素，每个元素是一个 pair，存放 index 和 value
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables);
  int value_index = 0;
  
  // x(i)^2 * (w_x + w_x_ref[i]), w_x_ref might be a uniform value for all x(i)
  // or piecewise values for different x(i)
  // P 矩阵的零阶导数部分
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(i, (weight_x_ + weight_x_ref_vec_[i]) /
                                   (scale_factor_[0] * scale_factor_[0])); // scale_factor - 缩放系数
    ++value_index;
  }
  // x(n-1)^2 * (w_x + w_x_ref[n-1] + w_end_x) - 最后一个节点增加了一个末状态的权重 w_end_x
  columns[n - 1].emplace_back(
      n - 1, (weight_x_ + weight_x_ref_vec_[n - 1] + weight_end_state_[0]) /
                 (scale_factor_[0] * scale_factor_[0]));
  ++value_index;

  // x(i)'^2 * w_dx - P 矩阵的一阶导数部分
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(
        n + i, weight_dx_ / (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  }
  // x(n-1)'^2 * (w_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(2 * n - 1,
                                  (weight_dx_ + weight_end_state_[1]) /
                                      (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

  auto delta_s_square = delta_s_ * delta_s_;
  // P 矩阵的二阶导数部分的第一个对角线元素
  // x(i)''^2 * (w_ddx + w_dddx / delta_s^2) 
  columns[2 * n].emplace_back(2 * n,
                              (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                  (scale_factor_[2] * scale_factor_[2]));
  ++value_index;
  // P 矩阵的二阶导数部分的对角线元素（除了对角线上的第一个元素和最后一个元素）
  // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2)
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
                       (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }
  // P 矩阵的二阶导数部分的最后一个对角线元素
   // x(i)''^2 * (w_ddx + w_dddx / delta_s^2 + w_end_ddx)
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
          (scale_factor_[2] * scale_factor_[2]));
  ++value_index;
  // P 矩阵的二阶导数部分的次对角线上的元素
  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  for (int i = 0; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(2 * n + i + 1,
                                    (-2.0 * weight_dddx_ / delta_s_square) /
                                        (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  CHECK_EQ(value_index, num_of_nonzeros);

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}
// 构建 q 矩阵
/*******************************************************************************
 * 
 * 目标函数中的 (l-l_ref)^2 = l^2 - 2*l_ref*l + l_ref^2
 * l_ref^2 是个非负实数，对梯度没贡献可以忽略，q 矩阵中就只剩下 -2*l_ref*l
 * 
 * *****************************************************************************/
void PiecewiseJerkPathProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  q->resize(kNumParam, 0.0);

  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * weight_x_ref_vec_.at(i) * x_ref_[i] / scale_factor_[0];
    }
  }
  // 
  if (has_end_state_ref_) {
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }
}

}  // namespace planning
}  // namespace apollo
