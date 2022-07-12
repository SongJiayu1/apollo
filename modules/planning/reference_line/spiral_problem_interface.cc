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

/*
 * spiral_problem_interface.cc
 */

#include "modules/planning/reference_line/spiral_problem_interface.h"

#include <utility>

#include "modules/common/math/math_utils.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {
auto THETA0 =
    QuinticSpiralPathWithDerivation<SpiralProblemInterface::N>::THETA0;
auto KAPPA0 =
    QuinticSpiralPathWithDerivation<SpiralProblemInterface::N>::KAPPA0;
auto DKAPPA0 =
    QuinticSpiralPathWithDerivation<SpiralProblemInterface::N>::DKAPPA0;
auto THETA1 =
    QuinticSpiralPathWithDerivation<SpiralProblemInterface::N>::THETA1;
auto KAPPA1 =
    QuinticSpiralPathWithDerivation<SpiralProblemInterface::N>::KAPPA1;
auto DKAPPA1 =
    QuinticSpiralPathWithDerivation<SpiralProblemInterface::N>::DKAPPA1;
auto DELTA_S =
    QuinticSpiralPathWithDerivation<SpiralProblemInterface::N>::DELTA_S;
}  // namespace

SpiralProblemInterface::SpiralProblemInterface(
    std::vector<Eigen::Vector2d> points)
    : init_points_(std::move(points)) {
  num_of_points_ = static_cast<int>(init_points_.size());
  CHECK_GT(num_of_points_, 1);
  // point_distances_ 里面存放的就是每两个 anchor point 之间的 deltaS。
  point_distances_.reserve(num_of_points_ - 1);
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    point_distances_.push_back((init_points_[i + 1] - init_points_[i]).norm());
    // 这里是 Eigen::Vector2d.norm() 返回向量的二范数
  }

  std::vector<double> normalized_theta;  
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    Eigen::Vector2d v = init_points_[i + 1] - init_points_[i];
    double theta = std::atan2(v.y(), v.x());
    normalized_theta.push_back(theta); // 计算每个点处的 theta
  }
  // 由于最后一个 anchor point 的 heading 无法计算，
  // 所以把倒数第二个点的 heading 当做最后一个点的 heading 了。
  normalized_theta.push_back(normalized_theta.back());

  // 把 normalized_theta 中的第一个元素压入 relative_theta，之后以它为参考 heading。
  relative_theta_.push_back(normalized_theta.front());
  for (int i = 1; i < num_of_points_; ++i) {
    double theta_diff =
        common::math::AngleDiff(relative_theta_.back(), normalized_theta[i]);
    relative_theta_.push_back(relative_theta_.back() + theta_diff);
  }

  piecewise_paths_.resize(num_of_points_ - 1);
}

void SpiralProblemInterface::get_optimization_results(
    std::vector<double>* ptr_theta, std::vector<double>* ptr_kappa,
    std::vector<double>* ptr_dkappa, std::vector<double>* ptr_s,
    std::vector<double>* ptr_x, std::vector<double>* ptr_y) const {
  *ptr_theta = opt_theta_;
  *ptr_kappa = opt_kappa_;
  *ptr_dkappa = opt_dkappa_;
  *ptr_s = opt_s_;
  *ptr_x = opt_x_;
  *ptr_y = opt_y_;
}
// 定义问题规模
bool SpiralProblemInterface::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                                          int& nnz_h_lag,
                                          IndexStyleEnum& index_style) {
  // number of variables
  // n 定义了变量个数，由于每个点包含了 5 个优化变量和 1 个两点之间弧长的变量，所以：
  n = num_of_points_ * 5 + num_of_points_ - 1; // delta_s 只有 num_of_points_ -1 个
  num_of_variables_ = n;

  // number of constraints
  // m 定义了约束的数量，除起始点外，每个点包含 x，y 的等式约束和每个点的位置平移约束
  // b. positional equality constraints; - 位置的等式约束
  // totally 2 * (num_of_points - 1) considering x and y separately
  m = (num_of_points_ - 1) * 2;  // x，y 的约束，不包括第一个点
  // a. positional movements; totally num_of_points - 位置平移约束
  m += num_of_points_;  // 位置平移约束
  num_of_constraints_ = m;

  // number of nonzero constraint jacobian. - 这部分是如何计算出来的？？？
  nnz_jac_g = (num_of_points_ - 1) * 2 * 9 + num_of_points_ * 2;

  // number of nonzero hessian and lagrangian.
  nnz_h_lag = 0; // 由于这里使用了 ipopt 的拟牛顿法，
  // 内部直接求了 Hessian 矩阵，因此这里赋值为 0 了。

  index_style = IndexStyleEnum::C_STYLE;
  return true;
}
// 定义优化变量的上下边界和约束的上下边界
bool SpiralProblemInterface::get_bounds_info(int n, double* x_l, double* x_u,
                                             int m, double* g_l, double* g_u) {
  // x_l，x_u 定义了优化变量本身的取值范围
  // g_l，g_u 定义了约束的下边界和上边界
  // m 为约束的数量，n 为变量的数量
  CHECK_EQ(n, num_of_variables_);
  CHECK_EQ(m, num_of_constraints_);

  // variables - 定义优化变量本身的取值范围
  // a. for theta, kappa, dkappa, x, y 
  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 5;

    double theta_lower = 0.0;
    double theta_upper = 0.0;
    double kappa_lower = 0.0;
    double kappa_upper = 0.0;
    double dkappa_lower = 0.0;
    double dkappa_upper = 0.0;
    double x_lower = 0.0;
    double x_upper = 0.0;
    double y_lower = 0.0;
    double y_upper = 0.0;
    if (i == 0 && has_fixed_start_point_) {
      // 对于第一个点优化变量约束为等式约束，所以其上下边界均为其本身
      theta_lower = start_theta_;
      theta_upper = start_theta_;
      kappa_lower = start_kappa_;
      kappa_upper = start_kappa_;
      dkappa_lower = start_dkappa_;
      dkappa_upper = start_dkappa_;
      x_lower = start_x_;
      x_upper = start_x_;
      y_lower = start_y_;
      y_upper = start_y_;

    } else if (i + 1 == num_of_points_ && has_fixed_end_point_) {
      // 最后一个点的 x, y, theta, kappa, dkappa 全部是严格的等式约束
      theta_lower = end_theta_;
      theta_upper = end_theta_;
      kappa_lower = end_kappa_;
      kappa_upper = end_kappa_;
      dkappa_lower = end_dkappa_;
      dkappa_upper = end_dkappa_;
      x_lower = end_x_;
      x_upper = end_x_;
      y_lower = end_y_;
      y_upper = end_y_;
    } else if (i + 1 == num_of_points_ && has_fixed_end_point_position_) {
      // 对于最后一个点的优化变量，只有 x, y 是严格的等式约束
      theta_lower = relative_theta_[i] - M_PI * 0.2;
      theta_upper = relative_theta_[i] + M_PI * 0.2;
      kappa_lower = -0.25;
      kappa_upper = 0.25;
      dkappa_lower = -0.02;
      dkappa_upper = 0.02;
      x_lower = end_x_;
      x_upper = end_x_;
      y_lower = end_y_;
      y_upper = end_y_;
    } else { 
      // 对于中间点的约束
      theta_lower = relative_theta_[i] - M_PI * 0.2;
      theta_upper = relative_theta_[i] + M_PI * 0.2;
      kappa_lower = -0.25; // 曲率的下边界
      kappa_upper = 0.25;  // 曲率的上边界
      dkappa_lower = -0.02; // 曲率变化率的下边界
      dkappa_upper = 0.02;  // 曲率变化率的上边界
      x_lower = init_points_[i].x() - default_max_point_deviation_;
      x_upper = init_points_[i].x() + default_max_point_deviation_;
      y_lower = init_points_[i].y() - default_max_point_deviation_;
      y_upper = init_points_[i].y() + default_max_point_deviation_;
    }

    // theta
    x_l[index] = theta_lower;
    x_u[index] = theta_upper;

    // kappa
    x_l[index + 1] = kappa_lower;
    x_u[index + 1] = kappa_upper;

    // dkappa
    x_l[index + 2] = dkappa_lower;
    x_u[index + 2] = dkappa_upper;

    // x
    x_l[index + 3] = x_lower;
    x_u[index + 3] = x_upper;

    // y
    x_l[index + 4] = y_lower;
    x_u[index + 4] = y_upper;
  }

  // b. for delta_s
  int variable_offset = num_of_points_ * 5;
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    // delta_s 的取值下界
    x_l[variable_offset + i] =
        point_distances_[i] - 2.0 * default_max_point_deviation_;
    // delta_s 的取值上界
    x_u[variable_offset + i] = point_distances_[i] * M_PI * 0.5; // ???
  }

  // constraints - 定义约束的上下边界
  // a. positional equality constraints - 位置的等式约束，其上下边界都为 0.0
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    // for x
    g_l[i * 2] = 0.0;
    g_u[i * 2] = 0.0;

    // for y
    g_l[i * 2 + 1] = 0.0;
    g_u[i * 2 + 1] = 0.0;
  }
  // b. positional deviation constraints - 位置平移约束的上下界
  int constraint_offset = 2 * (num_of_points_ - 1);
  for (int i = 0; i < num_of_points_; ++i) {
    g_l[constraint_offset + i] = 0.0; // 位置平移约束的下界
    g_u[constraint_offset + i] =
        default_max_point_deviation_ * default_max_point_deviation_;
        // 位置平移约束的上界 - 相当于 ri 的平方
  }
  return true;
}

// 给出优化变量的初始值，初始值为原始参考线上点的状态
bool SpiralProblemInterface::get_starting_point(int n, bool init_x, double* x,
                                                bool init_z, double* z_L,
                                                double* z_U, int m,
                                                bool init_lambda,
                                                double* lambda) {
  CHECK_EQ(n, num_of_variables_);
  ACHECK(init_x); // bool, init_x = true, 表示必须给所有的优化变量 x 赋初值
  ACHECK(!init_z); 
  ACHECK(!init_lambda);
  // 给每个点的 theta，曲率，曲率变化率，以及 x, y 坐标赋初值
  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 5;
    x[index] = relative_theta_[i];
    x[index + 1] = 0.0; // 中间点的曲率，曲率变化率初值为 0。
    x[index + 2] = 0.0;
    x[index + 3] = init_points_[i].x();
    x[index + 4] = init_points_[i].y();
  }
  // 求中间点的弧长
  int variable_offset = num_of_points_ * 5;
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    double delta_theta = relative_theta_[i + 1] - relative_theta_[i];
    x[variable_offset + i] = point_distances_[i] / std::cos(0.5 * delta_theta);
  }
  // 由上面计算得到的弧长，求中间点的曲率：角度变化 / 弧长
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    double delta_theta = relative_theta_[i + 1] - relative_theta_[i];
    x[(i + 1) * 5 + 1] = delta_theta / x[variable_offset + i];
    // 这里相当于给第二个点及其之后的点的曲率赋初值
    // i = 0, x[6] 
  }
  x[1] = x[6]; 
  // 第一个点的曲率，曲率变化率根据实际给出
  if (has_fixed_start_point_) {
    x[0] = start_theta_;
    x[1] = start_kappa_;
    x[2] = start_dkappa_;
  }
  return true;
}

// 定义目标函数
bool SpiralProblemInterface::eval_f(int n, const double* x, bool new_x,
                                    double& obj_value) {
  CHECK_EQ(n, num_of_variables_);
  if (new_x) {
    update_piecewise_spiral_paths(x, n);
  } 
  // 注：参数中 new_x 会反馈出求解器是否更新了优化变量，如果更新了优化变量，
  // 我们就要重新更新螺旋曲线函数，利用新的优化变量构造螺旋曲线
  // 新构造的螺旋曲线放在 piecewise_paths_ 这个容器中，
  // 里面每个元素都是 QuinticSpiralPathWithDerivation<N>

  obj_value = 0.0;
  // 这个 num_of_points_ 实际上就是 anchor points 的个数
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    const auto& spiral_curve = piecewise_paths_[i];
    double delta_s = spiral_curve.ParamLength(); // 第 i 段螺旋曲线的弧长

    obj_value += delta_s * weight_curve_length_; // 第 i 段曲线的长度 cost
    // 这里对每两个节点之间又均分出了 5 个内部节点，分别将内部节点的曲率和
    // 曲率变化率加权求和
    for (int j = 0; j < num_of_internal_points_; ++j) {
      double ratio =
          static_cast<double>(j) / static_cast<double>(num_of_internal_points_);
      // 内部节点在该段曲线上的 s 位置
      double s = ratio * delta_s;
      // 曲率加权和
      double kappa = spiral_curve.Evaluate(1, s);
      obj_value += kappa * kappa * weight_kappa_;
      // 曲率变化率加权和
      double dkappa = spiral_curve.Evaluate(2, s);
      obj_value += dkappa * dkappa * weight_dkappa_;
    }
  }
  return true;
}

// 定义目标函数的梯度，就是目标函数对于每一个优化变量的偏导数
bool SpiralProblemInterface::eval_grad_f(int n, const double* x, bool new_x,
                                         double* grad_f) {
  CHECK_EQ(n, num_of_variables_);
  std::fill(grad_f, grad_f + n, 0.0);
  // 和定义目标函数一样，梯度函数也需要首先更新螺旋曲线（根据标志位判断）
  // 这个 new_x 实际上是 ipopt 自带的标志位，用于判断是否需要更新自变量
  if (new_x) {
    update_piecewise_spiral_paths(x, n);
  }

  int variable_offset = num_of_points_ * 5;
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    int index0 = i * 5; 
    int index1 = (i + 1) * 5;
    // 注：这里之所以每一组是 5 个元素，是因为优化变量为下面的形式：
    // theta0，theta0', theta0'', x0, y0, theta1, theta1', theta1'', x1, y1...
    // - 由于目标函数中不包含 x, y 项，所以对 x, y 的偏导都为 0，所以：
    //     grad_f[index0 + 3] = 0
    //     grad_f[index0 + 4] = 0
    //     grad_f[index1 + 3] = 0
    //     grad_f[index1 + 4] = 0
    
    // 拿出第 i 条螺旋曲线
    auto& spiral_curve = piecewise_paths_[i];
    double delta_s = spiral_curve.ParamLength();
    // 第 i 条螺旋曲线对 delta_s_i 求偏导 - 第一项，length 部分
    grad_f[variable_offset + i] += weight_curve_length_ * 1.0; 

    for (int j = 0; j < num_of_internal_points_; ++j) {
      double ratio =
          static_cast<double>(j) / static_cast<double>(num_of_internal_points_);
      double s = ratio * delta_s;
      
       // 注：第 i 条螺旋曲线的参数，与 theta_i 和 theta_i+1 有关
      //    第 i+1 条螺旋曲线，与 theta_i+1 和 theta_i+2 有关

      // 计算第 i 条螺旋曲线上，第 j 个内部节点处的 kappa
      double kappa = spiral_curve.Evaluate(1, s); 
      // ------ 第 i 条螺旋曲线的一阶导表达式对优化变量求偏导部分 ----
      // 第 i 条螺旋曲线的一阶导表达式对起始点 theta 求偏导
      grad_f[index0] += weight_kappa_ * 2.0 * kappa *
                        spiral_curve.DeriveKappaDerivative(
                            THETA0, j, num_of_internal_points_); 
      // 第 i 条螺旋曲线的一阶导表达式对起始点 theta_dot 求偏导
      grad_f[index0 + 1] += weight_kappa_ * 2.0 * kappa *
                            spiral_curve.DeriveKappaDerivative(
                                KAPPA0, j, num_of_internal_points_);
      // 第 i 条螺旋曲线的一阶导表达式对起始点 theta_dot_dot 求偏导 
      grad_f[index0 + 2] += weight_kappa_ * 2.0 * kappa *
                            spiral_curve.DeriveKappaDerivative(
                                DKAPPA0, j, num_of_internal_points_); 
      // 第 i 条螺旋曲线的一阶导表达式对终止点的 theta, theta', theta'' 求偏导
      grad_f[index1] += weight_kappa_ * 2.0 * kappa *
                        spiral_curve.DeriveKappaDerivative(
                            THETA1, j, num_of_internal_points_); 
      grad_f[index1 + 1] += weight_kappa_ * 2.0 * kappa *
                            spiral_curve.DeriveKappaDerivative(
                                KAPPA1, j, num_of_internal_points_);
      grad_f[index1 + 2] += weight_kappa_ * 2.0 * kappa *
                            spiral_curve.DeriveKappaDerivative(
                                DKAPPA1, j, num_of_internal_points_);
      // 第 i 条螺旋曲线 - 对 delta_s_i 求偏导 - 第二项，kappa 部分
      grad_f[variable_offset + i] += weight_kappa_ * 2.0 * kappa *
                                     spiral_curve.DeriveKappaDerivative(
                                         DELTA_S, j, num_of_internal_points_); 

      // 计算第 i 条螺旋曲线，第 j 个节点处的 dkappa
      double dkappa = spiral_curve.Evaluate(2, s);
      // ------ 第 i 条螺旋曲线的二阶导表达式对优化变量求偏导部分 ---- 
      // 第 i 条螺旋曲线的二阶导表达式对起始点 theta 求偏导
      grad_f[index0] += weight_dkappa_ * 2.0 * dkappa *
                        spiral_curve.DeriveDKappaDerivative(
                            THETA0, j, num_of_internal_points_); 
      // 第 i 条螺旋曲线的二阶导表达式对起始点 theta_dot 求偏导 
      grad_f[index0 + 1] += weight_dkappa_ * 2.0 * dkappa *
                            spiral_curve.DeriveDKappaDerivative(
                                KAPPA0, j, num_of_internal_points_);
      // 第 i 条螺旋曲线的二阶导表达式对起始点 theta_dot_dot 求偏导 
      grad_f[index0 + 2] += weight_dkappa_ * 2.0 * dkappa * 
                            spiral_curve.DeriveDKappaDerivative(
                                DKAPPA0, j, num_of_internal_points_);
      // 第 i 条螺旋曲线的二阶导表达式对终止点的 theta，theta', theta'' 求偏导
      grad_f[index1] += weight_dkappa_ * 2.0 * dkappa *
                        spiral_curve.DeriveDKappaDerivative(
                            THETA1, j, num_of_internal_points_);  
      grad_f[index1 + 1] += weight_dkappa_ * 2.0 * dkappa *
                            spiral_curve.DeriveDKappaDerivative(
                                KAPPA1, j, num_of_internal_points_);
      grad_f[index1 + 2] += weight_dkappa_ * 2.0 * dkappa *
                            spiral_curve.DeriveDKappaDerivative(
                                DKAPPA1, j, num_of_internal_points_);
      // 第 i 条螺旋曲线 - 对 delta_s_i 求偏导 - 第三项，dkappa 部分
      grad_f[variable_offset + i] += weight_dkappa_ * 2.0 * dkappa *
                                     spiral_curve.DeriveDKappaDerivative(
                                         DELTA_S, j, num_of_internal_points_); 
    }
  }
  return true;
}

// 定义约束函数
// 其中 x 为优化变量，g 为约束函数返回值
bool SpiralProblemInterface::eval_g(int n, const double* x, bool new_x, int m,
                                    double* g) {
  CHECK_EQ(n, num_of_variables_);
  CHECK_EQ(m, num_of_constraints_);
  // 和目标函数一样，对于新的优化变量，约束函数首先更新螺旋曲线
  if (new_x) {
    update_piecewise_spiral_paths(x, n);
  }
  
  // 位置连续性约束（第 i 和 i+1 个点的 x, y 坐标需要相等）
  // first, fill in the positional equality constraints
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    int index0 = i * 5;
    int index1 = (i + 1) * 5;

    const auto& spiral_curve = piecewise_paths_[i];
    double delta_s = spiral_curve.ParamLength();    // 自变量的顺序是 theta, theta', theta'', x, y；
    // 所以 index0+3 正好是第 i 个点的 x 值，index1+3 是第 i+1 个点的 x 值
    double x_diff = x[index1 + 3] - x[index0 + 3] -
                    spiral_curve.ComputeCartesianDeviationX(delta_s);
    g[i * 2] = x_diff * x_diff;

    double y_diff = x[index1 + 4] - x[index0 + 4] -
                    spiral_curve.ComputeCartesianDeviationY(delta_s);
    g[i * 2 + 1] = y_diff * y_diff;
  }
  // 位置偏移约束
  // second, fill in the positional deviation constraints
  int constraint_offset = 2 * (num_of_points_ - 1);
  for (int i = 0; i < num_of_points_; ++i) {
    int variable_index = i * 5;
    double x_cor = x[variable_index + 3];  // xi
    double y_cor = x[variable_index + 4];  // yi

    double x_diff = x_cor - init_points_[i].x(); // 和第 i 个点的初始 x 坐标做差
    double y_diff = y_cor - init_points_[i].y(); // 和第 i 个点的初始 y 坐标做差

    g[constraint_offset + i] = x_diff * x_diff + y_diff * y_diff;
  }
  return true;
}
// 定义约束函数的 Jacobian 矩阵
// 注：约束函数的雅可比矩阵是通过稀疏矩阵的形式构造的
// 其中 iRow，jCol 分别表示雅克比矩阵非 0 项的行号和列号的数组
// nele_jac 是雅克比矩阵非 0 项的个数
// value 为雅克比矩阵非 0 项的值
bool SpiralProblemInterface::eval_jac_g(int n, const double* x, bool new_x,
                                        int m, int nele_jac, int* iRow,
                                        int* jCol, double* values) {
  CHECK_EQ(n, num_of_variables_);
  CHECK_EQ(m, num_of_constraints_);
// 当求解器第一次调用该函数时，values 为空指针，此时需要定义矩阵中
// 非恒为 0 项的 iRow，iCol。
// 之后求解器调用该函数，values 将始终有指针，此时求解雅克比矩阵非 0 项的值。
  if (values == nullptr) {
    int nz_index = 0;

    int variable_offset = num_of_points_ * 5;
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      int variable_index = i * 5;

      // theta0
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 0;
      ++nz_index;

      // kappa0
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 1;
      ++nz_index;

      // dkappa0
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 2;
      ++nz_index;

      // x0
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 3;
      ++nz_index;

      // theta1
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 5;
      ++nz_index;

      // kappa1
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 6;
      ++nz_index;

      // dkappa1
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 7;
      ++nz_index;

      // x1
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_index + 8;
      ++nz_index;

      // s
      iRow[nz_index] = i * 2;
      jCol[nz_index] = variable_offset + i;
      ++nz_index;

      // theta0
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 0;
      ++nz_index;

      // kappa0
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 1;
      ++nz_index;

      // dkappa0
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 2;
      ++nz_index;

      // y0
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 4;
      ++nz_index;

      // theta1
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 5;
      ++nz_index;

      // kappa1
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 6;
      ++nz_index;

      // dkappa1
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 7;
      ++nz_index;

      // y1
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_index + 9;
      ++nz_index;

      // s
      iRow[nz_index] = i * 2 + 1;
      jCol[nz_index] = variable_offset + i;
      ++nz_index;
    }

    int constraint_offset = 2 * (num_of_points_ - 1);
    for (int i = 0; i < num_of_points_; ++i) {
      iRow[nz_index] = constraint_offset + i;
      jCol[nz_index] = i * 5 + 3;
      ++nz_index;

      iRow[nz_index] = constraint_offset + i;
      jCol[nz_index] = i * 5 + 4;
      ++nz_index;
    }

    CHECK_EQ(nz_index, nele_jac);
  } else {
    if (new_x) {
      update_piecewise_spiral_paths(x, n);
    }

    std::fill(values, values + nele_jac, 0.0);
    // first, positional equality constraints
    int nz_index = 0;
    // 雅克比矩阵的计算，最后每个元素的值会存在 values[] 里面
    for (int i = 0; i + 1 < num_of_points_; ++i) {
      int index0 = i * 5;
      int index1 = (i + 1) * 5;

      auto& spiral_curve = piecewise_paths_[i];
      double delta_s = spiral_curve.ParamLength();

      double x_diff = x[index1 + 3] - x[index0 + 3] -
                      spiral_curve.ComputeCartesianDeviationX(delta_s);
      double y_diff = x[index1 + 4] - x[index0 + 4] -
                      spiral_curve.ComputeCartesianDeviationY(delta_s);

      auto pos_theta0 = spiral_curve.DeriveCartesianDeviation(THETA0);
      auto pos_kappa0 = spiral_curve.DeriveCartesianDeviation(KAPPA0);
      auto pos_dkappa0 = spiral_curve.DeriveCartesianDeviation(DKAPPA0);

      auto pos_theta1 = spiral_curve.DeriveCartesianDeviation(THETA1);
      auto pos_kappa1 = spiral_curve.DeriveCartesianDeviation(KAPPA1);
      auto pos_dkappa1 = spiral_curve.DeriveCartesianDeviation(DKAPPA1);

      auto pos_delta_s = spiral_curve.DeriveCartesianDeviation(DELTA_S);

      // for x coordinate - 约束 g1 对各项的偏导
      // theta0
      values[nz_index] += 2.0 * x_diff * (-pos_theta0.first);
      ++nz_index;

      // kappa0
      values[nz_index] += 2.0 * x_diff * (-pos_kappa0.first);
      ++nz_index;

      // dkappa0
      values[nz_index] += 2.0 * x_diff * (-pos_dkappa0.first);
      ++nz_index;

      // x0
      values[nz_index] += 2.0 * x_diff * (-1.0);
      ++nz_index;

      // theta1
      values[nz_index] += 2.0 * x_diff * (-pos_theta1.first);
      ++nz_index;

      // kappa1
      values[nz_index] += 2.0 * x_diff * (-pos_kappa1.first);
      ++nz_index;

      // dkappa1
      values[nz_index] += 2.0 * x_diff * (-pos_dkappa1.first);
      ++nz_index;

      // x1
      values[nz_index] += 2.0 * x_diff;
      ++nz_index;

      // delta_s
      values[nz_index] += 2.0 * x_diff * (-pos_delta_s.first);
      ++nz_index;

      // for y coordinate
      // theta0
      values[nz_index] += 2.0 * y_diff * (-pos_theta0.second);
      ++nz_index;

      // kappa0
      values[nz_index] += 2.0 * y_diff * (-pos_kappa0.second);
      ++nz_index;

      // dkappa0
      values[nz_index] += 2.0 * y_diff * (-pos_dkappa0.second);
      ++nz_index;

      // y0
      values[nz_index] += 2.0 * y_diff * (-1.0);
      ++nz_index;

      // theta1
      values[nz_index] += 2.0 * y_diff * (-pos_theta1.second);
      ++nz_index;

      // kappa1
      values[nz_index] += 2.0 * y_diff * (-pos_kappa1.second);
      ++nz_index;

      // dkappa1
      values[nz_index] += 2.0 * y_diff * (-pos_dkappa1.second);
      ++nz_index;

      // y1
      values[nz_index] += 2.0 * y_diff;
      ++nz_index;

      // delta_s
      values[nz_index] += 2.0 * y_diff * (-pos_delta_s.second);
      ++nz_index;
    }
    // 位置平移约束，对 x1 和 yi 的偏导数（它对 theta0 和 theta1 的偏导都为 0）。
    for (int i = 0; i < num_of_points_; ++i) {
      values[nz_index] = 2.0 * (x[i * 5 + 3] - init_points_[i].x());
      ++nz_index;

      values[nz_index] = 2.0 * (x[i * 5 + 4] - init_points_[i].y());
      ++nz_index;
    }

    CHECK_EQ(nz_index, nele_jac);
  }
  return true;
}
// 定义 Hessian 矩阵
// 这里由于设置了调用 ipopt 的拟牛顿法近似求解二阶偏导数，
// 代码：app->Options()->SetStringValue("hessian_approximation","limited-memory");
// 所以函数 eval_h() 这里无需实现
bool SpiralProblemInterface::eval_h(int n, const double* x, bool new_x,
                                    double obj_factor, int m,
                                    const double* lambda, bool new_lambda,
                                    int nele_hess, int* iRow, int* jCol,
                                    double* values) {
  ACHECK(false);
  return true;
}
// 获取优化结果并转存下来：
void SpiralProblemInterface::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  opt_theta_.reserve(num_of_points_);
  opt_kappa_.reserve(num_of_points_);
  opt_dkappa_.reserve(num_of_points_);
  opt_x_.reserve(num_of_points_);
  opt_y_.reserve(num_of_points_);
  for (int i = 0; i < num_of_points_; ++i) {
    int index = i * 5;
    opt_theta_.push_back(x[index]);
    opt_kappa_.push_back(x[index + 1]);
    opt_dkappa_.push_back(x[index + 2]);
    opt_x_.push_back(x[index + 3]);
    opt_y_.push_back(x[index + 4]);
  }

  opt_s_.reserve(num_of_points_ - 1);
  int variable_offset = num_of_points_ * 5;
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    opt_s_.push_back(x[variable_offset + i]);
  }
}

void SpiralProblemInterface::update_piecewise_spiral_paths(const double* x,
                                                           const int n) {
  int variable_offset = num_of_points_ * 5;
  for (int i = 0; i + 1 < num_of_points_; ++i) {
    int index0 = i * 5;
    int index1 = (i + 1) * 5;

    std::array<double, 3> x0 = {x[index0], x[index0 + 1], x[index0 + 2]};
    std::array<double, 3> x1 = {x[index1], x[index1 + 1], x[index1 + 2]};
    // x0 - 第 i 段螺旋曲线的起始 theta, theta_dot, theta_dot_dot
    // x1 - 第 i+1 段螺旋曲线的起始 theta, theta_dot, theta_dot_dot
    double delta_s = x[variable_offset + i]; // 取出每一段螺旋曲线的 delta_s
    // 这里就体现出 x 中优化变量的顺序：
    // theta0, theta_dot0, theta_dotdot0, x0, y0 - 第一段螺旋曲线的优化变量（不包括 delta_s）
    // theta1, theta_dot1, theta_dotdot1, x1, y1 - 第二段螺旋曲线的优化变量
    // theta2, theta_dot2, theta_dotdot2, x2, y2 - 第二段螺旋曲线的优化变量
    // delta_s0 - 第一段螺旋曲线的 delta_s
    // delta_s1 - 第二段螺旋曲线的 delta_s
    // delta_s2 - 第三段螺旋曲线的 delta_s
    piecewise_paths_[i] =
        std::move(QuinticSpiralPathWithDerivation<N>(x0, x1, delta_s));
  }
}

void SpiralProblemInterface::set_default_max_point_deviation(
    const double max_point_deviation) {
  default_max_point_deviation_ = max_point_deviation;
}

void SpiralProblemInterface::set_start_point(const double x, const double y,
                                             const double theta,
                                             const double kappa,
                                             const double dkappa) {
  has_fixed_start_point_ = true;
  start_x_ = x;
  start_y_ = y;
  start_theta_ = theta;
  start_kappa_ = kappa;
  start_dkappa_ = dkappa;
}

void SpiralProblemInterface::set_end_point(const double x, const double y,
                                           const double theta,
                                           const double kappa,
                                           const double dkappa) {
  has_fixed_end_point_ = true;
  end_x_ = x;
  end_y_ = y;
  end_theta_ = theta;
  end_kappa_ = kappa;
  end_dkappa_ = dkappa;
}

void SpiralProblemInterface::set_end_point_position(const double x,
                                                    const double y) {
  has_fixed_end_point_position_ = true;
  end_x_ = x;
  end_y_ = y;
}

void SpiralProblemInterface::set_element_weight_curve_length(
    const double weight_curve_length) {
  weight_curve_length_ = weight_curve_length;
}

void SpiralProblemInterface::set_element_weight_kappa(
    const double weight_kappa) {
  weight_kappa_ = weight_kappa;
}

void SpiralProblemInterface::set_element_weight_dkappa(
    const double weight_dkappa) {
  weight_dkappa_ = weight_dkappa;
}

}  // namespace planning
}  // namespace apollo
