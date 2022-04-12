/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/common/math/mpc_solver.h"

#include <algorithm>
#include <memory>

#include "cyber/common/log.h"
#include "modules/common/math/qp_solver/active_set_qp_solver.h"
#include "modules/common/math/qp_solver/qp_solver.h"

namespace apollo {
namespace common {
namespace math {

using Matrix = Eigen::MatrixXd;

// discrete linear predictive control solver, with control format
// x(i + 1) = A * x(i) + B * u (i) + C
// 注：这里并没有构造新的状态空间方程，将 状态量和控制量写在一起。
// matrix_a - matrix_ad，即离散化后的 A 矩阵
// matrix_b - matrix_bd
// matrix_c - matrix_cd
bool SolveLinearMPC(const Matrix &matrix_a, const Matrix &matrix_b,
                    const Matrix &matrix_c, const Matrix &matrix_q,
                    const Matrix &matrix_r, const Matrix &matrix_lower,
                    const Matrix &matrix_upper,
                    const Matrix &matrix_initial_state,
                    const std::vector<Matrix> &reference, const double eps,
                    const int max_iter, std::vector<Matrix> *control,
                    std::vector<Matrix> *control_gain,
                    std::vector<Matrix> *addition_gain) {
  if (matrix_a.rows() != matrix_a.cols() ||
      matrix_b.rows() != matrix_a.rows() ||
      matrix_lower.rows() != matrix_upper.rows()) {
    AERROR << "One or more matrices have incompatible dimensions. Aborting.";
    return false;
  }

  size_t horizon = static_cast<size_t>(reference.size()); // horizon = 10

  // Update augment reference matrix_t - 构造 Yref 矩阵
  Matrix matrix_t = Matrix::Zero(matrix_b.rows() * horizon, 1); // 维度 60x1
  for (size_t j = 0; j < horizon; ++j) {
    matrix_t.block(j * reference[0].size(), 0, reference[0].size(), 1) =
        reference[j]; // matrix.block(i, j, p, q) - 子矩阵的行数和列数是 p，q；子矩阵第一个元素的位置是 i，j 
        // 这里化简的写法就是 matrix_t.block(j * 6, 0, 6, 1)
        // 表示：子矩阵始终是 6 行 1 列的矩阵，把 reference 中的第 j 个 matrix 赋给它。
  }

  // Update augment control matrix_v
  Matrix matrix_v = Matrix::Zero((*control)[0].rows() * horizon, 1);
  for (size_t j = 0; j < horizon; ++j) {
    matrix_v.block(j * (*control)[0].rows(), 0, (*control)[0].rows(), 1) =
        (*control)[j];
        // control 是一个指向 vector<Matrix> 的指针，这里首先进行了解引用，然后找第 j 个元素赋给 matrix_v 的某一个子矩阵。
        // 这里化简的写法就是 matrix_v.block(j * 2, 0, 2, 1) 
        // 表示：子矩阵是 2 行 1 列的矩阵；子矩阵第一个元素的位置是 (j*2, 0)
  }
  // 构造 AP 矩阵
  std::vector<Matrix> matrix_a_power(horizon); // AP 矩阵是一个有 10 个元素的 vector
  matrix_a_power[0] = matrix_a;
  for (size_t i = 1; i < matrix_a_power.size(); ++i) {
    matrix_a_power[i] = matrix_a * matrix_a_power[i - 1]; // 第 i 个元素为 A 的 i 次方。
  }
  // matrix_k 的维度为 60x20
  Matrix matrix_k =
      Matrix::Zero(matrix_b.rows() * horizon, matrix_b.cols() * horizon);  // matrix_b 的维度为 6x2
  matrix_k.block(0, 0, matrix_b.rows(), matrix_b.cols()) = matrix_b; // 第一个子矩阵就是 matrix_b
  for (size_t r = 1; r < horizon; ++r) {
    for (size_t c = 0; c < r; ++c) { // c < r, 是因为 matrix_k 是下三角矩阵。
      matrix_k.block(r * matrix_b.rows(), c * matrix_b.cols(), matrix_b.rows(),
                     matrix_b.cols()) = matrix_a_power[r - c - 1] * matrix_b;
    }
    matrix_k.block(r * matrix_b.rows(), r * matrix_b.cols(), matrix_b.rows(),
                   matrix_b.cols()) = matrix_b; // 对角线矩阵始终为 B 矩阵（即 B_tilde 矩阵）
  }
  // Initialize matrix_m, matrix_t and matrix_v, matrix_qq, matrix_rr,
  // vector of matrix A power
  Matrix matrix_m = Matrix::Zero(matrix_b.rows() * horizon, 1); // 60x1
  Matrix matrix_qq = Matrix::Zero(matrix_k.rows(), matrix_k.rows()); // 60x60
  Matrix matrix_rr = Matrix::Zero(matrix_k.cols(), matrix_k.cols());
  Matrix matrix_ll = Matrix::Zero(horizon * matrix_lower.rows(), 1);
  Matrix matrix_uu = Matrix::Zero(horizon * matrix_upper.rows(), 1);
  Matrix matrix_cc = Matrix::Zero(horizon * matrix_c.rows(), 1);
  Matrix matrix_aa = Matrix::Zero(horizon * matrix_a.rows(), matrix_a.cols());
  matrix_aa.block(0, 0, matrix_a.rows(), matrix_a.cols()) = matrix_a;

  for (size_t i = 1; i < horizon; ++i) {
    matrix_aa.block(i * matrix_a.rows(), 0, matrix_a.rows(), matrix_a.cols()) =
        matrix_a * matrix_aa.block((i - 1) * matrix_a.rows(), 0,
                                   matrix_a.rows(), matrix_a.cols());
  }

  // Compute matrix_m
  matrix_m.block(0, 0, matrix_a.rows(), 1) = matrix_a * matrix_initial_state;
  for (size_t i = 1; i < horizon; ++i) {
    matrix_m.block(i * matrix_a.rows(), 0, matrix_a.rows(), 1) =
        matrix_a *
        matrix_m.block((i - 1) * matrix_a.rows(), 0, matrix_a.rows(), 1);
  }

  // Compute matrix_ll, matrix_uu, matrix_qq, matrix_rr
  for (size_t i = 0; i < horizon; ++i) {
    matrix_ll.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1) =
        matrix_lower;
    matrix_uu.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1) =
        matrix_upper;
    matrix_qq.block(i * matrix_q.rows(), i * matrix_q.rows(), matrix_q.rows(),
                    matrix_q.rows()) = matrix_q;
    matrix_rr.block(i * matrix_r.rows(), i * matrix_r.rows(), matrix_r.cols(),
                    matrix_r.cols()) = matrix_r;
  }

  matrix_cc.block(0, 0, matrix_c.rows(), 1) = matrix_c;
  for (size_t i = 1; i < horizon; ++i) {
    matrix_cc.block(i * matrix_c.rows(), 0, matrix_c.rows(), 1) =
        matrix_cc.block((i - 1) * matrix_c.rows(), 0, matrix_c.rows(), 1) +
        matrix_aa.block((i - 1) * matrix_a.rows(), 0, matrix_a.rows(),
                        matrix_a.cols()) *
            matrix_c;
  }

  // Update matrix_m1, matrix_m2, convert MPC problem to QP problem
  const Matrix matrix_m1 = static_cast<Matrix>(
      matrix_k.transpose() * matrix_qq * matrix_k + matrix_rr);
  const Matrix matrix_m2 = static_cast<Matrix>(
      matrix_k.transpose() * matrix_qq * (matrix_m + matrix_cc - matrix_t));
  // Update matrix_m0, matrix_ctrl_gain, matrix_add_gain, obtain the analytical
  // control gain matrix, corresponding to the unconstrained QP problem
  const Matrix matrix_m0 = static_cast<Matrix>(
      -1 * matrix_m1.inverse() * matrix_k.transpose() * matrix_qq);
  const Matrix matrix_ctrl_gain = static_cast<Matrix>(matrix_m0 * matrix_aa);
  const Matrix matrix_add_gain = static_cast<Matrix>(matrix_m0 * matrix_cc);

  // Format in qp_solver
  /**
   * *           min_x  : q(x) = 0.5 * x^T * Q * x  + x^T c
   * *           with respect to:  A * x = b (equality constraint)
   * *                             C * x >= d (inequality constraint)
   * **/

  // TODO(QiL) : change qp solver to box constraint or substitute QPOASES
  // Method 1: QPOASES
  Matrix matrix_inequality_constrain_ll =
      Matrix::Identity(matrix_ll.rows(), matrix_ll.rows());
  Matrix matrix_inequality_constrain_uu =
      Matrix::Identity(matrix_uu.rows(), matrix_uu.rows());
  Matrix matrix_inequality_constrain =
      Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.rows());
  matrix_inequality_constrain << matrix_inequality_constrain_ll,
      -matrix_inequality_constrain_uu;
  Matrix matrix_inequality_boundary =
      Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.cols());
  matrix_inequality_boundary << matrix_ll, -matrix_uu;
  Matrix matrix_equality_constrain =
      Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.rows());
  Matrix matrix_equality_boundary =
      Matrix::Zero(matrix_ll.rows() + matrix_uu.rows(), matrix_ll.cols());

  std::unique_ptr<QpSolver> qp_solver(new ActiveSetQpSolver(
      matrix_m1, matrix_m2, matrix_inequality_constrain,
      matrix_inequality_boundary, matrix_equality_constrain,
      matrix_equality_boundary));
  auto result = qp_solver->Solve();
  if (!result) {
    AERROR << "Linear MPC solver failed";
    return false;
  }
  matrix_v = qp_solver->params();

  for (size_t i = 0; i < horizon; ++i) {
    (*control)[i] =
        matrix_v.block(i * (*control)[0].rows(), 0, (*control)[0].rows(), 1);
  }

  for (size_t i = 0; i < horizon; ++i) {
    (*control_gain)[i] = matrix_ctrl_gain.block(i * (*control_gain)[0].rows(),
                                                0, (*control_gain)[0].rows(),
                                                (*control_gain)[0].cols());
  }

  for (size_t i = 0; i < horizon; ++i) {
    (*addition_gain)[i] = matrix_add_gain.block(
        i * (*addition_gain)[0].rows(), 0, (*addition_gain)[0].rows(), 1);
  }

  return true;
}

}  // namespace math
}  // namespace common
}  // namespace apollo
