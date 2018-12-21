/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mrinal Kalakrishnan */
//我在这里加了一条注释
#include <chomp_motion_planner/chomp_cost.h>
#include <chomp_motion_planner/chomp_utils.h>
#include <eigen3/Eigen/LU>

using namespace Eigen;
using namespace std;

namespace chomp
{
ChompCost::ChompCost(const ChompTrajectory& trajectory, int joint_number, const std::vector<double>& derivative_costs,
                     double ridge_factor)
{
  int num_vars_all = trajectory.getNumPoints();//轨迹点数量
  int num_vars_free = num_vars_all - 2 * (DIFF_RULE_LENGTH - 1);//在这里DIFF_RULE_LENGTH=7，num_vars_free =num_vars_all -12
  MatrixXd diff_matrix = MatrixXd::Zero(num_vars_all, num_vars_all);//差分矩阵的大小等于轨迹点数量
  quad_cost_full_ = MatrixXd::Zero(num_vars_all, num_vars_all);//与差分矩阵大小一致

  // construct the quad cost for all variables, as a sum of squared differentiation matrices
  double multiplier = 1.0;
  for (unsigned int i = 0; i < derivative_costs.size(); i++)
  {
    multiplier *= trajectory.getDiscretization();//还不知道这是啥，时间间隔
    diff_matrix = getDiffMatrix(num_vars_all, &DIFF_RULES[i][0]);//这里的DIFF_RULES是预先定义好的，生成的差分矩阵以第四个元素为对角线
    quad_cost_full_ += (derivative_costs[i] * multiplier) * (diff_matrix.transpose() * diff_matrix);//这里实际上只有加速度差分矩阵，其他的derivative_costs=0，这里输出的结果是A=KT×K
  }
  quad_cost_full_ += MatrixXd::Identity(num_vars_all, num_vars_all) * ridge_factor;//在对角线上加，这里是0，这里是最后的uad_cost_full_

  // extract the quad cost just for the free variables:
  quad_cost_ = quad_cost_full_.block(DIFF_RULE_LENGTH - 1, DIFF_RULE_LENGTH - 1, num_vars_free, num_vars_free);//取分块矩阵，取（6，6）开始，取轨迹点-12长度的矩阵

  // invert the matrix:
  quad_cost_inv_ = quad_cost_.inverse();

  // cout << quad_cost_inv_ << endl;
}

Eigen::MatrixXd ChompCost::getDiffMatrix(int size, const double* diff_rule) const//构成差分矩阵，size是矩阵大小，第二个参数是预先定义好的
{
  MatrixXd matrix = MatrixXd::Zero(size, size);//求构成和轨迹一样大小的0矩阵
  for (int i = 0; i < size; i++)
  {
    for (int j = -DIFF_RULE_LENGTH / 2; j <= DIFF_RULE_LENGTH / 2; j++)//在实际运用中，for (int j = -7/ 2; j <= 7/ 2; j++)，即for (int j = -4; j <= 3; j++)
    {
      int index = i + j;
      if (index < 0)
        continue;
      if (index >= size)//小于轨迹长度？
        continue;
      matrix(i, index) = diff_rule[j + DIFF_RULE_LENGTH / 2];
    }
  }
  return matrix;
}

double ChompCost::getMaxQuadCostInvValue() const
{
  return quad_cost_inv_.maxCoeff();//差分矩阵逆的最大元素
}

void ChompCost::scale(double scale)
{
  double inv_scale = 1.0 / scale;
  quad_cost_inv_ *= inv_scale;
  quad_cost_ *= scale;
  quad_cost_full_ *= scale;
}

ChompCost::~ChompCost()
{
}
}
