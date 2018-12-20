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

#ifndef CHOMP_TRAJECTORY_H_
#define CHOMP_TRAJECTORY_H_

#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_model/robot_model.h>
#include <chomp_motion_planner/chomp_utils.h>

#include <moveit_msgs/MotionPlanDetailedResponse.h>
#include <moveit_msgs/MotionPlanRequest.h>

#include <vector>
#include <eigen3/Eigen/Core>

namespace chomp
{
/**
 * \brief Represents a discretized joint-space trajectory for CHOMP
 */
class ChompTrajectory
{
public:
  /**
   * \brief Constructs a trajectory for a given robot model, trajectory duration, and discretization
   */
  ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model, double duration, double discretization,
                  std::string groupName);

  /**
   * \brief Constructs a trajectory for a given robot model, number of trajectory points, and discretization
   */
  ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model, int num_points, double discretization,
                  std::string groupName);

  /**
   * \brief Creates a new containing only the joints of interest, and adds padding to the start
   * and end if needed, to have enough trajectory points for the differentiation rules
   */
  ChompTrajectory(const ChompTrajectory& source_traj, const std::string& planning_group, int diff_rule_length);

  ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model, const std::string& planning_group,
                  const trajectory_msgs::JointTrajectory& traj);

  /**
   * \brief Destructor
   */
  virtual ~ChompTrajectory();

  double& operator()(int traj_point, int joint);

  double operator()(int traj_point, int joint) const;

  Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point);

  Eigen::MatrixXd::ColXpr getJointTrajectory(int joint);//获得当前关节的列向量

  void overwriteTrajectory(const trajectory_msgs::JointTrajectory& traj);

  /**
   * \brief Gets the number of points in the trajectory 获得轨迹的点数量
   */
  int getNumPoints() const;

  /**
   * \brief Gets the number of points (that are free to be optimized) in the trajectory获得可以被优化的轨迹的点数量
   */
  int getNumFreePoints() const;

  /**
   * \brief Gets the number of joints in each trajectory point获得每个轨迹点的关节数
   */
  int getNumJoints() const;

  /**
   * \brief Gets the discretization time interval of the trajectory获得轨迹的离散时间间隔
   */
  double getDiscretization() const;

  /**
   * \brief Generates a minimum jerk trajectory from the start index to end index
   *
   * Only modifies points from start_index_ to end_index_, inclusive.生成一个最小加加速度轨迹从开始到结束，只修改开始到结尾
   */
  void fillInMinJerk();

  /**
   * \brief Generates a linearly interpolated trajectory from the start index to end index生成一个线性插值轨迹，从开始到结束
   *
   * Only modifies points from start_index_ to end_index_, inclusive
   */
  void fillInLinearInterpolation();

  /**
   * \brief Generates a cubic interpolation of the trajectory from the start index to end index生成三次插值轨迹，从开始到结束
   *
   * Only modifies points from start_index_ to end_index_, inclusive
   */
  void fillInCubicInterpolation();

  /**
   * \brief Receives the path obtained from a given MotionPlanDetailedResponse res object's trajectory (e.g., trajectory
   * produced by OMPL) and puts it into the appropriate trajectory format required for CHOMP 按照CHOMP的格式填充一个预规划的轨迹
   * @param res
   */
  bool fillInFromTrajectory(moveit_msgs::MotionPlanDetailedResponse& res);

  /**
   * This function assigns the chomp_trajectory row / robot pose at index 'chomp_trajectory_point' obtained from input
   * trajectory_msgs at index 'trajectory_msgs_point'此函数分配轨迹row和从trajectory_msgs获得的机器人姿态
   * @param trajectory_msg the input trajectory_msg获得轨迹输入
   * @param num_joints_trajectory number of joints in the given robot trajectory获得给定机器人轨迹的关节数
   * @param trajectory_msgs_point index of the input trajectory_msg's point to get joint values from 给输入的轨迹点编码
   * @param chomp_trajectory_point index of the chomp_trajectory's point to get joint values from给CHOMP轨迹编码
   */
  void assignCHOMPTrajectoryPointFromInputTrajectoryPoint(moveit_msgs::RobotTrajectory trajectory_msg,
                                                          int num_joints_trajectory, int trajectory_msgs_point,
                                                          int chomp_trajectory_point);

  /**
   * \brief Sets the start and end index for the modifiable part of the trajectory
   *
   * (Everything before the start and after the end index is considered fixed)
   * The values default to 1 and getNumPoints()-2给轨迹设置开始和结束，开始和结束被认为是固定的，默认值为1，getNumPoints()-2
   */
  void setStartEndIndex(int start_index, int end_index);

  /**
   * \brief Gets the start index得到开始的编码
   */
  int getStartIndex() const;

  /**
   * \brief Gets the end index得到结束的编码
   */
  int getEndIndex() const;

  /**
   * \brief Gets the entire trajectory matrix得到整个轨迹矩阵
   */
  Eigen::MatrixXd& getTrajectory();

  /**
   * \brief Gets the block of the trajectory which can be optimized得到可以被优化的轨迹块
   */
  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeTrajectoryBlock();

  /**
   * \brief Gets the block of free (optimizable) trajectory for a single 得到单关节可被优化的块
   */
  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeJointTrajectoryBlock(int joint);

  /**
   * \brief Updates the full trajectory (*this) from the group trajectory更新整体轨迹
   */
  void updateFromGroupTrajectory(const ChompTrajectory& group_trajectory);

  /**
   * \brief Gets the index in the full trajectory which was copied to this group trajectory得到完整的轨迹编码
   */
  int getFullTrajectoryIndex(int i) const;

  /**
   * \brief Gets the joint velocities at the given trajectory point得到给定轨迹速度
   */
  template <typename Derived>
  void getJointVelocities(int traj_point, Eigen::MatrixBase<Derived>& velocities);

  double getDuration() const;

private:
  void init(); /**< \brief Allocates memory for the trajectory 分配轨迹内存*/

  std::string planning_group_name_; /**< Planning group that this trajectory corresponds to, if any */
  int num_points_;                  /**< Number of points in the trajectory 轨迹点数量*/
  int num_joints_;                  /**< Number of joints in each trajectory point 每个轨迹点的关节数*/
  double discretization_;           /**< Discretization of the trajectory 离散化的轨迹*/
  double duration_;                 /**< Duration of the trajectory 时间轨迹 */
  Eigen::MatrixXd trajectory_;      /**< Storage for the actual trajectory 存储实际轨迹 */
  int start_index_; /**< Start index (inclusive) of trajectory to be optimized (everything before it will not be
                       modified) 开始轨迹的编码优化*/
  int end_index_; /**< End index (inclusive) of trajectory to be optimized (everything after it will not be modified) 轨迹优化的结束编码*/
  std::vector<int> full_trajectory_index_; /**< If this is a "group" trajectory, the index from the original traj which
                                              each
                                              element here was copied 如说是轨迹集合，复制每个元素的编码 */
};

///////////////////////// inline functions follow //////////////////////

inline double& ChompTrajectory::operator()(int traj_point, int joint)
{
  return trajectory_(traj_point, joint);
}

inline double ChompTrajectory::operator()(int traj_point, int joint) const
{
  return trajectory_(traj_point, joint);
}

inline Eigen::MatrixXd::RowXpr ChompTrajectory::getTrajectoryPoint(int traj_point)
{
  return trajectory_.row(traj_point);
}

inline Eigen::MatrixXd::ColXpr ChompTrajectory::getJointTrajectory(int joint)
{
  return trajectory_.col(joint);
}

inline int ChompTrajectory::getNumPoints() const
{
  return num_points_;
}

inline int ChompTrajectory::getNumFreePoints() const
{
  return (end_index_ - start_index_) + 1;
}

inline int ChompTrajectory::getNumJoints() const
{
  return num_joints_;
}

inline double ChompTrajectory::getDiscretization() const
{
  return discretization_;
}

inline void ChompTrajectory::setStartEndIndex(int start_index, int end_index)
{
  start_index_ = start_index;
  end_index_ = end_index;
}

inline int ChompTrajectory::getStartIndex() const
{
  return start_index_;
}

inline int ChompTrajectory::getEndIndex() const
{
  return end_index_;
}

inline Eigen::MatrixXd& ChompTrajectory::getTrajectory()
{
  return trajectory_;
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ChompTrajectory::getFreeTrajectoryBlock()
{
  return trajectory_.block(start_index_, 0, getNumFreePoints(), getNumJoints());
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
ChompTrajectory::getFreeJointTrajectoryBlock(int joint)
{
  return trajectory_.block(start_index_, joint, getNumFreePoints(), 1);
}

inline int ChompTrajectory::getFullTrajectoryIndex(int i) const
{
  return full_trajectory_index_[i];
}

template <typename Derived>
void ChompTrajectory::getJointVelocities(int traj_point, Eigen::MatrixBase<Derived>& velocities)
{
  velocities.setZero();
  double invTime = 1.0 / discretization_;

  for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++)
  {
    velocities += (invTime * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2]) * trajectory_.row(traj_point + k).transpose();
  }
}

inline double ChompTrajectory::getDuration() const
{
  return duration_;
}
}

#endif /* CHOMP_TRAJECTORY_H_ */
