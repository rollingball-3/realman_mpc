/******************************************************************************
Copyright (c) 2023. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <memory>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace ocs2 {
namespace mobile_manipulator {

/**
 * 底盘朝向约束类
 * 
 * 该约束计算底盘朝向与目标方向之间的夹角误差，作为优化目标
 * 优化目标是让底盘朝向尽量指向末端执行器的目标位置
 */
class BaseOrientationConstraint final : public StateConstraint {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
  using quaternion_t = Eigen::Quaternion<scalar_t>;

  /**
   * 构造函数
   * 
   * @param endEffectorKinematics 末端执行器运动学
   * @param referenceManager 参考轨迹管理器
   */
  BaseOrientationConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, 
                           const ReferenceManager& referenceManager);
  
  ~BaseOrientationConstraint() override = default;
  
  BaseOrientationConstraint* clone() const override { 
    return new BaseOrientationConstraint(*endEffectorKinematicsPtr_, *referenceManagerPtr_); 
  }

  size_t getNumConstraints(scalar_t time) const override { 
    return 1;  // 只有一个角度约束
  }
  
  vector_t getValue(scalar_t time, const vector_t& state, 
                   const PreComputation& preComputation) const override;
                   
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                         const PreComputation& preComputation) const override;

 private:
  BaseOrientationConstraint(const BaseOrientationConstraint& other) = default;
  
  /**
   * 从参考轨迹中插值获取末端执行器目标位置
   */
  vector3_t interpolateEndEffectorTargetPosition(scalar_t time) const;
  
  /**
   * 计算底盘朝向角误差
   * 
   * @param state 当前状态向量
   * @param targetPosition 目标位置
   * @return 底盘朝向与目标方向之间的角度差
   */
  scalar_t computeBaseOrientationError(const vector_t& state, const vector3_t& targetPosition) const;

  /** Pinocchio末端执行器运动学指针 */
  PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

  /** 末端执行器运动学 */
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  
  /** 参考轨迹管理器 */
  const ReferenceManager* referenceManagerPtr_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
