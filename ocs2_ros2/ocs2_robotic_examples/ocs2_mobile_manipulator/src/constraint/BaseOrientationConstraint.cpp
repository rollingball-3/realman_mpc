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

#include <ocs2_mobile_manipulator/constraint/BaseOrientationConstraint.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
BaseOrientationConstraint::BaseOrientationConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                   const ReferenceManager& referenceManager)
    : StateConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      referenceManagerPtr_(&referenceManager) {
  
  if (endEffectorKinematics.getIds().size() != 1) {
    throw std::runtime_error("[BaseOrientationConstraint] endEffectorKinematics has wrong number of end effector IDs.");
  }
  pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t BaseOrientationConstraint::getValue(scalar_t time, const vector_t& state, 
                                           const PreComputation& preComputation) const {
  // 设置运动学计算所需的接口
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  // 获取目标位置
  const auto targetPosition = interpolateEndEffectorTargetPosition(time);
  
  // 计算底盘到目标的距离
  const scalar_t baseX = state(0);
  const scalar_t baseY = state(1);
  const scalar_t deltaX = targetPosition(0) - baseX;
  const scalar_t deltaY = targetPosition(1) - baseY;
  const scalar_t distance = std::sqrt(deltaX*deltaX + deltaY*deltaY);
  
  // 当距离非常小时不考虑朝向损失
  const scalar_t minDistance = 0.1;  // 最小距离阈值
  
  // 返回单值约束
  vector_t constraint(1);
  if (distance < minDistance) {
    constraint(0) = 0.0;  // 距离很小时，返回零损失
  } else {
    // 计算底盘朝向与目标方向之间的角度差
    scalar_t orientationError = computeBaseOrientationError(state, targetPosition);
    constraint(0) = orientationError;
  }
  
  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation BaseOrientationConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  
  // 设置运动学计算所需的接口
  if (pinocchioEEKinPtr_ != nullptr) {
    const auto& preCompMM = cast<MobileManipulatorPreComputation>(preComputation);
    pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
  }

  // 获取目标位置
  const auto targetPosition = interpolateEndEffectorTargetPosition(time);
  
  // 计算底盘朝向误差
  scalar_t orientationError = computeBaseOrientationError(state, targetPosition);
  
  // 计算线性近似
  auto approximation = VectorFunctionLinearApproximation(1, state.rows(), 0);
  approximation.f(0) = orientationError;
  
  // 对于轮式移动机械臂，状态向量前三个元素是(x,y,theta)
  const int baseXIdx = 0;
  const int baseYIdx = 1;
  const int baseThetaIdx = 2;
  
  // 获取底盘位置和朝向
  const scalar_t baseX = state(baseXIdx);
  const scalar_t baseY = state(baseYIdx);
  
  // 计算底盘到目标的向量
  const scalar_t deltaX = targetPosition(0) - baseX;
  const scalar_t deltaY = targetPosition(1) - baseY;
  const scalar_t distance = std::sqrt(deltaX*deltaX + deltaY*deltaY);
  
  const scalar_t minDistance = 0.1;  // 最小距离阈值
  const scalar_t safeDistance = 0.15;  // 安全距离阈值

  if (distance < minDistance) {
    // 距离太小，只保留朝向优化，不调整位置
    approximation.dfdx(0, baseXIdx) = 0.0;
    approximation.dfdx(0, baseYIdx) = 0.0;
    approximation.dfdx(0, baseThetaIdx) = -1.0;
  } else if (distance < safeDistance) {
    // 距离在安全阈值内，使用平滑过渡
    scalar_t alpha = (distance - minDistance) / (safeDistance - minDistance);  // 0到1的平滑因子
    scalar_t safeGradient = 1.0 / (safeDistance * safeDistance);
    scalar_t smoothingFactor = safeGradient * std::pow(alpha, 2);  // 二次平滑过渡
    
    approximation.dfdx(0, baseXIdx) = deltaY * smoothingFactor;
    approximation.dfdx(0, baseYIdx) = -deltaX * smoothingFactor;
    approximation.dfdx(0, baseThetaIdx) = -1.0;
  } else {
    // 正常计算梯度
    approximation.dfdx(0, baseXIdx) = deltaY / (distance * distance);
    approximation.dfdx(0, baseYIdx) = -deltaX / (distance * distance);
    approximation.dfdx(0, baseThetaIdx) = -1.0;
  }
  
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
BaseOrientationConstraint::vector3_t BaseOrientationConstraint::interpolateEndEffectorTargetPosition(scalar_t time) const {
  const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
  const auto& timeTrajectory = targetTrajectories.timeTrajectory;
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;

  vector3_t position;

  if (stateTrajectory.size() > 1) {
    // 正常插值情况
    int index;
    scalar_t alpha;
    std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

    const auto& lhs = stateTrajectory[index];
    const auto& rhs = stateTrajectory[index + 1];

    // 目标状态向量中的末端执行器位置应该是前3个元素
    position = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
  } else {  // stateTrajectory.size() == 1
    position = stateTrajectory.front().head<3>();
  }

  return position;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t BaseOrientationConstraint::computeBaseOrientationError(
    const vector_t& state, const vector3_t& targetPosition) const {
  // 对于轮式移动机械臂，状态向量前三个元素是(x,y,theta)
  const scalar_t baseX = state(0);
  const scalar_t baseY = state(1);
  const scalar_t baseTheta = state(2);  // 底盘朝向角
  
  // 计算底盘到目标的向量
  const scalar_t deltaX = targetPosition(0) - baseX;
  const scalar_t deltaY = targetPosition(1) - baseY;
  const scalar_t distance = std::sqrt(deltaX*deltaX + deltaY*deltaY);
  
  // 当距离非常小时不考虑朝向损失
  const scalar_t minDistance = 0.1;  // 最小距离阈值
  if (distance < minDistance) {
    return 0.0;  // 返回零损失，表示底盘朝向不重要
  }
  
  // 计算目标方向角
  scalar_t targetTheta = std::atan2(deltaY, deltaX);
  
  // 计算角度差（考虑角度循环，将结果限制在[-π,π]范围内）
  scalar_t thetaError = targetTheta - baseTheta;
  while (thetaError > M_PI) thetaError -= 2.0 * M_PI;
  while (thetaError < -M_PI) thetaError += 2.0 * M_PI;
  
  return thetaError;
}

}  // namespace mobile_manipulator
}  // namespace ocs2