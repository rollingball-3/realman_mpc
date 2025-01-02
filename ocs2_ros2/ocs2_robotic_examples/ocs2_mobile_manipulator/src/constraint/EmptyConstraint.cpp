#include <ocs2_mobile_manipulator/constraint/EmptyConstraint.h>

namespace ocs2 {
namespace mobile_manipulator {

EmptyConstraint::EmptyConstraint(const PinocchioStateInputMapping<scalar_t>& mapping)
    : StateConstraint(ConstraintOrder::Linear),
    mappingPtr_(mapping.clone()) {}

EmptyConstraint::EmptyConstraint(const EmptyConstraint& rhs)
    : StateConstraint(rhs), 
      mappingPtr_(rhs.mappingPtr_->clone()) {}

vector_t EmptyConstraint::getValue(scalar_t time, const vector_t& state, 
                                 const PreComputation& preComputation) const {
    // 返回一个空向量,表示没有约束
    return vector_t::Zero(getNumConstraints(time));
}

size_t EmptyConstraint::getNumConstraints(scalar_t time) const {
    // 返回约束的数量,这里设为0表示没有约束
    return 8;
}

VectorFunctionLinearApproximation EmptyConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
    // 返回线性近似
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 休眠1000毫秒

    VectorFunctionLinearApproximation approximation;
    const size_t numConstraints = getNumConstraints(time);
    const size_t stateDim = state.size();
    
    approximation.f = vector_t::Zero(numConstraints);
    approximation.dfdx = matrix_t::Zero(numConstraints, stateDim);
    
    return approximation;
}

VectorFunctionQuadraticApproximation EmptyConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
    // 返回二次近似
    VectorFunctionQuadraticApproximation approximation;
    const size_t numConstraints = getNumConstraints(time);
    const size_t stateDim = state.size();
    
    approximation.f = vector_t::Zero(numConstraints);
    approximation.dfdx = matrix_t::Zero(numConstraints, stateDim);
    approximation.dfdxx.resize(numConstraints);
    
    // 初始化二阶导数为零矩阵
    for (size_t i = 0; i < numConstraints; i++) {
        approximation.dfdxx[i] = matrix_t::Zero(stateDim, stateDim);
    }
    
    return approximation;
}

}  // namespace mobile_manipulator
}  // namespace ocs2