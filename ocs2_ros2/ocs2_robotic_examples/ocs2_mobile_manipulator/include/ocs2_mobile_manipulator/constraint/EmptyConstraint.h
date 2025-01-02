/*
Empty Constraint for testing purposes
inherited from the StateConstraint class
*/

#pragma once

#include <memory>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <Eigen/Dense>

namespace ocs2 {
namespace mobile_manipulator {

class EmptyConstraint : public StateConstraint {
 public:
  /**
   * Constructor
   * 
   * @param [in] mapping: The pinocchio mapping from pinocchio states to ocs2 states.
   */
  EmptyConstraint(const PinocchioStateInputMapping<scalar_t>& mapping);
  
  ~EmptyConstraint() override = default;

  EmptyConstraint* clone() const override { 
    return new EmptyConstraint(*mappingPtr_); 
  }

  vector_t getValue(scalar_t time, const vector_t& state, 
                   const PreComputation& preComputation) const override;

  size_t getNumConstraints(scalar_t time) const override;

  VectorFunctionLinearApproximation getLinearApproximation(
      scalar_t time, const vector_t& state,
      const PreComputation& preComputation) const override;

  VectorFunctionQuadraticApproximation getQuadraticApproximation(
      scalar_t time, const vector_t& state,
      const PreComputation& preComputation) const override;

 private:
  EmptyConstraint(const EmptyConstraint& rhs);

  std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2