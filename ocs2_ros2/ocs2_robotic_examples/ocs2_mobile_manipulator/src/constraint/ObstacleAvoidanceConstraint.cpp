/*
ObstacleAvoidanceConstraint.cpp

written by: Yufei Lei
*/
#include <ocs2_mobile_manipulator/constraint/ObstacleAvoidanceConstraint.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace ocs2{
    namespace mobile_manipulator{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ObstacleAvoidanceConstraint::ObstacleAvoidanceConstraint(
    const PinocchioStateInputMapping<scalar_t>& mapping, 
    const PinocchioSphereInterface& pinocchioSphereInterface, 
    EsdfClientInterface& esdfClientInterface)
    : StateConstraint(ConstraintOrder::Linear),
      pinocchioSphereInterface_(pinocchioSphereInterface), 
      esdfClientInterface_(esdfClientInterface),
      mappingPtr_(mapping.clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
ObstacleAvoidanceConstraint::ObstacleAvoidanceConstraint(const ObstacleAvoidanceConstraint& rhs)
    : StateConstraint(rhs), pinocchioSphereInterface_(rhs.pinocchioSphereInterface_), esdfClientInterface_(rhs.esdfClientInterface_),
    mappingPtr_(rhs.mappingPtr_->clone()) {}

vector_t ObstacleAvoidanceConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const{
    //get the state of the robot
    const auto& preComp = cast<MobileManipulatorPreComputation>(preComputation);
    const auto& pinocchioInterface_ = preComp.getPinocchioInterface();
    
    auto* mutableThis = const_cast<ObstacleAvoidanceConstraint*>(this);
    //get the value of the esdf
    return mutableThis->getEsdfConstraintValue(pinocchioInterface_, pinocchioSphereInterface_, esdfClientInterface_).first;
    //return vector_t::Zero(getNumConstraints(time));
}

std::pair<vector_t, std::vector<Eigen::Vector3d>> ObstacleAvoidanceConstraint::getEsdfConstraintValue(const PinocchioInterface &pinocchioInterface, const PinocchioSphereInterface &pinocchioSphereInterface, 
                                                                EsdfClientInterface &esdfClientInterface){
    // Get the world coordinates of the 7 robot arm links
    const auto& model = pinocchioInterface.getModel();
    const auto& data = pinocchioInterface.getData();
    
    // Create vector to store link positions
    std::vector<Eigen::Vector3d> linkPositions;
    linkPositions.reserve(7);
    
    // Loop through the 7 joints to get their world positions
    for(size_t i = 1; i <= 7; ++i) {
        // Get the transformation matrix for each joint
        const auto& transform = data.oMi[i];
        // Extract the translation part (world position)
        Eigen::Vector3d position = transform.translation();
        linkPositions.push_back(position);
        //std::cout << "Link position: " << position.transpose() << std::endl;
        }
    // get the sphere centers
    const auto& sphereCenters = pinocchioSphereInterface.computeSphereCentersInWorldFrame(pinocchioInterface);

    std::vector<Eigen::Vector3d> adjustedSphereCenters = sphereCenters;

    // SphereCenters are relative to the robot base frame, so we need to convert them to the world frame using the link positions
    for (size_t i = 0; i < adjustedSphereCenters.size(); ++i) {
        adjustedSphereCenters[i] = linkPositions[i] + sphereCenters[i];
        //std::cout << "Adjusted sphere center: " << adjustedSphereCenters[i].transpose() << std::endl;
    }

    // get the sphere radiis
    const auto& sphereRadii = pinocchioSphereInterface.getSphereRadii();

    // RCLCPP_INFO(this->get_logger(), "sphereRadii size: %zu", sphereRadii.size());
    
    // for (const auto& radius : sphereRadii) {
    //     std::cout << "sphere radius: " << radius << std::endl;
    // }

    std::vector<float> esdfValue;
    std::vector<Eigen::Vector3d> gradients;

    // //call the esdf client to get the esdf value
    // for (size_t i = 0; i < adjustedSphereCenters.size(); ++i) {
    //     const auto& esdf = esdfClientInterface.getEsdf(adjustedSphereCenters[i]);
    //     // esdf: 1D vector, size = number of voxels in the aabb
    //     esdfValue[i] = esdf[0];
    // }
    EsdfClientInterface::EsdfResponse esdfResponse = esdfClientInterface.getEsdf(adjustedSphereCenters);
    esdfValue = esdfResponse.esdf_values;
    gradients = esdfResponse.gradients;

    vector_t constraintValue = vector_t::Zero(esdfValue.size());
    // compute the constraint value
    for (size_t i = 0; i < esdfValue.size(); ++i) {
        constraintValue[i] = fabs(esdfValue[i]) - sphereRadii[i];
        //RCLCPP_INFO(this->get_logger(), "Constraint value: %f, radius: %f", constraintValue[i], sphereRadii[i]);
    }
    //std::cout << "constraintValue size: " << constraintValue.size() << std::endl;
    return std::make_pair(constraintValue, gradients);
}

size_t ObstacleAvoidanceConstraint::getNumConstraints(scalar_t time) const{
    return 7;
}

/******************************************************************************************************/
/******************************************************************************************************/
//linear approximation dfdq dfdu
VectorFunctionLinearApproximation ObstacleAvoidanceConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                            const PreComputation& preComputation) const {   
    // 1. 获取预计算数据
    const auto& preComp = cast<MobileManipulatorPreComputation>(preComputation);
    const auto& pinocchioInterface_ = preComp.getPinocchioInterface();
    mappingPtr_->setPinocchioInterface(pinocchioInterface_);

    VectorFunctionLinearApproximation constraint;
    matrix_t dfdq, dfdv;
    
    // 2. 计算约束值和梯度
    auto* mutableThis = const_cast<ObstacleAvoidanceConstraint*>(this);
    auto [constraintValues, gradients] = mutableThis->getEsdfConstraintValue(pinocchioInterface_, 
                                                                           pinocchioSphereInterface_, 
                                                                           esdfClientInterface_);
    constraint.f = constraintValues;

    // 3. 计算雅可比矩阵
    // 获取机器人的雅可比矩阵
    const auto& model = pinocchioInterface_.getModel();
    const auto& data = pinocchioInterface_.getData();
    
    // 为每个球体计算雅可比矩阵
    const auto& spherePositions = pinocchioSphereInterface_.computeSphereCentersInWorldFrame(pinocchioInterface_);
    dfdq.setZero(spherePositions.size(), model.nq);
    
    for (size_t i = 0; i < spherePositions.size(); ++i) {
        // 获取球体所在关节的雅可比矩阵
        matrix_t sphereJacobian = matrix_t::Zero(6, model.nv);
        pinocchio::getJointJacobian(model, data, i+1, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, sphereJacobian);
        
        // 将ESDF梯度与关节雅可比相乘
        dfdq.row(i) = gradients[i].transpose() * sphereJacobian.topRows(3);
    }
    
    // 4. 映射到OCS2状态空间
    dfdv.setZero(dfdq.rows(), dfdq.cols());
    std::tie(constraint.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, dfdq, dfdv);
    
    return constraint;
}

VectorFunctionQuadraticApproximation ObstacleAvoidanceConstraint::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                            const PreComputation& preComputation) const {
    VectorFunctionQuadraticApproximation constraint;

    auto linearApprox = getLinearApproximation(time, state, preComputation);
    constraint.f = std::move(linearApprox.f);
    constraint.dfdx = std::move(linearApprox.dfdx);
    constraint.dfdu = std::move(linearApprox.dfdu);

    const auto inputDim_ = 7;
    // TODO: 二阶导数设置为0
    constraint.dfdxx.assign(constraint.f.size(), matrix_t::Zero(state.size(), state.size()));
    constraint.dfdux.assign(constraint.f.size(), matrix_t::Zero(inputDim_, state.size()));
    constraint.dfduu.assign(constraint.f.size(), matrix_t::Zero(inputDim_, inputDim_));

    return constraint;
}

// const PinocchioInterface& ObstacleAvoidanceConstraint::getPinocchioInterface(const PreComputation& preComputation) const{
//     return cast<MobileManipulatorPreComputation>(preComputation).getPinocchioInterface();
// }




}//namespace mobile_manipulator
}//namespace ocs2