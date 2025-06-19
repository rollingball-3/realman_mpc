/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>

#include <ocs2_mobile_manipulator/EsdfClientInterface.h>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ocs2 {
namespace mobile_manipulator {

/**
 * Mobile Manipulator Robot Interface class
 */
class MobileManipulatorInterface final : public RobotInterface {
 public:
  /**
   * Constructor
   *
   * @note Creates directory for generated library into if it does not exist.
   * @throw Invalid argument error if input task file or urdf file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
   * @param [in] urdfFile: The absolute path to the URDF file for the robot.
   */
  MobileManipulatorInterface(const std::string& taskFile, const std::string& libraryFolder, const std::string& urdfFile);

  const vector_t& getInitialState() { return initialState_; }

  ddp::Settings& ddpSettings() { return ddpSettings_; }

  mpc::Settings& mpcSettings() { return mpcSettings_; }

  const OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }

  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  const Initializer& getInitializer() const override { return *initializerPtr_; }

  const RolloutBase& getRollout() const { return *rolloutPtr_; }

  const PinocchioInterface& getPinocchioInterface() const { return *pinocchioInterfacePtr_; }

  // TODO add new sphere interface
  const PinocchioSphereInterface& getPinocchioSphereInterface() const { return *pinocchioSphereInterfacePtr_; }
  
  const ManipulatorModelInfo& getManipulatorModelInfo() const { return manipulatorModelInfo_; }

  const EsdfClientInterface& getEsdfClientInterface() const { return *esdfClientInterfacePtr_; }

  // Publish Sphere Approximation
  void publishSphereVisualization(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher);

 private:
  // TODO add new costs function define under folder constraint and cost
  std::unique_ptr<StateInputCost> getQuadraticInputCost(const std::string& taskFile);
  // 在private部分添加
  std::unique_ptr<StateCost> getBaseOrientationConstraint(const PinocchioInterface& pinocchioInterface, 
                                                        const std::string& taskFile,
                                                        const std::string& prefix, 
                                                        bool usePreComputation, 
                                                        const std::string& libraryFolder,
                                                        bool recompileLibraries);
  std::unique_ptr<StateCost> getEndEffectorConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                      const std::string& prefix, bool useCaching, const std::string& libraryFolder,
                                                      bool recompileLibraries);
  std::unique_ptr<StateCost> getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile,
                                                        const std::string& urdfFile, const std::string& prefix, bool useCaching,
                                                        const std::string& libraryFolder, bool recompileLibraries);
  std::unique_ptr<StateInputCost> getJointLimitSoftConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile);

  std::unique_ptr<StateCost> getObstacleAvoidanceConstraint(const PinocchioSphereInterface& pinocchioSphereInterface, EsdfClientInterface& esdfClientInterface, const std::string& taskFile,
                                                                 const std::string& prefix, bool useCaching, const std::string& libraryFolder,
                                                                 bool recompileLibraries);
  std::unique_ptr<StateCost> getEmptyConstraint(const PinocchioInterface& pinocchioInterface, const std::string& taskFile, const std::string& prefix,
                                                                 bool useCaching, const std::string& libraryFolder, bool recompileLibraries);
  
  PinocchioSphereInterface createPinocchioSphereInterface(const PinocchioInterface& pinocchioInterface, const std::string& taskFile, const std::string& prefix);

  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;

  OptimalControlProblem problem_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr_;

  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> initializerPtr_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;

  // TODO add new sphere interface
  std::unique_ptr<PinocchioSphereInterface> pinocchioSphereInterfacePtr_;

  // TODO add new esdf client interface
  std::unique_ptr<EsdfClientInterface> esdfClientInterfacePtr_;

  ManipulatorModelInfo manipulatorModelInfo_;

  // TODO set activate paramter 
  // TODO set control parmater
  vector_t initialState_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
