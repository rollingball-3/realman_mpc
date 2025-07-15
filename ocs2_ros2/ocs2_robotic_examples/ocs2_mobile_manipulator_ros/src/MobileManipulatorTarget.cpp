/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

using namespace ocs2;

// 全局变量，用于TF监听
static std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
static std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
static rclcpp::Node::SharedPtr node_ptr_ = nullptr;

// 初始化TF监听器
void initializeTfListener(rclcpp::Node::SharedPtr node) {
  if (tf_buffer_ == nullptr) {
    node_ptr_ = node;
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 等待一段时间让TF缓存填充
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

// 通过TF获取当前末端执行器位姿
std::pair<Eigen::Vector3d, Eigen::Quaterniond> getCurrentEndEffectorPose() {
  if (tf_buffer_ == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("MobileManipulatorTarget"), 
                 "TF buffer not initialized!");
    return {Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
  }

  try {
    // 获取从world到end_effector_link的变换
    geometry_msgs::msg::TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform("world", "gripper_tip_link", 
                                  rclcpp::Time(0), 
                                  rclcpp::Duration::from_nanoseconds(1000000000)); // 1秒超时
    
    // 提取位置
    Eigen::Vector3d position(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z
    );
    
    // 提取旋转四元数
    Eigen::Quaterniond orientation(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z
    );
    
    return {position, orientation};
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(rclcpp::get_logger("MobileManipulatorTarget"),
                "Could not transform world to end_effector_link: %s", ex.what());
    
    // 返回默认值
    return {Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()};
  }
}

/**
 * Converts the pose of the interactive marker to TargetTrajectories.
 */
TargetTrajectories goalPoseToTargetTrajectories(
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
    const SystemObservation& observation) {

  // 构造目标状态（7维：3D位置 + 4D四元数）
  const vector_t targetState = (vector_t(7) << position, orientation.coeffs()).finished();

  // 获取当前末端执行器位姿
  const auto [currentEEPosition, currentEEOrientation] = getCurrentEndEffectorPose();
  const vector_t currentEEState = (vector_t(7) << currentEEPosition, currentEEOrientation.coeffs()).finished();

  // 计算到达时间 - 基于距离和平均速度
  constexpr scalar_t averageSpeed = 0.5;  // 可调参数
  const vector_t deltaPose = targetState.head<3>() - currentEEState.head<3>(); // 位置差
  const scalar_t distance = deltaPose.norm();
  
  // 设置最小时间，避免太快的运动
  const scalar_t minTime = 2.0;
  const scalar_t calculatedTime = distance / averageSpeed;
  const scalar_t targetTime = observation.time + std::max(minTime, calculatedTime);

  // 时间轨迹：从当前时间到目标时间
  const scalar_array_t timeTrajectory{observation.time, targetTime};

  // 状态轨迹：从当前末端执行器位姿到目标位姿
  const vector_array_t stateTrajectory{currentEEState, targetState};

  // 输入轨迹：起始输入和目标输入
  const vector_array_t inputTrajectory{
    vector_t::Zero(observation.input.size()), 
    vector_t::Zero(observation.input.size())
  };

  // 打印调试信息
  RCLCPP_INFO(rclcpp::get_logger("MobileManipulatorTarget"),
              "Current EE pose: [%.3f, %.3f, %.3f]", 
              currentEEPosition.x(), currentEEPosition.y(), currentEEPosition.z());
  RCLCPP_INFO(rclcpp::get_logger("MobileManipulatorTarget"),
              "Target EE pose: [%.3f, %.3f, %.3f]", 
              position.x(), position.y(), position.z());
  RCLCPP_INFO(rclcpp::get_logger("MobileManipulatorTarget"),
              "Distance: %.3f m, Time: %.3f s", distance, targetTime - observation.time);

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}
// TargetTrajectories goalPoseToTargetTrajectories(
//     const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
//     const SystemObservation& observation) {
//   // time trajectory
//   const scalar_array_t timeTrajectory{observation.time};
//   // state trajectory: 3 + 4 for desired position vector and orientation
//   // quaternion
//   const vector_t target =
//       (vector_t(7) << position, orientation.coeffs()).finished();
//   const vector_array_t stateTrajectory{target};
//   // input trajectory
//   const vector_array_t inputTrajectory{
//       vector_t::Zero(observation.input.size())};

//   return {timeTrajectory, stateTrajectory, inputTrajectory};
// }

int main(int argc, char* argv[]) {
  const std::string robotName = "mobile_manipulator";
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
      robotName + "_target",
      rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true));

  // 初始化TF监听器
  initializeTfListener(node);

  TargetTrajectoriesInteractiveMarker targetPoseCommand(
      node, robotName, &goalPoseToTargetTrajectories);
  targetPoseCommand.publishInteractiveMarker();

  // Successful exit
  return 0;
}