/*
EsdfClientInterface.cpp
written by: Yufei Lei
*/
#include "ocs2_mobile_manipulator/EsdfClientInterface.h"
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace ocs2{
    namespace mobile_manipulator{
    

EsdfClientInterface::EsdfClientInterface(const std::string& node_name, const std::string& service_name)
    : Node(node_name) {
    client_ = this->create_client<nvblox_msgs::srv::VoxelEsdfAndGradients>(service_name);
    
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for service '%s' to become available...", service_name.c_str());
}
    RCLCPP_INFO(this->get_logger(), "Service '%s' is now available.", service_name.c_str());
}

nvblox_msgs::srv::VoxelEsdfAndGradients::Response EsdfClientInterface::callEsdfService(std::vector<Eigen::Vector3d>& link_positions){
    if (!client_->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "Service is not ready.");
        return nvblox_msgs::srv::VoxelEsdfAndGradients::Response();
    }
    auto request = std::make_shared<nvblox_msgs::srv::VoxelEsdfAndGradients::Request>();
    // request->aabb_min_m.x = point(0);
    // request->aabb_min_m.y = point(1);
    // request->aabb_min_m.z = point(2);
    // request->aabb_size_m.x = direction(0);
    // request->aabb_size_m.y = direction(1);
    // request->aabb_size_m.z = direction(2);

    request->link_positions = link_positions;

    auto future = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS) {

        auto response = future.get();

        if (response) {
            // RCLCPP_INFO(this->get_logger(), "ESDF service call succeeded.");
            // RCLCPP_INFO(this->get_logger(), "Voxel size: %f", response->voxel_size.data);
            //RCLCPP_INFO(this->get_logger(), "ESDF data: %f", response->esdf_and_gradients.data[0]);

            nvblox_msgs::srv::VoxelEsdfAndGradients::Response esdf_response_ = *response;
            has_esdf_response_ = true;
            return esdf_response_;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Empty response from ESDF service.");
            return nvblox_msgs::srv::VoxelEsdfAndGradients::Response();
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call ESDF service.");
        return nvblox_msgs::srv::VoxelEsdfAndGradients::Response();
    }
}

// void EsdfClientInterface::setPoint(const Eigen::Vector3d& point) {
//     point_ = point;
//     RCLCPP_INFO(this->get_logger(), "Updated point: [%f, %f, %f]", point_.x(), point_.y(), point_.z());
// }


// void EsdfClientInterface::setDirection(const Eigen::Vector3d& direction) {
//     direction_ = direction;
//     RCLCPP_INFO(this->get_logger(), "Updated direction: [%f, %f, %f]", direction_.x(), direction_.y(), direction_.z());
// }

std::vector<float> EsdfClientInterface::getEsdf(std::vector<Eigen::Vector3d>& link_positions){
    // setPoint(point);
    // setDirection(direction);
    nvblox_msgs::srv::VoxelEsdfAndGradients::Response esdf_response_ = callEsdfService(link_positions);
    if (has_esdf_response_ && esdf_response_.valid) {
        return esdf_response_.esdf_values.data;
    } else {
        RCLCPP_ERROR(this->get_logger(), "No ESDF response available.");
        return std::vector<float>();
    }
}


}  // namespace mobile_manipulator
}  // namespace ocs2