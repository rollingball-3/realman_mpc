/*
Esdf client interface for the realman robot arm
service: /nvblox_node/get_esdf_and_gradient
service type: nvblox_msgs::srv::GetEsdfAndGradients
written by: Yufei Lei
*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <nvblox_msgs/srv/voxel_esdf_and_gradients.hpp>

namespace ocs2{
    namespace mobile_manipulator{

class EsdfClientInterface : public rclcpp::Node{
    public:
    EsdfClientInterface(const std::string& node_name, const std::string& service_name);
    ~EsdfClientInterface() = default;

    nvblox_msgs::srv::VoxelEsdfAndGradients::Response callEsdfService(std::vector<Eigen::Vector3d>& link_positions);

    struct EsdfResponse{
        std::vector<float> esdf_values;
        std::vector<Eigen::Vector3d> gradients;
    };


    // Setters
    // void setPoint(const Eigen::Vector3d& point);
    // void setDirection(const Eigen::Vector3d& direction);

    //return the esdf value
    EsdfResponse getEsdf(std::vector<Eigen::Vector3d>& link_positions);

    //return the gradient
    //Eigen::Vector3d getGradient(Eigen::Vector3d& point, Eigen::Vector3d& direction);

    private:
    rclcpp::Client<nvblox_msgs::srv::VoxelEsdfAndGradients>::SharedPtr client_;
    // Eigen::Vector3d point_;
    // Eigen::Vector3d direction_;
    //nvblox_msgs::srv::EsdfAndGradients::Response esdf_response_;
    std::mutex service_mutex_;  // 添加互斥锁
    bool has_esdf_response_{false};

    

};

}
}