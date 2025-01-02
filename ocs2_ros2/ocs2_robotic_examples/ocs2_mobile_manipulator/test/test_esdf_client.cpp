#include "ocs2_mobile_manipulator/EsdfClientInterface.h"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <thread>
#include <random>
#include <chrono>

using namespace ocs2::mobile_manipulator;

std::atomic<bool> g_running{true};

void signalHandler(int signum) {
    g_running = false;
    rclcpp::shutdown();
    exit(signum);
}

std::vector<Eigen::Vector3d> generateRandomTestPositions(int num_points) {
    std::vector<Eigen::Vector3d> positions;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0);  // 在-1.0到1.0米范围内随机
    
    for (int i = 0; i < num_points; ++i) {
        positions.push_back(Eigen::Vector3d(dis(gen), dis(gen), std::abs(dis(gen))));
    }
    return positions;
}

// 修改查询函数，不在线程中执行spin
void queryEsdfThread(const std::shared_ptr<EsdfClientInterface>& esdf_client, int thread_id) {
    try {
        while (g_running && rclcpp::ok()) {
            try {
                auto test_positions = generateRandomTestPositions(5);
                
                auto response = esdf_client->getEsdf(test_positions);
                
                if (!g_running) break;

                RCLCPP_INFO(esdf_client->get_logger(), "线程 %d 收到ESDF响应:", thread_id);
                for (size_t i = 0; i < response.esdf_values.size(); ++i) {
                    RCLCPP_INFO(esdf_client->get_logger(), 
                        "线程 %d - 位置 %zu: ESDF值 = %f, 梯度 = [%f, %f, %f]",
                        thread_id,
                        i,
                        response.esdf_values[i],
                        response.gradients[i].x(),
                        response.gradients[i].y(),
                        response.gradients[i].z()
                    );
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100 + rand() % 400));
            } catch (const std::exception& e) {
                RCLCPP_ERROR(esdf_client->get_logger(), "线程 %d 发生错误: %s", thread_id, e.what());
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(esdf_client->get_logger(), "线程 %d 发生致命错误: %s", thread_id, e.what());
    }
}

int main(int argc, char** argv) {
    try {
        signal(SIGINT, signalHandler);
        
        rclcpp::init(argc, argv);
        
        auto esdf_client = std::make_shared<EsdfClientInterface>(
            "test_esdf_client", 
            "/nvblox_node/get_voxel_esdf_and_gradient"
        );

        // 创建查询线程
        std::vector<std::thread> query_threads;
        const int num_threads = 4;
        
        for (int i = 0; i < num_threads; ++i) {
            query_threads.push_back(std::thread(queryEsdfThread, esdf_client, i));
        }

        // 等待Ctrl+C
        while (rclcpp::ok() && g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 设置退出标志
        g_running = false;


        // 等待所有查询线程结束
        for (auto& thread : query_threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }

        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "主线程发生错误: " << e.what() << std::endl;
        return 1;
    }
}