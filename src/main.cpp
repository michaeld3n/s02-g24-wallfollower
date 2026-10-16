// MTRX3760 2026 Project 2: Package Deal
// File: main.cpp
// Author(s): OpenAI Codex (AI-assisted adaptation)
//
// Uses Project 1's separate ROS context pattern to send zero velocity on Ctrl+C.

#include "wall_follower/wall_follower_node.hpp"
#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::InitOptions init;
    init.shutdown_on_signal = false;
    init.auto_initialize_logging(false);
    auto context = std::make_shared<rclcpp::Context>();
    context->init(argc, argv, init);
    rclcpp::NodeOptions options;
    options.context(context);
    auto node = std::make_shared<wall_follower::WallFollowerNode>(options);
    node->Run();
    context->shutdown("Wall follower stopped");
    rclcpp::shutdown();
    return 0;
}
