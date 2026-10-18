// Starting and stopping the wall follower
//
// This program creates the ROS node and runs its callbacks. A separate ROS
// context lets the node send a final zero velocity command when Ctrl+C is
// pressed, before its publishers are shut down.

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
