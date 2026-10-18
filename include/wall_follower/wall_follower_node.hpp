// MTRX3760 2026 Project 2: Package Deal
// File: wall_follower_node.hpp
// Author(s): OpenAI Codex (AI-assisted implementation)
//
// ROS adapter owning the scan reader, steering controller and runtime interfaces.
// Project 1 lineage: Copyright 2019 ROBOTIS CO., LTD.
// Original authors: Taehun Lim (Darby), Ryan Shim.
// Licensed under the Apache License, Version 2.0; see LICENSE and NOTICE.

#ifndef WALL_FOLLOWER_NODE_HPP
#define WALL_FOLLOWER_NODE_HPP

#include "wall_follower/controller.hpp"
#include "wall_follower/scan_reader.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <memory>
#include <string>

namespace wall_follower
{
// Connects scan, velocity and state topics with a runtime enable service.
class WallFollowerNode : public rclcpp::Node
{
public:
    explicit WallFollowerNode(const rclcpp::NodeOptions& options);

    // Dispatch callbacks until shutdown, then request a final zero command.
    void Run();

private:
    Controller::Settings ReadSettings();
    double ReadDouble(const std::string& name, double value);
    bool ReadBool(const std::string& name, bool value);
    // A scan can arrive while disabled; only post-enable input may drive motion.
    void OnScan(sensor_msgs::msg::LaserScan::ConstSharedPtr message);
    // Reset retained motion on value changes and publish the new state immediately.
    void SetEnabled(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void Update();
    void Publish(const Controller::Command& command, const std::string& state);

    ScanReader mScan;
    Controller mController;
    bool mEnabled;
    // At least one scan has arrived since the latest enable transition.
    bool mHaveScan = false;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mScanSubscription;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mVelocity;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mState;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mEnableService;
    rclcpp::TimerBase::SharedPtr mTimer;

    static constexpr int VelocityQueueDepth = 10;
    static constexpr int UpdateMilliseconds = 50;
    static constexpr int ShutdownAckWaitMilliseconds = 200;
    static constexpr double ExecutorFrequency = 100.0;
};
}
#endif
