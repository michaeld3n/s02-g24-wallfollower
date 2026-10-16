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
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <chrono>
#include <memory>
#include <string>

namespace wall_follower
{
class WallFollowerNode : public rclcpp::Node
{
public:
    explicit WallFollowerNode(const rclcpp::NodeOptions& options);
    void Run();

private:
    Controller::Settings ReadSettings();
    double ReadDouble(const std::string& name, double value);
    bool ReadBool(const std::string& name, bool value);
    void OnScan(sensor_msgs::msg::LaserScan::ConstSharedPtr message);
    void SetEnabled(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void Update();
    void Publish(const Controller::Command& command, const std::string& state);

    ScanReader mScan;
    Controller mController;
    bool mEnabled;
    const bool mStamped;
    const double mScanTimeout;
    bool mHaveScan = false;
    std::chrono::steady_clock::time_point mReceipt;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mScanSubscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mVelocity;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr mStampedVelocity;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mState;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mEnableService;
    rclcpp::TimerBase::SharedPtr mTimer;

    static constexpr int UpdateMilliseconds = 50;
    static constexpr double ExecutorFrequency = 100.0;
};
}
#endif
