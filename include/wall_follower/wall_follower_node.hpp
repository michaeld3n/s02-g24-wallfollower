// The ROS interface for the wall follower
//
// This node owns the scan reader and controller. Its callbacks handle scans,
// start/stop requests and regular output updates. The publishers send the
// requested movement and current state to the rest of the ROS system.

// Copyright 2019 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0: https://www.apache.org/licenses/LICENSE-2.0

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
class WallFollowerNode : public rclcpp::Node
{
public:
    explicit WallFollowerNode(const rclcpp::NodeOptions& options);

    // Run the callbacks, then send zero velocity before shutting down.
    void Run();

private:
    Controller::Settings ReadSettings();
    double ReadDouble(const std::string& name, double value);
    bool ReadBool(const std::string& name, bool value);
    // Scans can arrive while stopped, but starting requires another scan.
    void OnScan(sensor_msgs::msg::LaserScan::ConstSharedPtr message);
    // Clear saved movement when start/stop changes and update the output immediately.
    void SetEnabled(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void Update();
    void Publish(const Controller::Command& command, const std::string& state);

    ScanReader mScan;
    Controller mController;
    bool mEnabled;
    // True after a scan arrives; cleared whenever start/stop changes.
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
