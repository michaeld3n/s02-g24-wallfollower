// MTRX3760 2026 Project 2: Package Deal
// File: wall_follower_node.cpp
// Author(s): OpenAI Codex (AI-assisted implementation)
//
// Connects the Project 1-derived controller to ROS, with start/stop and state output.
// ROS lifecycle pattern adapted from Project 1; original attribution retained below.
// Copyright 2019 ROBOTIS CO., LTD.
// Original authors: Taehun Lim (Darby), Ryan Shim.
// Licensed under the Apache License, Version 2.0; see LICENSE and NOTICE.

#include "wall_follower/wall_follower_node.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <cmath>
#include <functional>

namespace wall_follower
{
WallFollowerNode::WallFollowerNode(const rclcpp::NodeOptions& options)
    : Node("wall_follower", options),
      mController(ReadSettings()),
      mEnabled(ReadBool("start_enabled", false)),
      mStamped(ReadBool("use_stamped_cmd_vel", true)),
      mScanTimeout(ReadDouble("scan_timeout", 1.0))
{
    if (mStamped)
    {
        mStampedVelocity = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    }
    else
    {
        mVelocity = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }
    mState = create_publisher<std_msgs::msg::String>(
        "state", rclcpp::QoS(1).reliable().transient_local());
    mScanSubscription = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&WallFollowerNode::OnScan, this, std::placeholders::_1));
    mEnableService = create_service<std_srvs::srv::SetBool>(
        "set_enabled", std::bind(&WallFollowerNode::SetEnabled, this,
                                std::placeholders::_1, std::placeholders::_2));
    mTimer = create_wall_timer(std::chrono::milliseconds(UpdateMilliseconds),
                              std::bind(&WallFollowerNode::Update, this));
    RCLCPP_INFO(get_logger(), "Right-wall follower ready; use set_enabled to start/stop");
}

double WallFollowerNode::ReadDouble(const std::string& name, double value)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<double>(name, value, descriptor);
}

bool WallFollowerNode::ReadBool(const std::string& name, bool value)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<bool>(name, value, descriptor);
}

Controller::Settings WallFollowerNode::ReadSettings()
{
    Controller::Settings settings;
    settings.WallDistance = ReadDouble("wall_distance", settings.WallDistance);
    settings.ForwardSpeed = ReadDouble("forward_speed", settings.ForwardSpeed);
    settings.TurnSpeed = ReadDouble("turn_speed", settings.TurnSpeed);
    settings.FrontStopDistance = ReadDouble("front_stop_distance", settings.FrontStopDistance);
    settings.FrontResumeDistance = ReadDouble("front_resume_distance", settings.FrontResumeDistance);
    settings.WallLostDistance = ReadDouble("wall_lost_distance", settings.WallLostDistance);
    settings.DistanceGain = ReadDouble("distance_gain", settings.DistanceGain);
    settings.HeadingGain = ReadDouble("heading_gain", settings.HeadingGain);
    return settings;
}

void WallFollowerNode::OnScan(sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
    mScan.Update(message->ranges, message->angle_min, message->angle_increment,
                 message->range_min, message->range_max);
    mReceipt = std::chrono::steady_clock::now();
    mHaveScan = true;
    // Deliberately ignore header timestamps: paused/reset simulation clocks are valid.
}

void WallFollowerNode::SetEnabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data && (!mController.HasValidSettings()
        || !std::isfinite(mScanTimeout) || mScanTimeout <= 0.0))
    {
        response->success = false;
        response->message = "Invalid configuration; correct startup parameters and restart";
        return;
    }
    if (mEnabled != request->data)
    {
        mController.Reset();
        mHaveScan = false;
    }
    mEnabled = request->data;
    response->success = true;
    response->message = mEnabled ? "Enabled" : "Disabled; zero velocity sent";
    Update();
}

void WallFollowerNode::Update()
{
    Controller::Command command;
    std::string state;
    if (!mController.HasValidSettings() || !std::isfinite(mScanTimeout) || mScanTimeout <= 0.0)
    {
        state = "invalid_configuration";
    }
    else if (!mEnabled)
    {
        state = "disabled";
    }
    else if (!mHaveScan || std::chrono::duration<double>(
                 std::chrono::steady_clock::now() - mReceipt).count() > mScanTimeout)
    {
        state = "waiting_for_scan";
        mController.Reset();
    }
    else
    {
        command = mController.CalculateCommand(mScan);
        state = mController.StateName();
    }
    Publish(command, state);
}

void WallFollowerNode::Publish(const Controller::Command& command, const std::string& state)
{
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = command.Linear;
    velocity.angular.z = command.Angular;
    if (mStamped)
    {
        geometry_msgs::msg::TwistStamped message;
        message.header.stamp = now();
        message.header.frame_id = "base_link";
        message.twist = velocity;
        mStampedVelocity->publish(message);
    }
    else
    {
        mVelocity->publish(velocity);
    }
    std_msgs::msg::String message;
    message.data = state;
    mState->publish(message);
}

void WallFollowerNode::Run()
{
    rclcpp::ExecutorOptions options;
    options.context = get_node_base_interface()->get_context();
    rclcpp::executors::SingleThreadedExecutor executor(options);
    executor.add_node(get_node_base_interface());
    rclcpp::WallRate poll(ExecutorFrequency);
    while (rclcpp::ok() && rclcpp::ok(options.context))
    {
        executor.spin_some();
        poll.sleep();
    }
    mTimer->cancel();
    if (rclcpp::ok(options.context))
    {
        Publish(Controller::Command{}, "disabled");
        if (mStamped)
        {
            mStampedVelocity->wait_for_all_acked(std::chrono::milliseconds(200));
        }
        else
        {
            mVelocity->wait_for_all_acked(std::chrono::milliseconds(200));
        }
    }
    executor.remove_node(get_node_base_interface());
}
}
