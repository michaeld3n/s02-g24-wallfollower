// Connecting the wall follower to ROS
//
// The node receives laser scans, passes them to the reader and publishes
// stamped velocity commands and state messages. A service enables or stops
// movement, while a timer updates the output every 50 ms.

// Copyright 2019 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0: https://www.apache.org/licenses/LICENSE-2.0

#include "wall_follower/wall_follower_node.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <chrono>
#include <functional>

namespace wall_follower
{
WallFollowerNode::WallFollowerNode(const rclcpp::NodeOptions& options)
    : Node("wall_follower", options),
      mController(ReadSettings()),
      mEnabled(ReadBool("start_enabled", false))
{
    mVelocity = create_publisher<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", VelocityQueueDepth);
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
    // Use the scan contents without checking its timestamp.
    mScan.Update(message->ranges, message->angle_min, message->angle_increment,
                 message->range_min, message->range_max);
    mHaveScan = true;
}

void WallFollowerNode::SetEnabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data && !mController.HasValidSettings())
    {
        response->success = false;
        response->message = "Invalid configuration; correct startup parameters and restart";
    }
    else
    {
        if (mEnabled != request->data)
        {
            // Forget old movement and wait for another scan after start/stop changes.
            mController.Reset();
            mHaveScan = false;
        }
        mEnabled = request->data;
        response->success = true;
        response->message = mEnabled ? "Enabled" : "Disabled; zero velocity sent";
        Update();
    }
}

void WallFollowerNode::Update()
{
    Controller::Command command;
    std::string state;
    if (!mController.HasValidSettings())
    {
        state = "invalid_configuration";
    }
    else if (!mEnabled)
    {
        state = "disabled";
    }
    else if (!mHaveScan)
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
    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.stamp = now();
    velocity.header.frame_id = "base_link";
    velocity.twist.linear.x = command.Linear;
    velocity.twist.angular.z = command.Angular;
    mVelocity->publish(velocity);
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
        mVelocity->wait_for_all_acked(
            std::chrono::milliseconds(ShutdownAckWaitMilliseconds));
    }
    executor.remove_node(get_node_base_interface());
}
}
