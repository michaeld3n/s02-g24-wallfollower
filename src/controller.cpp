// MTRX3760 2026 Project 2: Package Deal
// File: controller.cpp
// Author(s): Cody, Marzuq, Royce, Hajin
//
// The controller uses the front, right and front-right distances to follow a
// wall, search for one or turn away from an obstacle. Unusable scans keep the
// previous command. Different stop and resume distances prevent repeated
// switching near the front-obstacle threshold.

#include "wall_follower/controller.hpp"

#include <algorithm>
#include <cmath>

namespace wall_follower
{
Controller::Settings::Settings(double wall_distance, double forward_speed, double turn_speed,
                               double front_stop_distance, double front_resume_distance,
                               double wall_lost_distance, double distance_gain, double heading_gain)
    : mWallDistance(wall_distance),
      mForwardSpeed(forward_speed),
      mTurnSpeed(turn_speed),
      mFrontStopDistance(front_stop_distance),
      mFrontResumeDistance(front_resume_distance),
      mWallLostDistance(wall_lost_distance),
      mDistanceGain(distance_gain),
      mHeadingGain(heading_gain)
{
}

double Controller::Settings::WallDistance() const
{
    return mWallDistance;
}

double Controller::Settings::ForwardSpeed() const
{
    return mForwardSpeed;
}

double Controller::Settings::TurnSpeed() const
{
    return mTurnSpeed;
}

double Controller::Settings::FrontStopDistance() const
{
    return mFrontStopDistance;
}

double Controller::Settings::FrontResumeDistance() const
{
    return mFrontResumeDistance;
}

double Controller::Settings::WallLostDistance() const
{
    return mWallLostDistance;
}

double Controller::Settings::DistanceGain() const
{
    return mDistanceGain;
}

double Controller::Settings::HeadingGain() const
{
    return mHeadingGain;
}

bool Controller::Settings::IsValid() const
{
    return std::isfinite(mWallDistance) && std::isfinite(mForwardSpeed)
        && std::isfinite(mTurnSpeed) && std::isfinite(mFrontStopDistance)
        && std::isfinite(mFrontResumeDistance) && std::isfinite(mWallLostDistance)
        && std::isfinite(mDistanceGain) && std::isfinite(mHeadingGain)
        && mWallDistance > 0.0 && mForwardSpeed > 0.0 && mTurnSpeed > 0.0
        && mFrontStopDistance > 0.0 && mFrontResumeDistance > mFrontStopDistance
        && mWallLostDistance > mWallDistance && mDistanceGain > 0.0 && mHeadingGain >= 0.0;
}

Controller::Command::Command(double linear, double angular)
    : mLinear(linear), mAngular(angular)
{
}

double Controller::Command::Linear() const
{
    return mLinear;
}

double Controller::Command::Angular() const
{
    return mAngular;
}

Controller::Controller(const Settings& settings) : mSettings(settings)
{
    Reset();
}

bool Controller::HasValidSettings() const
{
    return mSettings.IsValid();
}

const char* Controller::StateName() const
{
    return mState;
}

void Controller::Reset()
{
    mLastCommand = Command{};
    mFrontBlocked = false;
    mState = HasValidSettings() ? "waiting_for_scan" : "invalid_configuration";
}

Controller::Command Controller::CalculateCommand(const ScanReader& scan)
{
    Command result;
    if (!HasValidSettings() || !scan.HasReceivedScan())
    {
        Reset();
    }
    else if (!scan.HasValidReadings())
    {
        // Keep the last command and remember whether an obstacle turn was in progress.
        mState = "invalid_scan";
        result = mLastCommand;
    }
    else
    {
        UpdateFrontBlocked(scan.FrontDistance());
        if (mFrontBlocked)
        {
            mState = "turning_left";
            result = Command(0.0, mSettings.TurnSpeed());
        }
        else if (scan.RightDistance() > mSettings.WallLostDistance())
        {
            mState = "searching_wall";
            result = CurveRightToFindWall();
        }
        else
        {
            mState = "following_wall";
            result = FollowWall(scan.RightDistance(), scan.FrontRightDistance());
        }
        mLastCommand = result;
    }
    return result;
}

void Controller::UpdateFrontBlocked(double front_distance)
{
    // Once turning, wait for the larger resume distance before moving forward again.
    mFrontBlocked = front_distance < mSettings.FrontStopDistance()
        || (mFrontBlocked && front_distance < mSettings.FrontResumeDistance());
}

Controller::Command Controller::CurveRightToFindWall() const
{
    const Command result(mSettings.ForwardSpeed(), -mSettings.TurnSpeed() * SearchTurnFraction);
    return result;
}

Controller::Command Controller::FollowWall(double right_distance,
                                           double front_right_distance) const
{
    double wall_angle = 0.0;
    if (front_right_distance < mSettings.WallLostDistance())
    {
        // The 45-degree beam has equal forward and sideways components.
        const double diagonal_component = front_right_distance / std::sqrt(2.0);
        // Compare its sideways component with the right beam to estimate the wall angle.
        wall_angle = std::atan2(diagonal_component - right_distance, diagonal_component);
    }
    // Use long double to reduce the risk of overflow before limiting the turn rate.
    const long double distance_term = static_cast<long double>(mSettings.DistanceGain())
        * (static_cast<long double>(mSettings.WallDistance()) - right_distance);
    const long double heading_term = static_cast<long double>(mSettings.HeadingGain())
        * wall_angle;
    const long double steering = distance_term - heading_term;
    const long double turn_limit = mSettings.TurnSpeed();
    const double angular = static_cast<double>(std::max(-turn_limit,
                                                       std::min(turn_limit, steering)));
    const double speed_fraction = 1.0 - MaxSteeringSlowdown
        * (std::abs(angular) / mSettings.TurnSpeed());
    const Command result(mSettings.ForwardSpeed() * speed_fraction, angular);
    return result;
}
}  // namespace wall_follower
