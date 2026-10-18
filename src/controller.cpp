// MTRX3760 2026 Project 2: Package Deal
// File: controller.cpp
// Author(s): Cody, royce, Marz, Paekinator; OpenAI Codex (AI-assisted adaptation)
//
// Description: Right-wall heading/distance steering with front hysteresis.
// Provenance: Adapted from Project1 CWallFollower in ../project1_repo.
#include "wall_follower/controller.hpp"

#include <algorithm>
#include <cmath>

namespace wall_follower
{
bool Controller::Settings::IsValid() const
{
    return std::isfinite(WallDistance) && std::isfinite(ForwardSpeed)
        && std::isfinite(TurnSpeed) && std::isfinite(FrontStopDistance)
        && std::isfinite(FrontResumeDistance) && std::isfinite(WallLostDistance)
        && std::isfinite(DistanceGain) && std::isfinite(HeadingGain)
        && WallDistance > 0.0 && ForwardSpeed > 0.0 && TurnSpeed > 0.0
        && FrontStopDistance > 0.0 && FrontResumeDistance > FrontStopDistance
        && WallLostDistance > WallDistance && DistanceGain > 0.0 && HeadingGain >= 0.0;
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
        // Reject the whole scan without changing the saved command or obstacle latch.
        mState = "invalid_scan";
        result = mLastCommand;
    }
    else
    {
        UpdateFrontBlocked(scan.FrontDistance());
        if (mFrontBlocked)
        {
            mState = "turning_left";
            result.Angular = mSettings.TurnSpeed;
        }
        else if (scan.RightDistance() > mSettings.WallLostDistance)
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
    mFrontBlocked = front_distance < mSettings.FrontStopDistance
        || (mFrontBlocked && front_distance < mSettings.FrontResumeDistance);
}

Controller::Command Controller::CurveRightToFindWall() const
{
    Command result;
    result.Linear = mSettings.ForwardSpeed;
    result.Angular = -mSettings.TurnSpeed * SearchTurnFraction;
    return result;
}

Controller::Command Controller::FollowWall(double right_distance,
                                           double front_right_distance) const
{
    double wall_angle = 0.0;
    if (front_right_distance < mSettings.WallLostDistance)
    {
        const double diagonal_component = front_right_distance / std::sqrt(2.0);
        wall_angle = std::atan2(diagonal_component - right_distance, diagonal_component);
    }
    Command result;
    // Wider intermediates preserve cancellation for large finite settings.
    const long double distance_term = static_cast<long double>(mSettings.DistanceGain)
        * (static_cast<long double>(mSettings.WallDistance) - right_distance);
    const long double heading_term = static_cast<long double>(mSettings.HeadingGain)
        * wall_angle;
    const long double steering = distance_term - heading_term;
    const long double turn_limit = mSettings.TurnSpeed;
    result.Angular = static_cast<double>(std::max(-turn_limit,
                                                std::min(turn_limit, steering)));
    const double speed_fraction = 1.0 - MaxSteeringSlowdown
        * (std::abs(result.Angular) / mSettings.TurnSpeed);
    result.Linear = mSettings.ForwardSpeed * speed_fraction;
    return result;
}
}  // namespace wall_follower
