// MTRX3760 2026 Project 2: Package Deal
// File: controller.hpp
// Author(s): Cody, Marzuq, Royce
//
// This class defines the steering settings and velocity command used by the
// wall follower. It keeps the previous command and obstacle-turn state so
// unusable scans do not replace the last movement decision. It does not
// depend on ROS messages.

#ifndef WALL_FOLLOWER_CONTROLLER_HPP
#define WALL_FOLLOWER_CONTROLLER_HPP

#include "wall_follower/scan_reader.hpp"

namespace wall_follower
{
class Controller
{
public:
    // Startup settings stay fixed while the controller is running.
    class Settings
    {
    public:
        Settings() = default;
        Settings(double wall_distance, double forward_speed, double turn_speed,
                 double front_stop_distance, double front_resume_distance,
                 double wall_lost_distance, double distance_gain, double heading_gain);
        double WallDistance() const;
        double ForwardSpeed() const;
        double TurnSpeed() const;
        double FrontStopDistance() const;
        double FrontResumeDistance() const;
        double WallLostDistance() const;
        double DistanceGain() const;
        double HeadingGain() const;
        bool IsValid() const;

    private:
        double mWallDistance = 0.25;
        double mForwardSpeed = 0.10;
        double mTurnSpeed = 0.65;
        double mFrontStopDistance = 0.25;
        double mFrontResumeDistance = 0.35;
        double mWallLostDistance = 0.90;
        double mDistanceGain = 2.0;
        double mHeadingGain = 1.2;
    };
    // Forward speed is in m/s. Turning is in rad/s, with positive values turning left.
    class Command
    {
    public:
        Command(double linear = 0.0, double angular = 0.0);
        double Linear() const;
        double Angular() const;

    private:
        double mLinear;
        double mAngular;
    };
    explicit Controller(const Settings& settings);

    // An unusable scan holds the previous command and reports invalid_scan.
    Command CalculateCommand(const ScanReader& scan);
    const char* StateName() const;

    // Forget the last command and obstacle turn when starting or stopping.
    void Reset();
    bool HasValidSettings() const;

private:
    static constexpr double SearchTurnFraction = 0.5;
    static constexpr double MaxSteeringSlowdown = 0.5;

    void UpdateFrontBlocked(double front_distance);
    Command FollowWall(double right_distance, double front_right_distance) const;
    Command CurveRightToFindWall() const;
    const Settings mSettings;
    // Remains zero until a usable scan is processed after reset.
    Command mLastCommand;
    bool mFrontBlocked = false;
    const char* mState = "waiting_for_scan";
};
}  // namespace wall_follower
#endif
