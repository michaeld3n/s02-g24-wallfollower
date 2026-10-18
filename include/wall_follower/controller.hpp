// The wall-following controller
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
    struct Settings
    {
        double WallDistance = 0.25;
        double ForwardSpeed = 0.10;
        double TurnSpeed = 0.65;
        double FrontStopDistance = 0.25;
        double FrontResumeDistance = 0.35;
        double WallLostDistance = 0.90;
        double DistanceGain = 2.0;
        double HeadingGain = 1.2;
        bool IsValid() const;
    };
    // Forward speed is in m/s. Turning is in rad/s, with positive values turning left.
    struct Command
    {
        double Linear = 0.0;
        double Angular = 0.0;
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
