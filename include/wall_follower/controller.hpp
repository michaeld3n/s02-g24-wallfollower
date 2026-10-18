// MTRX3760 2026 Project 2: Package Deal
// File: controller.hpp
// Author(s): Cody, royce, Marz, Paekinator; OpenAI Codex (AI-assisted adaptation)
//
// Description: ROS-independent right-wall control and diagnostic states.
// Provenance: Adapted from Project1 CWallFollower in ../project1_repo.
#ifndef WALL_FOLLOWER_CONTROLLER_HPP
#define WALL_FOLLOWER_CONTROLLER_HPP

#include "wall_follower/scan_reader.hpp"

namespace wall_follower
{
// Produces right-wall steering and retains the command from the last usable scan.
class Controller
{
public:
    // Startup tuning values; IsValid checks their numerical relationships.
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
    // Forward speed in metres/second and left-positive turn rate in radians/second.
    struct Command
    {
        double Linear = 0.0;
        double Angular = 0.0;
    };
    explicit Controller(const Settings& settings);

    // An unusable scan holds the previous command and reports invalid_scan.
    Command CalculateCommand(const ScanReader& scan);
    const char* StateName() const;

    // Clear the saved command and front-obstacle hysteresis on an enable transition.
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
