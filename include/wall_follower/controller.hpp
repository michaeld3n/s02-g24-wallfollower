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
class Controller
{
public:
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
    struct Command
    {
        double Linear = 0.0;
        double Angular = 0.0;
    };
    explicit Controller(const Settings& settings);
    Command CalculateCommand(const ScanReader& scan);
    const char* StateName() const;
    // Clear hysteresis; the caller must supply a scan again before driving.
    void Reset();
    bool HasValidSettings() const;

private:
    static constexpr double CornerTurnFraction = 0.5;
    static constexpr double MaxSteeringSlowdown = 0.5;
    static constexpr double PartialScanSpeed = 0.01;

    void UpdateFrontBlocked(double front_distance);
    Command FollowWall(double right_distance, double front_right_distance) const;
    Command CurveRightToFindWall() const;
    const Settings mSettings;
    bool mFrontBlocked = false;
    const char* mState = "waiting_for_scan";
};
}  // namespace wall_follower
#endif
