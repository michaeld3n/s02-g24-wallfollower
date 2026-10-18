// MTRX3760 2026 Project 2: Package Deal
// File: scan_reader.hpp
// Author(s): Cody, royce, Marz, Paekinator; OpenAI Codex (AI-assisted adaptation)
//
// Description: Extract front and right-wall sectors from a laser scan.
// Provenance: Adapted from Project1 CScanReader in ../project1_repo.
#ifndef WALL_FOLLOWER_SCAN_READER_HPP
#define WALL_FOLLOWER_SCAN_READER_HPP

#include <vector>

namespace wall_follower
{
// Reduces laser rays to front, right and front-right sector distances.
class ScanReader
{
public:
    // Angles are radians counterclockwise from forward; distances are metres.
    // Positive infinity and an entirely NaN sector denote range_max.
    // Missing coverage and other unusable sectors remain invalid.
    // Every update replaces all previous sector readings.
    void Update(const std::vector<float>& ranges, double angle_min,
                double angle_increment, double range_min, double range_max);
    // True only when all three required sectors have usable distances.
    bool HasValidReadings() const;
    bool HasReceivedScan() const;

    // Distances are usable for steering only when HasValidReadings is true.
    double FrontDistance() const;
    double RightDistance() const;
    double FrontRightDistance() const;

private:
    static constexpr double Pi = 3.14159265358979323846;
    static constexpr double FrontHalfWidth = Pi / 9.0;
    static constexpr double SideHalfWidth = Pi / 36.0;

    struct Sector
    {
        double Distance = 0.0;
        bool Valid = false;
    };
    static Sector ReadSector(const std::vector<float>& ranges, double angle_min,
                             double angle_increment, double range_min,
                             double range_max, double centre, double half_width,
                             bool use_minimum);
    Sector mFront;
    Sector mRight;
    Sector mFrontRight;
    bool mHaveScan = false;
};
}  // namespace wall_follower
#endif
