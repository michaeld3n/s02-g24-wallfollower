// MTRX3760 2026 Project 2: Package Deal
// File: scan_reader.cpp
// Author(s): Cody, royce, Marz, Paekinator; OpenAI Codex (AI-assisted adaptation)
//
// Description: Angle-based minimum/median sector extraction.
// Provenance: Adapted from Project1 CScanReader in ../project1_repo.
#include "wall_follower/scan_reader.hpp"

#include <algorithm>
#include <cmath>

namespace wall_follower
{
void ScanReader::Update(const std::vector<float>& ranges, double angle_min,
                        double angle_increment, double range_min, double range_max)
{
    mHaveScan = true;
    mFront = Sector{};
    mRight = Sector{};
    mFrontRight = Sector{};
    if (ranges.empty() || !std::isfinite(angle_min)
        || !std::isfinite(angle_increment) || angle_increment == 0.0
        || !std::isfinite(range_min) || !std::isfinite(range_max)
        || range_min < 0.0 || range_max <= range_min)
    {
        return;
    }

    // Reject metadata whose final beam angle would overflow, too.
    const double last_angle = angle_min
        + static_cast<double>(ranges.size() - 1) * angle_increment;
    if (!std::isfinite(last_angle))
    {
        return;
    }
    mFront = ReadSector(ranges, angle_min, angle_increment, range_min, range_max,
                        0.0, FrontHalfWidth, true);
    mRight = ReadSector(ranges, angle_min, angle_increment, range_min, range_max,
                        -Pi / 2.0, SideHalfWidth, false);
    mFrontRight = ReadSector(ranges, angle_min, angle_increment, range_min, range_max,
                             -Pi / 4.0, SideHalfWidth, false);
}

ScanReader::Sector ScanReader::ReadSector(
    const std::vector<float>& ranges, double angle_min, double angle_increment,
    double range_min, double range_max, double centre, double half_width,
    bool use_minimum)
{
    std::vector<double> valid_ranges;
    for (std::size_t index = 0; index < ranges.size(); ++index)
    {
        const double angle = angle_min + static_cast<double>(index) * angle_increment;
        const double difference = std::remainder(angle - centre, 2.0 * Pi);
        if (std::abs(difference) > half_width)
        {
            continue;
        }
        const double range = ranges[index];
        if (std::isinf(range) && range > 0.0)
        {
            valid_ranges.push_back(range_max);
        }
        else if (std::isfinite(range) && range >= range_min && range <= range_max)
        {
            valid_ranges.push_back(range);
        }
    }
    Sector result;
    if (!valid_ranges.empty())
    {
        std::sort(valid_ranges.begin(), valid_ranges.end());
        result.Valid = true;
        result.Distance = use_minimum ? valid_ranges.front()
            : valid_ranges[valid_ranges.size() / 2];
    }
    return result;
}

bool ScanReader::HasValidReadings() const
{ return mFront.Valid && mRight.Valid && mFrontRight.Valid; }
bool ScanReader::HasValidFrontReading() const { return mFront.Valid; }
bool ScanReader::HasValidRightReading() const { return mRight.Valid; }
bool ScanReader::HasValidFrontRightReading() const { return mFrontRight.Valid; }
bool ScanReader::HasReceivedScan() const { return mHaveScan; }
double ScanReader::FrontDistance() const { return mFront.Distance; }
double ScanReader::RightDistance() const { return mRight.Distance; }
double ScanReader::FrontRightDistance() const { return mFrontRight.Distance; }
}  // namespace wall_follower
