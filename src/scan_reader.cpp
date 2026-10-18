// Turning laser rays into sector distances
//
// The reader groups rays by angle and finds the front minimum and side
// medians. It ignores unusable individual returns and uses the sensor's
// maximum range when every ray in a sector is NaN. Missing coverage and
// malformed scans remain invalid.

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
    const bool valid_metadata = !ranges.empty() && std::isfinite(angle_min)
        && std::isfinite(angle_increment) && angle_increment != 0.0
        && std::isfinite(range_min) && std::isfinite(range_max)
        && range_min >= 0.0 && range_max > range_min;
    if (valid_metadata)
    {
        const double last_angle = angle_min
            + static_cast<double>(ranges.size() - 1) * angle_increment;
        if (std::isfinite(last_angle))
        {
            mFront = ReadSector(ranges, angle_min, angle_increment, range_min, range_max,
                                0.0, FrontHalfWidth, true);
            mRight = ReadSector(ranges, angle_min, angle_increment, range_min, range_max,
                                -Pi / 2.0, SideHalfWidth, false);
            mFrontRight = ReadSector(ranges, angle_min, angle_increment, range_min, range_max,
                                     -Pi / 4.0, SideHalfWidth, false);
        }
    }
}

ScanReader::Sector ScanReader::ReadSector(
    const std::vector<float>& ranges, double angle_min, double angle_increment,
    double range_min, double range_max, double centre, double half_width,
    bool use_minimum)
{
    std::vector<double> valid_ranges;
    std::size_t beam_count = 0;
    std::size_t nan_count = 0;
    for (std::size_t index = 0; index < ranges.size(); ++index)
    {
        const double angle = angle_min + static_cast<double>(index) * angle_increment;
        const double difference = std::remainder(angle - centre, 2.0 * Pi);
        if (std::abs(difference) <= half_width)
        {
            ++beam_count;
            const double range = ranges[index];
            if (std::isnan(range))
            {
                ++nan_count;
            }
            else if (std::isinf(range) && range > 0.0)
            {
                valid_ranges.push_back(range_max);
            }
            else if (std::isfinite(range) && range >= range_min && range <= range_max)
            {
                valid_ranges.push_back(range);
            }
        }
    }

    Sector result;
    if (!valid_ranges.empty())
    {
        std::sort(valid_ranges.begin(), valid_ranges.end());
        result.Valid = true;
        // Side sectors use the upper middle reading when the count is even.
        result.Distance = use_minimum ? valid_ranges.front()
            : valid_ranges[valid_ranges.size() / 2];
    }
    else if (beam_count > 0 && nan_count == beam_count)
    {
        // All NaNs count as maximum range; a sector with no rays stays unknown.
        result.Valid = true;
        result.Distance = range_max;
    }
    return result;
}

bool ScanReader::HasValidReadings() const
{
    return mFront.Valid && mRight.Valid && mFrontRight.Valid;
}

bool ScanReader::HasReceivedScan() const
{
    return mHaveScan;
}

double ScanReader::FrontDistance() const
{
    return mFront.Distance;
}

double ScanReader::RightDistance() const
{
    return mRight.Distance;
}

double ScanReader::FrontRightDistance() const
{
    return mFrontRight.Distance;
}
}  // namespace wall_follower
