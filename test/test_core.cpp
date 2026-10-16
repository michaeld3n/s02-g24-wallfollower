// MTRX3760 2026 Project 2: Package Deal
// File: test_core.cpp
// Author(s): OpenAI Codex (AI-assisted implementation)
//
// Description: Standalone scan and controller regression tests.
// Provenance: Original core by Cody, royce, Marz, Paekinator in ../project1_repo;
// regression tests newly written by OpenAI Codex for this adaptation.
#include "wall_follower/controller.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using wall_follower::Controller;
using wall_follower::ScanReader;
class CoreTests
{
public:
    int Run();

private:
    static constexpr double Pi = 3.14159265358979323846;
    static constexpr float NaN = std::numeric_limits<float>::quiet_NaN();
    static constexpr float Infinity = std::numeric_limits<float>::infinity();
    static constexpr double StepSeconds = 0.05;
    int Checks = 0;

    void Check(bool condition, const char* description);
    static bool Near(double actual, double expected, double tolerance = 1e-6);
    void State(const Controller& controller, const char* expected);
    void Stopped(const Controller::Command& command);
    static ScanReader Scan(float front = 2.0F, float right = 0.25F,
                           float diagonal = static_cast<float>(0.25 * std::sqrt(2.0)));
    void TestScanValidation();
    void TestAnglesAndFiltering();
    void TestSettings();
    void TestControl();
    void TestPartials();
    void TestGeometricFeedbackLoop();
};

void CoreTests::Check(bool condition, const char* description)
{
    ++Checks;
    if (!condition)
    {
        throw std::runtime_error(description);
    }
}
bool CoreTests::Near(double actual, double expected, double tolerance)
{ return std::abs(actual - expected) <= tolerance; }
void CoreTests::State(const Controller& controller, const char* expected)
{ Check(std::string(controller.StateName()) == expected, expected); }
void CoreTests::Stopped(const Controller::Command& command)
{ Check(command.Linear == 0.0 && command.Angular == 0.0, "must stop completely"); }

// Three isolated beams exercise control independently of sector filtering.
ScanReader CoreTests::Scan(float front, float right, float diagonal)
{
    ScanReader scan;
    scan.Update({right, diagonal, front}, -Pi / 2.0, Pi / 4.0, 0.01, 10.0);
    return scan;
}

void CoreTests::TestScanValidation()
{
    ScanReader scan;
    Check(!scan.HasReceivedScan() && !scan.HasValidReadings(), "initial scan empty");
    scan = Scan();
    Check(scan.HasValidReadings(), "three sectors valid");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    struct Metadata { double Start; double Step; double Min; double Max; };
    const Metadata bad[] = {
        {nan, 1, 0, 10}, {inf, 1, 0, 10}, {0, nan, 0, 10},
        {0, inf, 0, 10}, {0, 0, 0, 10}, {0, 1, -1, 10},
        {0, 1, nan, 10}, {0, 1, 0, inf}, {0, 1, 0, nan},
        {0, 1, 1, 1}, {0, 1, 2, 1},
        {0, std::numeric_limits<double>::max(), 0, 10}
    };
    for (const auto& metadata : bad)
    {
        scan = Scan();
        scan.Update({1, 1, 1}, metadata.Start, metadata.Step, metadata.Min, metadata.Max);
        Check(scan.HasReceivedScan() && !scan.HasValidFrontReading()
              && !scan.HasValidRightReading() && !scan.HasValidFrontRightReading(),
              "bad metadata clears every sector");
        Check(scan.FrontDistance() == 0.0 && scan.RightDistance() == 0.0
              && scan.FrontRightDistance() == 0.0, "no stale distances");
    }
    scan.Update({}, 0, 1, 0, 10);
    Check(scan.HasReceivedScan() && !scan.HasValidReadings(), "empty received scan");
    for (float invalid : {NaN, -Infinity, -1.0F, 0.0F, 11.0F})
    {
        scan = Scan(invalid, invalid, invalid);
        Check(!scan.HasValidFrontReading() && !scan.HasValidRightReading()
              && !scan.HasValidFrontRightReading(), "invalid returns rejected");
    }
    scan = Scan(Infinity, Infinity, Infinity);
    Check(scan.HasValidReadings() && scan.FrontDistance() == 10.0
          && scan.RightDistance() == 10.0, "positive infinity means clear space");
    scan.Update({0.125F, 10.0F, 0.125F}, -Pi / 2, Pi / 4, 0.125, 10);
    Check(scan.HasValidReadings(), "range endpoints included");
}

void CoreTests::TestAnglesAndFiltering()
{
    for (int count : {180, 360, 720, 1080})
    {
        for (double start : {-Pi, 0.0, 2.0 * Pi})
        {
            for (double sign : {-1.0, 1.0})
            {
                const double step = sign * 2.0 * Pi / count;
                std::vector<float> ranges(static_cast<std::size_t>(count), NaN);
                for (int i = 0; i < count; ++i)
                {
                    const double angle = start + i * step;
                    if (std::abs(std::remainder(angle, 2 * Pi)) < Pi / 9)
                    {
                        ranges[i] = 2.0F;
                    }
                    if (std::abs(std::remainder(angle + Pi / 2, 2 * Pi)) < Pi / 36)
                    {
                        ranges[i] = 0.25F;
                    }
                    if (std::abs(std::remainder(angle + Pi / 4, 2 * Pi)) < Pi / 36)
                    {
                        ranges[i] = 0.35F;
                    }
                }
                ScanReader scan;
                scan.Update(ranges, start, step, 0.01, 10);
                Check(scan.HasValidReadings() && Near(scan.FrontDistance(), 2)
                      && Near(scan.RightDistance(), 0.25)
                      && Near(scan.FrontRightDistance(), 0.35), "wrapped/resampled sectors");
            }
        }
    }
    ScanReader scan;
    scan.Update({2, 0.2F, 3}, -Pi / 180, Pi / 180, 0.01, 10);
    Check(Near(scan.FrontDistance(), 0.2), "front selects nearest obstacle");
    scan.Update({0.25F, 0.05F, 0.25F}, -Pi / 2 - Pi / 180, Pi / 180, 0.01, 10);
    Check(Near(scan.RightDistance(), 0.25), "side median rejects outlier");
    Check(!scan.HasValidFrontReading(), "limited field of view stays partial");
}

void CoreTests::TestSettings()
{
    Controller::Settings defaults;
    Check(defaults.IsValid() && defaults.WallDistance == 0.25
          && defaults.ForwardSpeed == 0.10 && defaults.FrontStopDistance == 0.25
          && defaults.FrontResumeDistance == 0.35, "requested defaults");
    const std::vector<double Controller::Settings::*> fields = {
        &Controller::Settings::WallDistance, &Controller::Settings::ForwardSpeed,
        &Controller::Settings::TurnSpeed, &Controller::Settings::FrontStopDistance,
        &Controller::Settings::FrontResumeDistance, &Controller::Settings::WallLostDistance,
        &Controller::Settings::DistanceGain, &Controller::Settings::HeadingGain
    };
    for (auto field : fields)
    {
        for (double value : {-1.0, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()})
        {
            auto settings = defaults;
            settings.*field = value;
            Controller controller(settings);
            Check(!settings.IsValid() && !controller.HasValidSettings(), "invalid settings rejected");
            Stopped(controller.CalculateCommand(Scan()));
            controller.Reset();
            State(controller, "invalid_configuration");
        }
    }
    for (auto field : fields)
    {
        auto settings = defaults;
        settings.*field = 0.0;
        Check(settings.IsValid() == (field == &Controller::Settings::HeadingGain),
              "zero allowed only for heading gain");
    }
    defaults.FrontResumeDistance = defaults.FrontStopDistance;
    Check(!defaults.IsValid(), "resume must exceed stop");
    defaults = Controller::Settings{};
    defaults.WallLostDistance = defaults.WallDistance;
    Check(!defaults.IsValid(), "wall lost must exceed desired distance");
}

void CoreTests::TestControl()
{
    Controller controller(Controller::Settings{});
    State(controller, "waiting_for_scan");
    Stopped(controller.CalculateCommand(ScanReader{}));
    auto command = controller.CalculateCommand(Scan());
    State(controller, "following_wall");
    Check(Near(command.Angular, 0) && Near(command.Linear, 0.1), "parallel wall drives straight");
    command = controller.CalculateCommand(Scan(2, 0.15F, 0.15F * std::sqrt(2.0F)));
    Check(command.Angular > 0 && command.Linear < 0.1, "too close turns left and slows");
    command = controller.CalculateCommand(Scan(2, 0.45F, 0.45F * std::sqrt(2.0F)));
    Check(command.Angular < 0, "too far turns right");
    Check(controller.CalculateCommand(Scan(2, 0.25F, 0.28F)).Angular > 0,
          "heading toward wall corrects left");
    Check(controller.CalculateCommand(Scan(2, 0.25F, 0.45F)).Angular < 0,
          "heading away from wall corrects right");
    command = controller.CalculateCommand(Scan(2, 0.8F, 2));
    Check(Near(command.Angular, -0.65) && Near(command.Linear, 0.05), "steering saturation");
    command = controller.CalculateCommand(Scan(Infinity, Infinity, Infinity));
    State(controller, "searching_wall");
    Check(Near(command.Linear, 0.1) && Near(command.Angular, -0.325), "lost wall search");

    // Binary-exact thresholds avoid float representation obscuring boundary tests.
    auto settings = Controller::Settings{};
    settings.FrontResumeDistance = 0.375;
    Controller hysteresis(settings);
    hysteresis.CalculateCommand(Scan(0.25F));
    State(hysteresis, "following_wall");
    command = hysteresis.CalculateCommand(Scan(0.20F));
    Check(command.Linear == 0 && command.Angular > 0, "front obstacle turns in place");
    hysteresis.CalculateCommand(Scan(0.30F));
    State(hysteresis, "turning_left");
    hysteresis.CalculateCommand(Scan(0.375F));
    State(hysteresis, "following_wall");
    hysteresis.CalculateCommand(Scan(0.2F));
    hysteresis.Reset();
    State(hysteresis, "waiting_for_scan");
    hysteresis.CalculateCommand(Scan(0.30F));
    State(hysteresis, "following_wall");
}

void CoreTests::TestPartials()
{
    Controller controller(Controller::Settings{});
    for (const auto& scan : {Scan(NaN, NaN, NaN), Scan(NaN), Scan(-Infinity)})
    {
        controller.CalculateCommand(Scan(0.2F));
        Stopped(controller.CalculateCommand(scan));
        State(controller, "invalid_scan");
        controller.CalculateCommand(Scan(0.3F));
        State(controller, "following_wall");
    }
    controller.CalculateCommand(Scan(0.2F));
    Stopped(controller.CalculateCommand(ScanReader{}));
    controller.CalculateCommand(Scan(0.3F));
    State(controller, "following_wall");
    for (const auto& scan : {Scan(2, NaN, NaN), Scan(2, NaN), Scan(2, 0.25F, NaN)})
    {
        const auto command = controller.CalculateCommand(scan);
        State(controller, "partial_scan");
        Check(command.Linear >= 0 && command.Linear <= 0.01, "partial speed capped");
    }
    Check(Near(controller.CalculateCommand(Scan(2, 0.25F, NaN)).Angular, 0),
          "missing diagonal does not invent heading");
    Check(controller.CalculateCommand(Scan(2, 0.15F, NaN)).Angular > 0,
          "distance-only partial steers away from close wall");
    auto command = controller.CalculateCommand(Scan(2, NaN, 0.15F));
    Check(command.Linear == 0 && command.Angular > 0, "unknown right avoids close diagonal");
    controller.CalculateCommand(Scan(0.2F, NaN, NaN));
    State(controller, "turning_left");
    controller.CalculateCommand(Scan(0.3F, NaN, NaN));
    State(controller, "turning_left");
    controller.CalculateCommand(Scan(0.5F, NaN, NaN));
    State(controller, "partial_scan");
    controller.CalculateCommand(Scan());
    State(controller, "following_wall");
}

void CoreTests::TestGeometricFeedbackLoop()
{
    // Raycast against the infinite horizontal wall y=0, then integrate a
    // unicycle. This checks feedback convergence, not only synthetic signs.
    for (double initial_gap : {0.16, 0.45})
    {
        double gap = initial_gap;
        double heading = 0.0;
        Controller controller(Controller::Settings{});
        for (int tick = 0; tick < 1200; ++tick)
        {
            std::vector<float> ranges(720, Infinity);
            for (std::size_t i = 0; i < ranges.size(); ++i)
            {
                const double direction = heading - Pi + i * Pi / 360;
                if (std::sin(direction) < -1e-9)
                {
                    const double distance = -gap / std::sin(direction);
                    if (distance <= 10)
                    {
                        ranges[i] = static_cast<float>(distance);
                    }
                }
            }
            ScanReader scan;
            scan.Update(ranges, -Pi, Pi / 360, 0.01, 10);
            const auto command = controller.CalculateCommand(scan);
            Check(std::isfinite(command.Linear) && std::isfinite(command.Angular)
                  && command.Linear >= 0 && command.Linear <= 0.1
                  && std::abs(command.Angular) <= 0.65, "bounded geometric-loop command");
            gap += command.Linear * std::sin(heading) * StepSeconds;
            heading += command.Angular * StepSeconds;
            Check(gap > 0.10, "geometric loop avoids wall");
        }
        Check(std::abs(gap - 0.25) < 0.015 && std::abs(heading) < 0.03,
              "geometric feedback converges to parallel target wall");
    }
}

int CoreTests::Run()
{
    Checks = 0;
    try
    {
        TestScanValidation();
        TestAnglesAndFiltering();
        TestSettings();
        TestControl();
        TestPartials();
        TestGeometricFeedbackLoop();
        std::cout << "PASS: " << Checks << " core checks (6 test groups)\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL after " << Checks << " checks: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
}  // namespace

int main()
{
    CoreTests tests;
    return tests.Run();
}
