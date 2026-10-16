# Validation record

Recorded 16 October 2026. This file distinguishes checks actually run from checks still required. The user authorized publishing this development version to `michaeld3n/s02-g24-wallfollower` on GitHub.com. No release tag is included; university publication and Linux validation remain outstanding.

## Completed locally on macOS

- Source baseline verified: Project 1 local HEAD and remote HEAD both `05df1f5b89db609be2e087675ce7abb3d40a67cf`.
- Portable C++ core: `bash scripts/test_core.sh` passed **4,987 checks in six groups**. This exercises the actual scan reader and controller, not a mock of them.
- Core checks include steering signs, heading correction, front-obstacle hysteresis, partial/missing/invalid scans, positive infinity, angular conventions/resolutions, reset behaviour, invalid settings and geometric wall convergence.
- AddressSanitizer and UndefinedBehaviorSanitizer core execution passed during implementation.
- Python launch/test files parse successfully. This is syntax validation, not execution against ROS.
- Package manifest and SVG diagrams parse as XML.
- Both manual SVG diagrams rendered and visually inspected in Chromium. Editable PlantUML sources accompany them; PlantUML compilation itself has not been run here.

## Still required on Ubuntu 24.04 with ROS 2 Jazzy

This development machine has no ROS 2, colcon, Gazebo or container runtime. Therefore the following results are **not yet verified**, and the package must not be described as release-tested until they pass.

1. Clone the development repository (after this commit is pushed), or copy the entire local repository to a fresh workspace's `src` directory using the README instructions.
2. Install dependencies with `rosdep`; run `colcon build --symlink-install --packages-select wall_follower`.
3. Source `install/setup.bash`, run `colcon test --packages-select wall_follower --event-handlers console_direct+`, then `colcon test-result --verbose`.
4. Confirm the isolated launch test passes for both TwistStamped and Twist. It checks velocity output, state, start/stop, fresh-input restart, timestamp independence and receipt-timeout recovery.
5. Follow the README's standard `turtlebot3_world` session. Confirm movement, right-wall following, obstacle turns, state output and runtime stopping.
6. Pause and resume Gazebo: expect a quiet `waiting_for_scan` state and zero velocity after the receipt timeout, followed by automatic recovery when scans resume.
7. Stop the controller with Ctrl+C while enabled and verify its final zero command reaches the simulator. Forced termination and network loss are not equivalent to graceful shutdown.
8. Repeat the build and documented run from a clean workspace with only this package and declared/external dependencies; record the ROS/Gazebo versions and actual results here.

## Physical robot

Not tested in this adaptation. The same ROS adapter supports either velocity message type, but that is not evidence of tested hardware compatibility. Confirm the robot's scan and velocity interfaces and tune on hardware separately if desired.

## Release checks after validation

- Review the README, diagrams and actual author names as a team.
- Confirm no binaries or workspace build output are staged.
- Create/connect the required private university repository when requested.
- Publish a version tag only after the user explicitly authorizes pushing. Never move a published tag.
