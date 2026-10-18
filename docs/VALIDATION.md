# Validation record

Originally recorded 16 October 2026; behaviour and local checks updated 18 October 2026. This file distinguishes checks actually run from checks still required. The user authorized publishing this development version to `michaeld3n/s02-g24-wallfollower` on GitHub.com. No release tag is included; university publication and Linux validation remain outstanding.

## Completed locally on macOS

- Source baseline verified: Project 1 local HEAD and remote HEAD both `05df1f5b89db609be2e087675ce7abb3d40a67cf`.
- Historical validation before removal: the standalone C++ suite passed **5,069 checks in six groups**, including scan filtering, steering, retained commands, reset and geometric wall convergence. AddressSanitizer and UndefinedBehaviorSanitizer execution also passed on 18 October 2026. That suite and its shell runner were subsequently removed at the user's request; they are not runnable from the current checkout. The package now ships only the ROS integration test.
- Python launch/test files parse successfully. This is syntax validation, not execution against ROS.
- Package manifest and SVG diagrams parse as XML.
- Both manual SVG diagrams were rendered and visually inspected in Chromium during the original implementation. The updated class diagram was rendered locally and visually inspected on 18 October 2026. Editable PlantUML sources accompany them; PlantUML compilation itself has not been run here.

## Still required on Ubuntu 24.04 with ROS 2 Jazzy

This development machine has no ROS 2, colcon, Gazebo or container runtime. Therefore the following results are **not yet verified**, and the package must not be described as release-tested until they pass.

1. Clone the development repository after explicit push authorization and publication, or copy the entire local repository to a fresh workspace's `src` directory using the README instructions.
2. Install dependencies with `rosdep`; run `colcon build --symlink-install --packages-select wall_follower`.
3. Source `install/setup.bash`, run `colcon test --packages-select wall_follower --event-handlers console_direct+`, then `colcon test-result --verbose`.
4. Confirm the isolated launch test passes for TwistStamped output. It checks velocity and stamped headers, state, start/stop, fresh-input restart, all-NaN substitution, held commands on unusable input and scan silence, timestamp independence, and saved-command clearing after restart.
5. Follow the README's standard `turtlebot3_world` session. Confirm movement, right-wall following, obstacle turns, state output and runtime stopping.
6. Verify scan silence retains the last command and state: there is no scan timeout. Disable before pausing Gazebo if zero commands are required. Resume with usable scans and confirm steering updates. An unusable scan must report `invalid_scan` while holding the last command; after disable/re-enable it must instead hold zero until a usable scan arrives.
7. Stop the controller with Ctrl+C while enabled and verify its final zero command reaches the simulator. Forced termination and network loss are not equivalent to graceful shutdown.
8. Repeat the build and documented run from a clean workspace with only this package and declared/external dependencies; record the ROS/Gazebo versions and actual results here.

## Physical robot

Not tested in this adaptation. The ROS adapter publishes only TwistStamped; this is not evidence of tested hardware compatibility. Confirm the robot's scan and stamped velocity interfaces and tune on hardware separately if desired.

## Release checks after validation

- Review the README, diagrams and actual author names as a team.
- Confirm no binaries or workspace build output are staged.
- Create/connect the required private university repository when requested.
- Publish a version tag only after the user explicitly authorizes pushing. Never move a published tag.
