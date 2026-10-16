# Working instructions

Do not push code, branches, or tags to any remote unless the user explicitly authorizes it. Work locally first.

This is Group 24, Section 02's Phase 1 wall follower package. The final publication destination required by the brief is a private repository named `s02-g24-wallfollower` in `mtrx3760-26` on `github.sydney.edu.au`. The current GitHub.com remote is a separate development repository.

## Phase 1 acceptance checklist

- Rework the team's Project 1 wall follower into a self-contained standard ROS 2 C++ package.
- Include package.xml, CMakeLists.txt, source, and a launch file that starts the package.
- Consume laser scans, drive through cmd_vel, support runtime start/stop, and report state.
- Choose and document topic names, message types, parameters, and edge behaviour.
- Run in a standard TurtleBot3 Gazebo world without physical hardware; document the world and launch instructions.
- Include a launch_testing integration test run with colcon test, using the isolated test runner and handmade scans; check the main output.
- README: purpose, build/run/test instructions, simulation instructions, complete interface, and edge behaviour.
- Include a manual ROS Node Design Diagram following the unit standard; rqt_graph is not a substitute.
- Include a simplified UML Class Diagram following the unit standard, without member variables or member functions.
- Every C++ source/header file must begin with the brief's Project 2 header, filename, actual authors, and description. Do not invent author contributions.
- Follow the unit's design/style checklist: meaningful classes, private data, logical atomic member functions, appropriate composition/polymorphism, clear naming and formatting, and constants rather than magic numbers.
- Do not commit binaries or build output.
- Verify from a clean checkout before release.
- Publication requires a pushed v1.0 tag by 23:55 Tuesday of Week 11, but requires the user's explicit push authorization first. Never move a published tag; use later version tags for improvements.

Source of truth: ../Project 2 - Package Deal.pdf, particularly pages 3–5, 7, and 10. The brief leaves message types to the team; do not assume Twist is mandatory without checking the target TurtleBot interface.
