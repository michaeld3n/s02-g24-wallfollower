#!/bin/sh
: <<'PROJECT_HEADER'
// MTRX3760 2026 Project 2: Package Deal
// File: test_core.sh
// Author(s): OpenAI Codex (AI-assisted adaptation)
//
// Description: Build and test the ROS-independent core with temporary outputs.
// Provenance: Test runner for the Project1 adaptation of code by Cody, royce,
// Marz, Paekinator in ../project1_repo; script newly written by OpenAI Codex.
PROJECT_HEADER
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
project_dir=$(CDPATH= cd -- "$script_dir/.." && pwd)
if [ -n "${CXX:-}" ]; then
    compiler=$CXX
elif command -v clang++ >/dev/null 2>&1; then
    compiler=clang++
else
    compiler=g++
fi
test_dir=$(mktemp -d "${TMPDIR:-/tmp}/wall-follower-core.XXXXXXXX")
cleanup() {
    # Only remove the exact temporary directory created by this invocation.
    if [ -n "$test_dir" ] && [ -d "$test_dir" ]; then
        rm -rf -- "$test_dir"
    fi
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

"$compiler" -std=c++17 -Wall -Wextra -Wpedantic -Werror \
    -I "$project_dir/include" \
    "$project_dir/src/scan_reader.cpp" \
    "$project_dir/src/controller.cpp" \
    "$project_dir/test/test_core.cpp" \
    -o "$test_dir/test_core"
"$test_dir/test_core"
