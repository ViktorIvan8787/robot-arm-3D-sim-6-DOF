#!/usr/bin/env bash

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${project_dir}/build"

for command_name in cmake git g++; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "Error: '${command_name}' is required but was not found."
        exit 1
    fi
done

echo "Configuring robot-arm simulation..."
cmake -S "${project_dir}" -B "${build_dir}"

echo "Building robot-arm simulation..."
cmake --build "${build_dir}" --parallel "$(nproc)"

simulator="${build_dir}/cpp/robot_arm_simulator"
if [[ ! -x "${simulator}" ]]; then
    echo "Error: build completed but the simulator was not found at:"
    echo "  ${simulator}"
    exit 1
fi

echo "Launching robot-arm simulation..."
exec "${simulator}"