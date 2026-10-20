#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${workspace_dir}/install/setup.bash"
exec ros2 run stm32_pos_control_test multi_stm32_control "$@"
