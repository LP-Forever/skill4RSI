#!/usr/bin/env bash
# Original ROS/FrankaPy installation is an EXTERNAL dependency; never start its driver here.
set -euo pipefail
cd "$(dirname "$0")"
: "${SKILL4RSI_ROS_SETUP:?Set the existing ROS setup.bash path}"
: "${SKILL4RSI_CATKIN_SETUP:?Set the existing catkin devel/setup.bash path}"
: "${SKILL4RSI_ROS_PYTHON:?Set the interpreter compatible with the existing ROS/FrankaPy installation}"
source "$SKILL4RSI_ROS_SETUP"
source "$SKILL4RSI_CATKIN_SETUP"
export PYTHONDONTWRITEBYTECODE=1
exec "$SKILL4RSI_ROS_PYTHON" -u -B run_task.py "$@"
