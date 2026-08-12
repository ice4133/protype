#!/usr/bin/env bash

# A fresh checkout has no colcon overlay yet. Source it after the first build,
# while still allowing Pixi to run the build task that creates it.
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/install/setup.bash"
fi

