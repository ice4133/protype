#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_SETUP="${PROJECT_ROOT}/install/setup.bash"

if [[ ! -f "${INSTALL_SETUP}" ]]; then
  printf '%s\n' \
    '错误：找不到 install/setup.bash。请先执行 pixi run build-marvin。' >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${INSTALL_SETUP}"
set -u

if [[ "$(uname -m)" != "x86_64" ]]; then
  printf '%s\n' '错误：当前厂商 SDK 只提供 x86_64 动态库。' >&2
  exit 1
fi

printf '%s\n' '1/4 ROS 包发现检查'
ros2 pkg prefix marvin_hardware_backend
EXECUTABLES="$(ros2 pkg executables marvin_hardware_backend)"
printf '%s\n' "${EXECUTABLES}"
if [[ "${EXECUTABLES}" != *"marvin_hardware_bridge"* ]] || \
   [[ "${EXECUTABLES}" != *"marvin_backend_probe"* ]] || \
   [[ "${EXECUTABLES}" != *"marvin_feedback_probe"* ]] || \
   [[ "${EXECUTABLES}" != *"marvin_hold_position"* ]] || \
   [[ "${EXECUTABLES}" != *"marvin_small_motion"* ]]; then
  printf '%s\n' '错误：Marvin ROS 入口程序安装不完整。' >&2
  exit 1
fi

printf '%s\n' '2/4 后端 Python 模块与 SDK 路径检查（不加载动态库）'
ros2 run marvin_hardware_backend marvin_backend_probe --resolve-only

printf '%s\n' '3/4 厂商动态库依赖检查（不连接机器人）'
SDK_SOURCE_ROOT="${PROJECT_ROOT}/src/marvin/marvin_hardware_backend"
(
  cd "${SDK_SOURCE_ROOT}"
  sha256sum --check VENDOR_SDK_SHA256SUMS
)
SDK_LIBRARY="$(python -c 'from marvin_hardware_backend.sdk_loader import resolve_marvin_sdk; print(resolve_marvin_sdk().library)')"
if [[ ! -f "${SDK_LIBRARY}" ]]; then
  printf '错误：找不到 %s\n' "${SDK_LIBRARY}" >&2
  exit 1
fi
LDD_OUTPUT="$(ldd "${SDK_LIBRARY}")"
if [[ "${LDD_OUTPUT}" == *"not found"* ]]; then
  printf '错误：动态库存在未满足依赖：%s\n' "${SDK_LIBRARY}" >&2
  printf '%s\n' "${LDD_OUTPUT}" >&2
  exit 1
fi

printf '%s\n' '4/4 SDK 本地加载检查（不调用 connect，不访问机器人）'
ros2 run marvin_hardware_backend marvin_backend_probe

printf '%s\n' \
  'Marvin 中间通信后端检查通过；未连接机器人，也未发送任何关节命令。'
