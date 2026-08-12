#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_SETUP="${PROJECT_ROOT}/install/setup.bash"
CONFIG_FILE="${PROJECT_ROOT}/install/marvin_hardware_backend/share/marvin_hardware_backend/config/real.yaml"

if [[ "${1:-}" != "--confirm-real" ]]; then
  printf '%s\n' \
    '拒绝启动：必须显式提供 --confirm-real。' \
    '该命令会在应用层安全门满足后连接并驱动 Marvin 真机。' >&2
  exit 2
fi
shift

if [[ ! -f "${INSTALL_SETUP}" || ! -f "${CONFIG_FILE}" ]]; then
  printf '%s\n' \
    '错误：Marvin 后端尚未构建，请先执行 pixi run build-marvin。' >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${INSTALL_SETUP}"
set -u

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}"
LOCK_FILE="${RUNTIME_DIR}/marvin-hardware-backend-${UID}.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  printf '%s\n' '拒绝启动：已有 Marvin 真机后端持有运行锁。' >&2
  exit 1
fi

printf '%s\n' \
  '警告：即将启动 Marvin 真机通信后端。' \
  '请确认急停、48V、网卡地址、工作空间和控制器独占状态。'

exec ros2 run marvin_hardware_backend marvin_hardware_bridge \
  --confirm-real \
  --ros-args \
  --params-file "${CONFIG_FILE}" \
  "$@"
