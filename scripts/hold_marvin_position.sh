#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_SETUP="${PROJECT_ROOT}/install/setup.bash"

SHOW_HELP=false
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  SHOW_HELP=true
elif [[ "${1:-}" != "--confirm-hold" ]]; then
  printf '%s\n' \
    '拒绝下发：必须显式提供 --confirm-hold。' \
    '该验收会进入 state=1，并把读取到的当前位置经过安全桥重新下发。' >&2
  exit 2
fi

if [[ ! -f "${INSTALL_SETUP}" ]]; then
  printf '%s\n' \
    '错误：Marvin 后端尚未构建，请先执行 pixi run build-marvin。' >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${INSTALL_SETUP}"
set -u

if [[ "${SHOW_HELP}" == true ]]; then
  exec ros2 run marvin_hardware_backend marvin_hold_position --help
fi

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}"
LOCK_FILE="${RUNTIME_DIR}/marvin-hardware-backend-${UID}.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  printf '%s\n' \
    '拒绝下发：另一个 Marvin 反馈或控制进程正在占用 SDK。' >&2
  exit 1
fi

printf '%s\n' \
  '警告：这是第一条真机下行位置命令验收。' \
  '程序将读取当前位置、进入 state=1，并只保持该位置。' \
  '请关闭其他 SDK 会话，清空工作空间，并将实体急停置于随手可按位置。' \
  '本程序不会回 Home，也不会添加关节偏移。'

exec ros2 run marvin_hardware_backend marvin_hold_position "$@"

