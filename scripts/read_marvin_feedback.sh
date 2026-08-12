#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_SETUP="${PROJECT_ROOT}/install/setup.bash"

SHOW_HELP=false
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  SHOW_HELP=true
elif [[ "${1:-}" != "--confirm-readonly" ]]; then
  printf '%s\n' \
    '拒绝连接：必须显式提供 --confirm-readonly。' \
    '该诊断只调用 connect、subscribe 和 release_robot，不发送控制命令。' >&2
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
  exec ros2 run marvin_hardware_backend marvin_feedback_probe --help
fi

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}"
LOCK_FILE="${RUNTIME_DIR}/marvin-hardware-backend-${UID}.lock"
exec 9>"${LOCK_FILE}"
if ! flock -n 9; then
  printf '%s\n' \
    '拒绝连接：另一个 Marvin 反馈或控制进程正在占用 SDK。' >&2
  exit 1
fi

printf '%s\n' \
  '即将建立 Marvin 只读诊断连接。' \
  '请关闭 FxStation、官方控制节点及其他 Marvin SDK 会话。' \
  '本程序不会清错、使能、设置模式、回零或发送关节目标。'

exec ros2 run marvin_hardware_backend marvin_feedback_probe "$@"
