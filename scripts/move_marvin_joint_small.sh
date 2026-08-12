#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_SETUP="${PROJECT_ROOT}/install/setup.bash"

SHOW_HELP=false
CONFIRMED=false
for argument in "$@"; do
  case "${argument}" in
    -h|--help)
      SHOW_HELP=true
      ;;
    --confirm-motion)
      CONFIRMED=true
      ;;
  esac
done

if [[ "${SHOW_HELP}" != true && "${CONFIRMED}" != true ]]; then
  printf '%s\n' \
    '拒绝下发：必须显式提供 --confirm-motion。' \
    '该验收会让指定的单臂单关节小幅移动并返回实测初始位置。' >&2
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
  exec ros2 run marvin_hardware_backend marvin_small_motion --help
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
  '警告：这是真机单关节小幅往返运动验收。' \
  '请确认指定关节和正负方向的完整扫掠空间无人员、线缆和障碍物。' \
  '请关闭其他 SDK 会话，并将实体急停置于随手可按位置。' \
  '程序不会回配置 Home；它会从实测位置出发并返回该实测位置。'

exec ros2 run marvin_hardware_backend marvin_small_motion "$@"
