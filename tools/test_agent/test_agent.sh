#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SESS_NAME="test_agent"
SESS_ROOT="${ROOT_DIR}/tools/test_agent/sessions"

timestamp() { date +"%Y%m%d_%H%M%S"; }

usage() {
  cat <<'EOF'
Usage:
  tools/test_agent/test_agent.sh start --controller {pp|rpp|dwb} [--session NAME]
  tools/test_agent/test_agent.sh attach [--session NAME]
  tools/test_agent/test_agent.sh stop   [--session NAME]
  tools/test_agent/test_agent.sh status [--session NAME]
  tools/test_agent/test_agent.sh report

Notes:
  - Requires ROS 2 Humble + workspace overlays sourced before start.
  - Publishes/uses waypoint_editor path topic: waypoint_path (latched).
EOF
}

session_dir_for() {
  local controller="$1"
  local dir="${SESS_ROOT}/$(timestamp)_${controller}"
  mkdir -p "${dir}"
  echo "${dir}"
}

start() {
  local controller=""
  local session="${SESS_NAME}"

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --controller) controller="$2"; shift 2;;
      --session) session="$2"; shift 2;;
      -h|--help) usage; exit 0;;
      *) echo "Unknown arg: $1" >&2; usage; exit 2;;
    esac
  done
  if [[ -z "${controller}" ]]; then
    echo "--controller is required" >&2
    exit 2
  fi

  local dir
  dir="$(session_dir_for "${controller}")"
  ln -sfn "${dir}" "${SESS_ROOT}/latest"

  local base_params="${ROOT_DIR}/src/robot_simulation/wpr_simulation2/config/nav2_params.yaml"
  local gen_params="${dir}/nav2_params.generated.yaml"
  python3 "${ROOT_DIR}/tools/test_agent/gen_nav2_params.py" --base "${base_params}" --controller "${controller}" --out "${gen_params}"

  local map_file="${ROOT_DIR}/src/robot_simulation/wpr_simulation2/maps/map.yaml"
  local sim_launch="wpr_simulation2 robocup_home.launch.py"
  local env_setup="source /opt/ros/humble/setup.bash; if [ -f \"${ROOT_DIR}/install/setup.bash\" ]; then source \"${ROOT_DIR}/install/setup.bash\"; fi"

  # Kill if exists
  tmux has-session -t "${session}" 2>/dev/null && tmux kill-session -t "${session}"

  tmux new-session -d -s "${session}" -n sim "bash -lc '${env_setup}; cd \"${ROOT_DIR}\"; echo \"[sim] launching...\"; ros2 launch ${sim_launch}'"
  tmux new-window -t "${session}" -n nav2 "bash -lc '${env_setup}; cd \"${ROOT_DIR}\"; echo \"[nav2] launching...\"; ros2 launch nav2_bringup bringup_launch.py map:=\"${map_file}\" use_sim_time:=true params_file:=\"${gen_params}\"'"
  tmux new-window -t "${session}" -n follow "bash -lc '${env_setup}; cd \"${ROOT_DIR}\"; echo \"[follow] waiting for path + sending FollowPath goals...\"; python3 tools/test_agent/follow_path_on_topic.py --path-topic waypoint_path --action-name follow_path --controller-id FollowPath --goal-checker-id general_goal_checker 2>&1 | tee \"${dir}/follow_path_on_topic.log\"'"
  tmux new-window -t "${session}" -n eval "bash -lc '${env_setup}; cd \"${ROOT_DIR}\"; echo \"[eval] evaluating tracking...\"; python3 tools/test_agent/path_tracking_eval.py --path-topic waypoint_path --cmd-vel-topic /cmd_vel --report \"${dir}/report.md\" 2>&1 | tee \"${dir}/path_tracking_eval.log\"'"

  tmux select-window -t "${session}:sim"
  echo "Started tmux session: ${session}"
  echo "Session dir: ${dir}"
  echo "Attach: tools/test_agent/test_agent.sh attach --session ${session}"
}

attach() {
  local session="${SESS_NAME}"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --session) session="$2"; shift 2;;
      -h|--help) usage; exit 0;;
      *) echo "Unknown arg: $1" >&2; usage; exit 2;;
    esac
  done
  tmux attach -t "${session}"
}

stop() {
  local session="${SESS_NAME}"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --session) session="$2"; shift 2;;
      -h|--help) usage; exit 0;;
      *) echo "Unknown arg: $1" >&2; usage; exit 2;;
    esac
  done
  tmux has-session -t "${session}" 2>/dev/null && tmux kill-session -t "${session}" || true
  echo "Stopped tmux session: ${session}"
}

status() {
  local session="${SESS_NAME}"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --session) session="$2"; shift 2;;
      -h|--help) usage; exit 0;;
      *) echo "Unknown arg: $1" >&2; usage; exit 2;;
    esac
  done
  if ! tmux has-session -t "${session}" 2>/dev/null; then
    echo "tmux session not running: ${session}"
    exit 0
  fi
  tmux list-windows -t "${session}"
  echo "Latest session dir: ${SESS_ROOT}/latest"
}

report() {
  local report="${SESS_ROOT}/latest/report.md"
  if [[ ! -f "${report}" ]]; then
    echo "No report yet: ${report}"
    exit 1
  fi
  echo "${report}"
}

cmd="${1:-}"
shift || true
case "${cmd}" in
  start) start "$@";;
  attach) attach "$@";;
  stop) stop "$@";;
  status) status "$@";;
  report) report "$@";;
  ""|-h|--help) usage;;
  *) echo "Unknown command: ${cmd}" >&2; usage; exit 2;;
esac
