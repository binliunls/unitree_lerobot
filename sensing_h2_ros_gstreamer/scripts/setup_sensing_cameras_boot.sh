#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cyclonedds_config="${repo_root}/config/cyclonedds/image_streams.xml"
sysctl_config="/etc/sysctl.d/99-cyclonedds-image-streams.conf"

unitree_i2c_script="${UNITREE_I2C_SCRIPT:-${HOME}/.Unitree/set_i2c3.sh}"
unitree_modules_dir="${UNITREE_MODULES_DIR:-${HOME}/.Unitree/YUSHU_4A_AGTH_G2Y_7.1}"
unitree_modules_script="${UNITREE_MODULES_SCRIPT:-${unitree_modules_dir}/load_modules.sh}"
head_trigger_cameras="${HEAD_TRIGGER_CAMERAS:-0 1}"

log() {
  echo "[sensing-setup] $*"
}

ensure_apt_package() {
  local package="$1"
  local status

  status="$(dpkg-query -W -f='${Status}' "${package}" 2>/dev/null || true)"
  if [[ "${status}" == "install ok installed" ]]; then
    log "${package} is installed"
    return
  fi

  log "Installing ${package}"
  sudo apt-get update
  sudo apt-get install -y "${package}"
}

configure_cyclonedds() {
  if [[ ! -f "${cyclonedds_config}" ]]; then
    echo "CycloneDDS config not found: ${cyclonedds_config}" >&2
    exit 2
  fi

  ensure_apt_package ros-jazzy-rmw-cyclonedds-cpp

  log "Configuring ROS 2 image-stream receive buffers"
  sudo tee "${sysctl_config}" >/dev/null <<'EOF'
net.core.rmem_max=33554432
net.core.rmem_default=16777216
EOF
  sudo sysctl -w net.core.rmem_max=33554432 >/dev/null
  sudo sysctl -w net.core.rmem_default=16777216 >/dev/null
}

run_unitree_camera_setup() {
  if [[ ! -f "${unitree_i2c_script}" ]]; then
    echo "Unitree I2C setup script not found: ${unitree_i2c_script}" >&2
    exit 2
  fi
  if [[ ! -f "${unitree_modules_script}" ]]; then
    echo "Unitree camera module script not found: ${unitree_modules_script}" >&2
    exit 2
  fi

  log "Running Unitree I2C setup"
  bash "${unitree_i2c_script}"

  log "Loading Unitree camera modules"
  (cd "${unitree_modules_dir}" && bash "${unitree_modules_script}")
}

verify_head_trigger_mode_in_modules_script() {
  for camera_index in ${head_trigger_cameras}; do
    local device="/dev/video${camera_index}"
    local expected="-d ${device} -c sensor_mode=0,trig_pin=0x00020007,trig_mode=1"

    if ! grep -F -- "${expected}" "${unitree_modules_script}" >/dev/null; then
      echo "Unitree module script does not enable trigger mode for ${device}." >&2
      echo "Expected ${unitree_modules_script} to contain:" >&2
      echo "  v4l2-ctl ${expected}" >&2
      exit 2
    fi

    log "Verified trigger mode is configured by load_modules.sh for ${device}"
  done
}

prompt_argus_daemon_override() {
  if [[ ! -t 0 ]]; then
    log "Skipping nvargus-daemon override prompt because stdin is not interactive"
    return
  fi

  echo
  read -r -p "Apply the one-time nvargus-daemon systemd override now? This restarts nvargus-daemon. [y/N] " reply
  case "${reply}" in
    [Yy]|[Yy][Ee][Ss])
      log "Applying nvargus-daemon systemd override"
      sudo install -d -m 0755 /etc/systemd/system/nvargus-daemon.service.d
      sudo tee /etc/systemd/system/nvargus-daemon.service.d/sensing-cameras.conf >/dev/null <<'EOF'
[Service]
Environment="NVCAMERA_NITO_PATH=CONFIG"
Environment="enableCamInfiniteTimeout=1"
EOF
      sudo systemctl daemon-reload
      sudo systemctl restart nvargus-daemon.service
      sudo systemctl status --no-pager nvargus-daemon.service
      ;;
    *)
      log "Skipped nvargus-daemon override"
      ;;
  esac
}

configure_cyclonedds
verify_head_trigger_mode_in_modules_script
run_unitree_camera_setup

log "Boot setup complete"
log "Launch scripts default to RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
log "Launch scripts default to CYCLONEDDS_URI=file://${cyclonedds_config}"
prompt_argus_daemon_override
