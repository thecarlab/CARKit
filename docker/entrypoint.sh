#!/usr/bin/env bash

set -e

if [ "$(id -u)" -eq 0 ] && [ "${CARKIT_RUN_AS_ROOT:-0}" != "1" ] && [ -n "${CARKIT_HOST_UID:-}" ]; then
  host_uid="${CARKIT_HOST_UID}"
  host_gid="${CARKIT_HOST_GID:-${CARKIT_HOST_UID}}"
  host_user="${CARKIT_HOST_USER:-carkit}"

  group_name="$(getent group "${host_gid}" | cut -d: -f1 || true)"
  if [ -z "${group_name}" ]; then
    group_name="${host_user}"
    groupadd --gid "${host_gid}" "${group_name}" 2>/dev/null || group_name="$(getent group "${host_gid}" | cut -d: -f1)"
  fi

  user_name="$(getent passwd "${host_uid}" | cut -d: -f1 || true)"
  if [ -z "${user_name}" ]; then
    user_name="${host_user}"
    if getent passwd "${user_name}" >/dev/null; then
      user_name="carkit"
    fi
    useradd --uid "${host_uid}" --gid "${host_gid}" --create-home --shell /bin/bash "${user_name}"
  fi

  for group in sudo dialout video plugdev input render; do
    if getent group "${group}" >/dev/null; then
      usermod -aG "${group}" "${user_name}" || true
    fi
  done
  printf '%s ALL=(ALL) NOPASSWD:ALL\n' "${user_name}" > /etc/sudoers.d/carkit-host-user
  chmod 0440 /etc/sudoers.d/carkit-host-user

  if [ "${CARKIT_FIX_PERMISSIONS_ON_START:-1}" = "1" ] && [ -d /workspaces/CARKit ]; then
    for path in \
      /workspaces/CARKit/build \
      /workspaces/CARKit/install \
      /workspaces/CARKit/log \
      /workspaces/CARKit/map \
      /workspaces/CARKit/map.pcd \
      /workspaces/CARKit/pose_graph.g2o \
      /workspaces/CARKit/carkit/sensors/realsense-ros \
      /workspaces/CARKit/carkit/sensors/sllidar_ros2
    do
      if [ -e "${path}" ]; then
        chown -R "${host_uid}:${host_gid}" "${path}" || true
      fi
    done
  fi

  exec sudo -E -H -u "${user_name}" "$0" "$@"
fi

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

if [ -f /workspaces/CARKit/install/setup.bash ]; then
  source /workspaces/CARKit/install/setup.bash
fi

exec "$@"
