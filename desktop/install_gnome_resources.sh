#!/usr/bin/env bash

# CARKit learning annotation: orchestrates a repeatable CARKit command-line workflow.
set -euo pipefail

UUID="carkit-resources@carkit.local"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${SCRIPT_DIR}/gnome-shell/${UUID}"
DATA_HOME="${XDG_DATA_HOME:-${HOME}/.local/share}"
TARGET_DIR="${DATA_HOME}/gnome-shell/extensions/${UUID}"

if ! command -v gnome-extensions >/dev/null 2>&1; then
  echo "CARKit error: gnome-extensions is not installed." >&2
  exit 1
fi

SHELL_VERSION="$(gnome-shell --version | awk '{print $3}' | cut -d. -f1)"
if [ "${SHELL_VERSION}" != "46" ]; then
  echo "CARKit error: this extension targets GNOME Shell 46, found ${SHELL_VERSION}." >&2
  exit 1
fi

# GNOME's global Extensions switch can remain off even when this UUID is in
# enabled-extensions.  Set it explicitly so logout/login does not leave the
# indicator discovered but suppressed.
gsettings set org.gnome.shell disable-user-extensions false

install -d -m 0755 "${TARGET_DIR}"
install -m 0644 "${SOURCE_DIR}/metadata.json" "${TARGET_DIR}/metadata.json"
install -m 0644 "${SOURCE_DIR}/extension.js" "${TARGET_DIR}/extension.js"
install -m 0644 "${SOURCE_DIR}/stylesheet.css" "${TARGET_DIR}/stylesheet.css"

# Keep the extension in GNOME's persistent enabled set. GNOME 46 caches loaded
# extension modules, so updated source is picked up by the next Shell restart.
if ! gnome-extensions enable "${UUID}" 2>/dev/null; then
  enabled="$(gsettings get org.gnome.shell enabled-extensions)"
  updated="$(python3 - "${UUID}" "${enabled}" <<'PY'
import ast
import sys

uuid = sys.argv[1]
extensions = ast.literal_eval(sys.argv[2].removeprefix("@as "))
if uuid not in extensions:
    extensions.append(uuid)
print(repr(extensions))
PY
)"
  gsettings set org.gnome.shell enabled-extensions "${updated}"
fi

# The CLI can exit successfully while GNOME's global Extensions switch is off,
# so verify the state reported by the running Shell instead of its exit code.
sleep 1
extension_info="$(gnome-extensions info "${UUID}" 2>/dev/null || true)"
if grep -q 'State: ACTIVE' <<<"${extension_info}"; then
  echo "CARKit Resources is active in the GNOME top bar."
else
  printf '%s\n' \
    "CARKit Resources was installed at ${TARGET_DIR}." \
    "User extensions are enabled and CARKit is marked enabled, but the running Shell has not loaded it." \
    "On X11 press Alt+F2, enter r. On Wayland log out and back in."
fi
