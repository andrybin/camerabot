#!/usr/bin/env bash
set -euo pipefail

# Unified installer for Raspberry Pi Camera on Ubuntu 22.04 (RPi 5)
# - Installs deps
# - Builds Raspberry Pi libcamera (user prefix)
# - Builds kmsxx/pykms (user prefix)
# - Sets environment in ~/.profile
# - Verifies detection and headless capture

main() {
  start_dir=$(pwd)
  REBUILD=${REBUILD:-0}
  echo "[1/7] Checking group membership (video)";
  if id -nG | tr ' ' '\n' | grep -qx video; then
    echo "  - User is in video group";
  else
    echo "  - WARNING: user not in video group (camera access may fail)";
    echo "    Hint: sudo usermod -aG video $USER && newgrp video"
  fi

  echo "[2/7] Installing apt dependencies";
  sudo apt update
  sudo apt install -y \
    libcamera0 libcamera-tools libcamera-dev v4l-utils \
    git meson ninja-build pkg-config build-essential python3-dev python3-pybind11 \
    libglib2.0-dev libgnutls28-dev openssl libdrm-dev libjpeg-dev libtiff5-dev \
    libexif-dev libevent-dev libexpat1-dev libunwind-dev libudev-dev libyaml-dev \
    python3-yaml python3-jinja2 python3-ply \
    python3-pip libcap-dev python3-prctl

  echo "[3/7] Ensuring modern Meson via pip (user)";
  python3 -m pip install --user --upgrade 'meson>=0.63.3' meson-python

  echo "[4/7] Building and installing Raspberry Pi libcamera (user prefix)";
  cd "$HOME"
  if [ ! -d libcamera-rpi ]; then
    git clone --depth=1 https://github.com/raspberrypi/libcamera.git libcamera-rpi
  fi
  cd libcamera-rpi
  meson setup build --buildtype=release -Dpipelines=all -Dpycamera=enabled --prefix="$HOME/.local"
  ninja -C build -j"$(nproc)"
  ninja -C build install

  echo "[5/7] Building and installing kmsxx with pykms (user prefix)";
  cd "$HOME"
  if [ ! -d kmsxx ]; then
    git clone --depth=1 https://github.com/tomba/kmsxx.git
  fi
  cd kmsxx
  meson setup build --buildtype=release -Dpykms=enabled --prefix="$HOME/.local"
  ninja -C build -j"$(nproc)"
  ninja -C build install

  echo "[6/7] Updating ~/.profile environment"
  pyver=$(python3 -c 'import sys;print(f"{sys.version_info.major}.{sys.version_info.minor}")')
  arch=$(python3 - <<'PY'
import sysconfig
print(sysconfig.get_config_var('MULTIARCH') or 'aarch64-linux-gnu')
PY
  )
  ensure_profile_line "export PATH=\"$HOME/.local/bin:$PATH\""
  ensure_profile_line "export LD_LIBRARY_PATH=\"$HOME/.local/lib:$HOME/.local/lib/${arch}:\${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\""
  ensure_profile_line "export PYTHONPATH=\"$HOME/.local/lib/python${pyver}/site-packages:$HOME/.local/lib/python3/dist-packages:$HOME/.local/lib/${arch}/python${pyver}/site-packages:\${PYTHONPATH:+:$PYTHONPATH}\""

  # Export for current shell
  export PATH="$HOME/.local/bin:$PATH"
  export LD_LIBRARY_PATH="$HOME/.local/lib:$HOME/.local/lib/${arch}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export PYTHONPATH="$HOME/.local/lib/${arch}/python${pyver}/site-packages:$HOME/.local/lib/python${pyver}/site-packages:$HOME/.local/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"

  echo "[7/7] Installing Picamera2 and OpenCV (Python)"
  python3 -m pip install --user --upgrade picamera2 pillow simplejpeg || true
  sudo apt install -y python3-opencv || true

  echo "[8/8] Verifying detection and headless capture"
  if ! command -v cam >/dev/null 2>&1; then
    echo "  - ERROR: cam tool not found on PATH" >&2
  fi
  cam -l || true
  echo "  - If no cameras are listed, try rebooting: sudo reboot"

  # Optional: run repo test if present
  cd "$start_dir"
  if [ -f "cv2_test.py" ]; then
    echo "Running cv2_test.py (headless)"
    python3 cv2_test.py || true
  else
    echo "Note: cv2_test.py not found in current directory ($(pwd)). Skipping test run."
  fi

  echo "Done. Re-login or 'source ~/.profile' to persist environment.";
}

ensure_profile_line() {
  local line="$1"
  local profile="$HOME/.profile"
  grep -qxF "$line" "$profile" 2>/dev/null || echo "$line" >> "$profile"
}

main "$@"


