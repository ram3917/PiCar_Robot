#!/usr/bin/env bash
# Checks that a Raspberry Pi is ready for the PiCar stack and says what to fix.
# It changes nothing on the system: run it, fix what it reports, run it again.
#
#   ./scripts/setup_pi.sh
set -uo pipefail

problems=0
ok() { echo "  ok    $*"; }
fix() {
    echo "  FIX   $*"
    problems=$((problems + 1))
}

echo "Checking the Pi"

arch="$(uname -m)"
if [ "$arch" = "aarch64" ]; then
    ok "64-bit OS (${arch})"
else
    fix "architecture is ${arch}; reflash Raspberry Pi OS 64-bit (Lite)"
fi

if [ -e /dev/gpiomem ]; then
    ok "/dev/gpiomem present"
else
    fix "/dev/gpiomem missing; is this a Raspberry Pi?"
fi

if ls /dev/i2c-* >/dev/null 2>&1; then
    ok "I2C device present"
else
    fix "no /dev/i2c-* device; enable I2C with: sudo raspi-config (Interface Options)"
fi

if command -v docker >/dev/null 2>&1; then
    ok "docker installed"
else
    fix "docker not installed; see docs/deployment.md"
fi

if docker compose version >/dev/null 2>&1; then
    ok "docker compose plugin installed"
else
    fix "docker compose plugin missing; see docs/deployment.md"
fi

if id -nG | tr ' ' '\n' | grep -qx docker; then
    ok "user is in the docker group"
else
    fix "add your user to the docker group: sudo usermod -aG docker \"\$USER\", then log out and back in"
fi

if [ "$problems" -eq 0 ]; then
    echo "Ready. Next: ./scripts/update.sh"
else
    echo "${problems} item(s) to fix, then run this again."
    exit 1
fi
