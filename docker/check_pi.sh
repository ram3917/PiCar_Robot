#!/usr/bin/env bash
# Checks that a Raspberry Pi is ready for the PiCar stack and says what to fix.
# It changes nothing on the system: run it, fix what it reports, run it again.
#
#   docker/check_pi.sh
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

if [ -e /dev/gpiochip0 ]; then
    ok "/dev/gpiochip0 present"
else
    fix "/dev/gpiochip0 missing; is this a Raspberry Pi with a current kernel?"
fi

if [ -e /dev/i2c-1 ]; then
    ok "/dev/i2c-1 present"
else
    fix "no /dev/i2c-1; enable I2C with: sudo raspi-config nonint do_i2c 0, then reboot"
fi

# A live 'dtparam i2c_arm=on' creates /dev/i2c-1 but is lost on the next reboot.
if grep -qs '^dtparam=i2c_arm=on' /boot/firmware/config.txt /boot/config.txt; then
    ok "I2C is enabled in config.txt (survives a reboot)"
else
    fix "I2C is not enabled in config.txt; add dtparam=i2c_arm=on (sudo raspi-config nonint do_i2c 0)"
fi

if command -v docker >/dev/null 2>&1; then
    ok "docker installed"
else
    fix "docker not installed; see README.md (Deploy to the Pi)"
fi

if docker compose version >/dev/null 2>&1; then
    ok "docker compose plugin installed"
else
    fix "docker compose plugin missing; see README.md (Deploy to the Pi)"
fi

if id -nG | tr ' ' '\n' | grep -qx docker; then
    ok "user is in the docker group"
else
    fix "add your user to the docker group: sudo usermod -aG docker \"\$USER\", then log out and back in"
fi

if [ "$problems" -eq 0 ]; then
    echo "Ready. Next: load the image and start it, see README.md (Deploy to the Pi)"
else
    echo "${problems} item(s) to fix, then run this again."
    exit 1
fi
