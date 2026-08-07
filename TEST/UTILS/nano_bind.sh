#!/usr/bin/env bash
#
# bind_arduino.sh — Validated udev symlink generator for Arduino Nano 33 BLE
# Usage: sudo ./bind_arduino.sh [PORT] [CUSTOM_NAME] [--force]
# Example: sudo ./bind_arduino.sh /dev/ttyACM0 ttyNANO

set -euo pipefail

ARDUINO_VID="2341"  # Official Arduino SA Vendor ID
# Known Nano 33 BLE / BLE Sense PIDs (Main Sketch & Bootloader modes)
KNOWN_PIDS=("805a" "005a" "805b" "005b")

if [[ $EUID -ne 0 ]]; then
  echo "Error: This script must be run as root (use sudo)." >&2
  exit 1
fi

PORT="${1:-}"
CUSTOM_NAME="${2:-ttyNANO}"
FORCE_FLAG="${3:-}"

if [[ -z "$PORT" ]]; then
  echo "Usage: sudo $0 <serial-port> [custom-symlink-name] [--force]"
  echo "Example: sudo $0 /dev/ttyACM0 ttyNANO"
  echo ""
  echo "Currently connected USB serial ports:"
  ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  No ACM/USB serial ports found."
  exit 1
fi

if [[ ! -e "$PORT" ]]; then
  echo "Error: Device '$PORT' does not exist." >&2
  exit 1
fi

CUSTOM_NAME="${CUSTOM_NAME#/dev/}"

echo "Inspecting device on $PORT..."

# 1. Query hardware attributes via udevadm
VID=$(udevadm info -a -n "$PORT" | grep -m1 'ATTRS{idVendor}' | cut -d'"' -f2 || true)
PID=$(udevadm info -a -n "$PORT" | grep -m1 'ATTRS{idProduct}' | cut -d'"' -f2 || true)
MANUFACTURER=$(udevadm info -a -n "$PORT" | grep -m1 'ATTRS{manufacturer}' | cut -d'"' -f2 || true)
SERIAL=$(udevadm info -a -n "$PORT" | grep -m1 'ATTRS{serial}' | cut -d'"' -f2 || true)

echo "  -> Vendor ID (idVendor):    ${VID:-<unknown>}"
echo "  -> Product ID (idProduct):  ${PID:-<unknown>}"
echo "  -> Manufacturer:            ${MANUFACTURER:-<unknown>}"
echo "  -> Serial Number:           ${SERIAL:-<none>}"

# 2. Hardware Verification Check
IS_ARDUINO=false
if [[ "${VID,,}" == "${ARDUINO_VID,,}" ]]; then
  IS_ARDUINO=true
fi

IS_NANO33=false
for known_pid in "${KNOWN_PIDS[@]}"; do
  if [[ "${PID,,}" == "${known_pid,,}" ]]; then
    IS_NANO33=true
    break
  fi
done

# 3. Handle validation failures
if [[ "$FORCE_FLAG" != "--force" ]]; then
  if ! $IS_ARDUINO; then
    echo "" >&2
    echo "ERROR: Device at $PORT has VID '$VID' (Expected '$ARDUINO_VID' for Arduino)." >&2
    echo "Refusing to create rule for non-Arduino device." >&2
    echo "If you want to bind this non-Arduino device anyway, run with --force:" >&2
    echo "  sudo $0 $PORT $CUSTOM_NAME --force" >&2
    exit 1
  fi

  if ! $IS_NANO33; then
    echo "WARNING: Device has Arduino VID ($VID), but PID '$PID' is not a standard Nano 33 BLE PID." >&2
  fi
fi

# 4. Generate the udev rule
RULE_FILE="/etc/udev/rules.d/99-arduino-${CUSTOM_NAME}.rules"

if [[ -n "$SERIAL" ]]; then
  RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", ATTRS{serial}==\"$SERIAL\", SYMLINK+=\"$CUSTOM_NAME\", MODE=\"0666\""
else
  RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$VID\", ATTRS{idProduct}==\"$PID\", SYMLINK+=\"$CUSTOM_NAME\", MODE=\"0666\""
fi

echo "Writing rule to $RULE_FILE..."
echo "# Persistent symlink for Arduino Nano 33 BLE ($PORT)" > "$RULE_FILE"
echo "$RULE" >> "$RULE_FILE"

# 5. Reload rules
echo "Reloading udev..."
udevadm control --reload-rules
udevadm trigger

echo ""
echo "Success! Device at $PORT verified and bound to /dev/$CUSTOM_NAME."
