#!/bin/bash
# YAM arm CAN setup. Bind each USB-CAN adapter to a stable name by serial.
#   setup            install deps, gs_usb driver rule, Python deps (run once)
#   list             show connected adapters: interface, serial, registered name
#   add <name> [serial]  bind an adapter to <name> (e.g. can_left), auto bring-up
set -e

RULES_DRIVER="/etc/udev/rules.d/80-yam-can-driver.rules"
RULES_NAMES="/etc/udev/rules.d/90-yam-can.rules"
BITRATE=1000000

if [ "$(id -u)" != "0" ]; then SUDO="sudo"; else SUDO=""; fi

_can_ifaces() {
    for d in /sys/class/net/*; do
        [ -e "$d/type" ] || continue
        [ "$(cat "$d/type")" = "280" ] && basename "$d"
    done
}

_serial_of() {
    udevadm info -a -p "/sys/class/net/$1" 2>/dev/null \
        | grep -m1 'ATTRS{serial}==' \
        | sed -E 's/.*ATTRS\{serial\}=="([^"]+)".*/\1/'
}

_registered_name() {
    [ -n "$1" ] && [ -f "$RULES_NAMES" ] || return 0
    grep -F "ATTRS{serial}==\"$1\"" "$RULES_NAMES" 2>/dev/null \
        | sed -E 's/.*NAME="([^"]+)".*/\1/' | head -1
}

cmd_setup() {
    $SUDO apt-get update
    $SUDO apt-get install -y can-utils build-essential
    $SUDO tee "$RULES_DRIVER" > /dev/null << 'EOF'
ACTION=="add|change", SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="606f", RUN+="/sbin/modprobe gs_usb"
ACTION=="add|change", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", RUN+="/bin/sh -c 'echo 1d50 606f > /sys/bus/usb/drivers/gs_usb/new_id'"
EOF
    $SUDO udevadm control --reload-rules && $SUDO udevadm trigger
    $SUDO adduser "$(logname)" plugdev || true
    uv pip install -e ".[robots]"
    echo "[OK] host setup complete. Register arms with: bash scripts/yam.sh add can_<role>"
}

cmd_list() {
    printf '%-14s %-28s %s\n' "INTERFACE" "SERIAL" "REGISTERED_AS"
    for ifc in $(_can_ifaces); do
        serial="$(_serial_of "$ifc")"
        printf '%-14s %-28s %s\n' "$ifc" "${serial:-?}" "$(_registered_name "$serial" || true)"
    done
}

cmd_add() {
    name="$1"
    serial="$2"
    [ -n "$name" ] || { echo "usage: yam.sh add <can_name> [serial]"; exit 1; }
    case "$name" in can*) ;; *) echo "error: name must start with 'can'"; exit 1 ;; esac
    [ "${#name}" -le 13 ] || { echo "error: name must be <= 13 characters"; exit 1; }

    if [ -z "$serial" ]; then
        found=""
        for ifc in $(_can_ifaces); do
            s="$(_serial_of "$ifc")"
            [ -n "$s" ] && [ -z "$(_registered_name "$s")" ] && found="$found $s"
        done
        set -- $found
        if [ "$#" -eq 0 ]; then
            echo "error: no unregistered CAN adapter found. Plug in the arm, or run: bash scripts/yam.sh list"
            exit 1
        elif [ "$#" -gt 1 ]; then
            echo "error: $# unregistered adapters present; pass a serial: yam.sh add $name <serial> (see: yam.sh list)"
            exit 1
        fi
        serial="$1"
    fi

    if [ -f "$RULES_NAMES" ]; then
        $SUDO sed -i "/ATTRS{serial}==\"$serial\"/d;/NAME=\"$name\"/d" "$RULES_NAMES"
    fi
    rule="SUBSYSTEM==\"net\", ACTION==\"add\", ATTRS{serial}==\"$serial\", NAME=\"$name\""
    rule="$rule, RUN+=\"/sbin/ip link set \$name type can bitrate $BITRATE\", RUN+=\"/sbin/ip link set \$name up\""
    echo "$rule" | $SUDO tee -a "$RULES_NAMES" > /dev/null

    $SUDO udevadm control --reload-rules
    $SUDO udevadm trigger
    $SUDO udevadm settle || true
    $SUDO ip link set "$name" down 2>/dev/null || true
    $SUDO ip link set "$name" up type can bitrate "$BITRATE" 2>/dev/null || true
    echo "[OK] bound serial $serial -> $name. Replug the arm if '$name' is missing from: ip link show"
}

case "${1:-}" in
    setup) cmd_setup ;;
    list)  cmd_list ;;
    add)   shift; cmd_add "$@" ;;
    *)     echo "usage: bash scripts/yam.sh {setup | list | add <name> [serial]}"; exit 1 ;;
esac
