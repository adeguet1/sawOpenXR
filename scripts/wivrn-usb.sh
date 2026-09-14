#!/usr/bin/env bash
# Force an authorized Android headset to connect to WiVRn over USB/ADB.

set -euo pipefail

if [[ $EUID -eq 0 ]]; then
    echo "Run this script as the desktop user, without sudo." >&2
    exit 1
fi

if ! command -v adb >/dev/null; then
    echo "adb is required; run install-wivrn-ubuntu-24.04.sh first." >&2
    exit 1
fi

adb start-server

devices="$(adb devices)"
if grep -q '[[:space:]]unauthorized\([[:space:]]\|$\)' <<<"$devices"; then
    echo "The headset is unauthorized. Put it on and accept the USB-debugging prompt." >&2
    exit 1
fi
if grep -q '[[:space:]]no permissions\([[:space:]]\|$\)' <<<"$devices"; then
    echo "ADB cannot access the headset. Install the supplied udev rule and reconnect it." >&2
    exit 1
fi

device_count="$(awk '$2 == "device" {count++} END {print count + 0}' <<<"$devices")"
if [[ $device_count -eq 0 ]]; then
    echo "No authorized ADB headset found. Connect it and run 'adb devices -l'." >&2
    exit 1
fi
if [[ $device_count -gt 1 && -z ${ANDROID_SERIAL:-} ]]; then
    echo "Multiple ADB devices found; set ANDROID_SERIAL to the headset serial." >&2
    exit 1
fi

package_name="${1:-}"
if [[ -z $package_name ]]; then
    installed_packages="$(adb shell pm list packages | sed -n 's/^package://p' | tr -d '\r')"
    candidates=(
        org.meumeu.wivrn
        org.meumeu.wivrn.github
        org.meumeu.wivrn.github.nighly
        org.meumeu.wivrn.github.testing
        org.meumeu.wivrn.local
    )
    for candidate in "${candidates[@]}"; do
        if grep -Fxq "$candidate" <<<"$installed_packages"; then
            package_name="$candidate"
            break
        fi
    done
fi

if [[ -z $package_name ]]; then
    echo "No WiVRn headset package found. Pass its package name as the first argument." >&2
    echo "Installed candidates can be listed with: adb shell pm list packages | grep wivrn" >&2
    exit 1
fi

package_path="$(adb shell pm path "$package_name" 2>/dev/null | tr -d '\r')"
if [[ $package_path != package:* ]]; then
    echo "WiVRn package '$package_name' is not installed on the headset." >&2
    exit 1
fi

adb shell am force-stop "$package_name"
adb reverse --remove tcp:9757 >/dev/null 2>&1 || true
adb reverse tcp:9757 tcp:9757

# Removing Wi-Fi makes use of the localhost ADB tunnel unambiguous.
if ! adb shell svc wifi disable; then
    echo "Warning: Android rejected the request to disable Wi-Fi." >&2
    echo "Disable Wi-Fi manually in the headset before continuing." >&2
fi
wifi_state="$(adb shell settings get global wifi_on | tr -d '\r')"
if [[ $wifi_state != "0" ]]; then
    echo "Warning: Android did not report Wi-Fi as disabled (wifi_on=$wifi_state)." >&2
    echo "Disable Wi-Fi manually in the headset before continuing." >&2
fi

adb shell am start \
    -a android.intent.action.VIEW \
    -d "wivrn+tcp://localhost" \
    "$package_name"

echo
echo "WiVRn USB tunnel active for package: $package_name"
adb reverse --list
echo "Headset Wi-Fi state (0 is disabled): $wifi_state"
echo "To restore Wi-Fi later: adb shell svc wifi enable"
