#!/usr/bin/env bash
# Install the WiVRn PC server/dashboard on Ubuntu 24.04.
#
# WiVRn's headset-side client is an Android application.  After this script
# completes, launch the dashboard and use its wizard to install a version that
# exactly matches the server onto the headset.

set -euo pipefail

if [[ $EUID -eq 0 ]]; then
    echo "Run this script as the desktop user, without sudo." >&2
    exit 1
fi

if [[ "$(. /etc/os-release && printf '%s' "$ID")" != "ubuntu" ]] || \
   [[ "$(. /etc/os-release && printf '%s' "$VERSION_ID")" != "24.04" ]]; then
    echo "This installer supports Ubuntu 24.04 only." >&2
    exit 1
fi

if ! command -v sudo >/dev/null; then
    echo "sudo is required to install Ubuntu packages and start Avahi." >&2
    exit 1
fi

script_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

sudo apt update
sudo apt install --yes \
    flatpak avahi-daemon android-sdk-platform-tools \
    libopenxr-dev glslang-tools

# Ubuntu 24.04's packaged Android rules do not include Meta/Oculus vendor 2833.
# Install the deliberately device-scoped mode-0666 rule used for Quest ADB.
sudo install -o root -g root -m 0644 \
    "$script_directory/51-oculus.rules" \
    /etc/udev/rules.d/51-oculus.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=2833

# The user installation avoids adding the WiVRn Flatpak to every account.
flatpak remote-add --if-not-exists --user flathub \
    https://dl.flathub.org/repo/flathub.flatpakrepo
flatpak install --user --noninteractive --assumeyes flathub \
    io.github.wivrn.wivrn

# WiVRn uses Avahi/mDNS to find the headset on the local network.
sudo systemctl enable --now avahi-daemon

cat <<EOF

WiVRn is installed. Start its setup wizard with:
  flatpak run io.github.wivrn.wivrn

The wizard installs or links to the headset-side WiVRn client.  Keep its
version matched to this server.  To confirm USB debugging for a sideloaded
client, connect the headset and run:
  adb devices

For a wired connection, accept the headset's USB-debugging prompt and run:
  $script_directory/wivrn-usb.sh

The USB rule uses MODE=0666 for Meta/Oculus vendor 2833.  Unplug and reconnect
the headset if adb still reports "no permissions" after installation.

If UFW is enabled, allow WiVRn discovery and traffic:
  sudo ufw allow 5353/udp
  sudo ufw allow 9757
EOF
