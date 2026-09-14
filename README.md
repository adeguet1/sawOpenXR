# sawOpenXR

`mtsOpenXR` is the SAW component intended to own one Quest/OpenXR session and
provide two `MTM_GENERIC` interfaces (`MTML`, `MTMR`) plus console inputs.

The supplied system JSON targets the dVRK Isaac patient cart over ROS. It is
for Isaac/simulation only: it must not be used to operate hardware until the
OpenXR action/render loop is connected to the cisst state tables and all
loss-of-tracking safety transitions are tested.

Video source configurations:

- `sawOpenXR-isaac-rtsp.json`: H.264 SBS RTSP from Isaac Sim.
- `sawOpenXR-dvrk-socket.json`: an existing dVRK abstract GStreamer socket.

Like `dvrk_data` and `dvrk_console`, each configuration uses a root-level
`gst_input` string.  The string contains the complete source, decode, queue,
and conversion pipeline, so latency properties can be tuned without rebuilding
`sawOpenXR`.  The component appends only its RGBA output caps and appsink.

The OpenXR console control mapping is:

- Controller poses use an `OpenXR_HRSV` frame whose origin is the live midpoint
  between the HMD eyes and whose axes follow the virtual video plane.  This
  matches the physical da Vinci stereo-viewer geometry used by ECM
  teleoperation.
- Hold right A or left X and move that controller to position the video
  window.  The global clutch is pressed while either button is held.  On
  release, both controller poses are expressed relative to the plane's new
  pose before the global clutch is released.
- Otherwise the global clutch is released; operator-present is always pressed.
- Pull both thumbsticks toward the user to press the camera pedal.  Releasing
  either thumbstick releases the camera pedal.
- Push a thumbstick at least 75 percent up/away to release the clutch for that
  controller's PSM only (left: PSM2; right: PSM1). Returning it below the
  threshold clutches that PSM. The index triggers remain the PSM jaws/grippers.

Build the ROS 2 package with colcon from the dVRK workspace:

```bash
cd $HOME/wss/dvrk
source install/setup.bash
colcon build --packages-select saw_openxr --symlink-install
source install/setup.bash
```

## WiVRn on Ubuntu 24.04

For a Quest or another Android-based standalone headset, WiVRn provides the
OpenXR runtime on the Linux PC; its streaming client is installed on the
headset.  Install the PC server/dashboard, Avahi discovery service, and `adb`
with:

```bash
cd $HOME/wss/dvrk/src/cisst-saw/sawOpenXR
./scripts/install-wivrn-ubuntu-24.04.sh
```

It installs the current stable WiVRn Flatpak into the invoking user's account
and enables Avahi.  Start `WiVRn server` from the desktop launcher, or from a
terminal with:

```bash
flatpak run io.github.wivrn.wivrn
```

Use its wizard to install the headset app.  The headset app and PC server must
use the same WiVRn version.
If UFW is enabled, allow `5353/udp` and `9757` as shown by the script.

### Wired WiVRn over USB

WiVRn's wired mode uses an ADB reverse TCP tunnel.  The installer copies
`scripts/51-oculus.rules` to `/etc/udev/rules.d`; this grants mode `0666` only
to USB devices with the Meta/Oculus vendor ID `2833`.  Enable developer mode
for the headset, reconnect its USB data cable, put on the headset, accept the
USB-debugging prompt, and verify that the state is `device`:

```bash
adb devices -l
```

Start the WiVRn dashboard, then force the headset onto USB with:

```bash
cd $HOME/wss/dvrk/src/cisst-saw/sawOpenXR
./scripts/wivrn-usb.sh
```

The helper detects the installed WiVRn package, stops and relaunches it using
`wivrn+tcp://localhost`, maps host and headset TCP port `9757`, and disables
headset Wi-Fi to prevent fallback.  It supports the Meta Store, GitHub release,
nightly/testing, and local package names.  An explicit package name can be
passed when more than one is installed:

```bash
./scripts/wivrn-usb.sh org.meumeu.wivrn.github
```

Verify the tunnel and disabled Wi-Fi state with:

```bash
adb reverse --list
adb shell settings get global wifi_on
```

The first command should include `tcp:9757 tcp:9757`; the second should print
`0`.  Recreate the tunnel after disconnecting the cable or rebooting the
headset.  Restore Wi-Fi with `adb shell svc wifi enable`.

### Build dependencies

The WiVRn Flatpak supplies the OpenXR runtime but not the headers and shader
compiler needed to build `saw_openxr`.  On Ubuntu 24.04 install:

```bash
sudo apt install libopenxr-dev glslang-tools
```

The dVRK workspace build dependencies normally provide the Vulkan and
GStreamer development packages.  After installing the packages above, build
the component with:

```bash
cd $HOME/wss/dvrk
source install/setup.bash
colcon build --packages-select saw_openxr --symlink-install
source install/setup.bash
```

Start the Isaac Sim stereo patient-cart scene and then the dVRK system with
its RTSP video configuration:

```bash
ros2 launch saw_openxr isaac_patient_cart_rtsp.launch.py
```

The launch file runs `dvrk_robot dvrk_system` with the installed
`system-MTML-MTMR-OpenXR-patient-cart-ROS.json`. That system file loads the
`sawOpenXR` plugin and its `config/sawOpenXR-isaac-rtsp.json` configuration.
It starts `dvrk_isaac_sim` with `ECM_PSM1_PSM2_PSM3_stereo.yaml` first and
waits 15 seconds before launching `dvrk_system`. Override the scene or delay,
for example, with `isaac_scene:=ECM_PSM1_PSM2_PSM3_mono.yaml` or
`dvrk_system_delay:=30.0`.

`mtsOpenXR` owns the OpenXR, Vulkan, and GStreamer runtime. Each OpenXR frame
updates the two controller grip poses and input actions. The cisst task loop
publishes those samples through `MTML` and `MTMR`, maintains the camera and
video-plane clutch inputs, and converts each thumbstick-up gesture into the
fail-closed local clutch for its PSM teleop.
