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

The OpenXR console is configured for continuous operator presence. Its control
mapping is:

- Hold right A and move the right controller to position the video window.
- Left X emulates the camera pedal.
- Hold both controller side triggers to release the clutch. Releasing either
  trigger asserts clutch, so the PSMs cannot move.

Build the ROS 2 package with colcon from the dVRK workspace:

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
publishes those samples through `MTML` and `MTMR`, and converts the two side
triggers into the fail-closed console clutch event.
