#ifndef _sawOpenXROpenXRRuntime_h
#define _sawOpenXROpenXRRuntime_h

#include <array>
#include <atomic>
#include <functional>
#include <string>
#include <utility>

#include <sawOpenXR/sawOpenXRExport.h>

namespace sawOpenXR {

enum class VideoType { Mono, SideBySide };

struct ControllerState {
  bool session_focused = false;
  bool tracked = false;
  double thumbstick_x = 0.0;
  double thumbstick_y = 0.0;
  bool thumbstick_click = false;
  bool front_trigger_active = false;
  double front_trigger = 0.0;
  bool window_move_pressed = false;
  std::array<double, 3> position{};
  std::array<double, 4> orientation{{0.0, 0.0, 0.0, 1.0}};
  double timestamp = 0.0;
};

class CISST_EXPORT OpenXRRuntime {
public:
  using ControllerCallback =
      std::function<void(const std::array<ControllerState, 2> &)>;
  // Only lifecycle messages for the externally configured GStreamer source
  // are dispatched to the dVRK component.  OpenXR runtime diagnostics remain
  // normal process output.
  using GStreamerCallback = std::function<void(const std::string &)>;
  using ErrorCallback = std::function<void(const std::string &)>;

  OpenXRRuntime(const std::string &video_pipeline, VideoType video_type,
                ControllerCallback controller_callback,
                ErrorCallback error_callback,
                GStreamerCallback gstreamer_status_callback,
                GStreamerCallback gstreamer_warning_callback);

  void Run(void);

  void RequestStop(void);

private:
  std::string m_video_pipeline;
  VideoType m_video_type;
  ControllerCallback m_controller_callback;
  ErrorCallback m_error_callback;
  GStreamerCallback m_gstreamer_status_callback;
  GStreamerCallback m_gstreamer_warning_callback;
  std::atomic_bool m_stop_requested{false};
};

} // namespace sawOpenXR

#endif
