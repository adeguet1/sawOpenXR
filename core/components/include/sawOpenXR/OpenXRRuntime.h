#ifndef _sawOpenXROpenXRRuntime_h
#define _sawOpenXROpenXRRuntime_h

#include <array>
#include <atomic>
#include <functional>
#include <string>
#include <utility>

#include <sawOpenXR/sawOpenXRExport.h>

namespace sawOpenXR {

struct ControllerState {
  bool tracked = false;
  bool side_trigger_pressed = false;
  bool front_trigger_active = false;
  double front_trigger = 0.0;
  bool a_pressed = false;
  bool x_pressed = false;
  std::array<double, 3> position{};
  std::array<double, 4> orientation{{0.0, 0.0, 0.0, 1.0}};
  double timestamp = 0.0;
};

class CISST_EXPORT OpenXRRuntime {
public:
  using ControllerCallback =
      std::function<void(const std::array<ControllerState, 2> &)>;
  using ErrorCallback = std::function<void(const std::string &)>;

  OpenXRRuntime(const std::string &video_pipeline,
                ControllerCallback controller_callback,
                ErrorCallback error_callback);

  void Run(void);

  void RequestStop(void);

private:
  std::string m_video_pipeline;
  ControllerCallback m_controller_callback;
  ErrorCallback m_error_callback;
  std::atomic_bool m_stop_requested{false};
};

} // namespace sawOpenXR

#endif
