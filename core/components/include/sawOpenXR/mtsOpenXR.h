#ifndef _mtsOpenXR_h
#define _mtsOpenXR_h

#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmStateCartesian.h>
#include <cisstParameterTypes/prmStateJoint.h>

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <sawOpenXR/OpenXRRuntime.h>
#include <sawOpenXR/sawOpenXRExport.h>

// The component owns the single OpenXR session.  Its public controller sample
// is deliberately independent from OpenXR so the gesture/safety state machine
// can be tested without a headset.
class CISST_EXPORT mtsOpenXR : public mtsTaskContinuous {
  CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
  struct ControllerSample {
    bool session_focused = false;
    bool tracked = false;
    double thumbstick_y = 0.0;
    bool front_trigger_active = false;
    double front_trigger = 0.0;
    bool window_move_pressed = false;
    std::array<double, 3> position{};
    std::array<double, 4> orientation{{0.0, 0.0, 0.0, 1.0}};
    double timestamp = 0.0;
  };

  mtsOpenXR(const std::string &component_name);
  mtsOpenXR(const mtsTaskContinuousConstructorArg &argument);
  ~mtsOpenXR() override;

  void Configure(const std::string &filename = "") override;
  void Startup(void) override;
  void Run(void) override;
  void Cleanup(void) override;

  // Called by the OpenXR action layer. Right A moves the video plane and left
  // X resets it in front of the user. Thumbsticks control the local clutches
  // and camera, and index triggers provide MTM gripper input.
  void UpdateControllerSamples(const ControllerSample &left,
                               const ControllerSample &right);
  void SetTestThumbsticks(const std::string &command);
  // Called by the OpenXR action layer.  Console events remain explicit so
  // that a tracking/session fault can release every active control.
  void SetConsoleButton(const std::string &name, const bool pressed);
  void ReportSessionFailure(const std::string &reason);

protected:
  enum HandIndex { LEFT = 0, RIGHT = 1 };
  enum OperatorState { DISABLED, ENABLED };

  struct HandData {
    prmStateCartesian measured_cs;
    prmStateJoint gripper_measured_js;
    prmOperatingState operating_state;
  };

  void Init(void);
  void ConfigureInterfaces(void);
  void ConfigureVideoSource(const std::string &filename);
  void StartOpenXRRuntime(void);
  void StopOpenXRRuntime(void);
  void HandleOpenXRControllers(
      const std::array<sawOpenXR::ControllerState, 2> &controllers);
  void HandleOpenXRError(const std::string &reason);
  void ApplyControllerSamples(void);
  void UpdateSafetyState(void);
  void SetOperatorPresent(const bool present, const std::string &reason);
  void SetLocalClutch(const HandIndex hand, const bool clutched);
  void SetHandOperatingState(const bool enabled);
  void EmitHandOperatingStateEvents(void);
  void SetVirtualMTMState(const std::string &command);

  std::array<ControllerSample, 2> m_samples;
  std::array<ControllerSample, 2> m_pending_samples;
  std::mutex m_samples_mutex;
  bool m_pending_samples_available = false;
  std::string m_pending_runtime_error;
  std::unique_ptr<sawOpenXR::OpenXRRuntime> m_runtime;
  std::thread m_runtime_thread;
  std::array<HandData, 2> m_hands;
  std::array<mtsFunctionWrite, 2> m_operating_state_events;
  std::array<mtsFunctionWrite, 3> m_console_button_events;
  std::array<bool, 3> m_console_button_states{{false, false, false}};
  std::array<mtsFunctionWrite, 2> m_local_clutch_events;
  std::array<bool, 2> m_local_clutched{{false, false}};
  OperatorState m_operator_state = DISABLED;
  std::string m_video_pipeline;
  std::string m_video_description;
  bool m_configured = false;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsOpenXR);

#endif
