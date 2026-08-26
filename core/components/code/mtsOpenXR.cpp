#include <sawOpenXR/mtsOpenXR.h>

#include <cisstCommon/cmnLogger.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <json/json.h>

#include <algorithm>
#include <fstream>
#include <stdexcept>

namespace {
constexpr size_t ConsoleOperatorPresent = 0;
constexpr size_t ConsoleClutch = 1;
constexpr size_t ConsoleCamera = 2;

size_t ConsoleButtonIndex(const std::string &name) {
  if (name == "operator_present") {
    return ConsoleOperatorPresent;
  }

  if (name == "clutch") {
    return ConsoleClutch;
  }

  if (name == "camera") {
    return ConsoleCamera;
  }

  throw std::runtime_error("unknown console button: " + name);
}

std::string ExpandDvrkSocket(const std::string &input) {
  if (input.rfind("@dvrk:", 0) != 0) {
    return input;
  }

  const size_t reference_end = input.find_first_of(" \t!");
  const std::string reference = reference_end == std::string::npos
                                    ? input.substr(1)
                                    : input.substr(1, reference_end - 1);
  const std::string remainder =
      reference_end == std::string::npos ? "" : input.substr(reference_end);

  return "unixfdsrc socket-path=" + reference +
         " socket-type=abstract do-timestamp=true" + remainder;
}
} // namespace

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsOpenXR, mtsTaskContinuous,
                                      mtsTaskContinuousConstructorArg);

mtsOpenXR::mtsOpenXR(const std::string &component_name)
    : mtsTaskContinuous(component_name, 256) {
  Init();
}

mtsOpenXR::mtsOpenXR(const mtsTaskContinuousConstructorArg &argument)
    : mtsTaskContinuous(argument) {
  Init();
}

mtsOpenXR::~mtsOpenXR() { Cleanup(); }

void mtsOpenXR::Init(void) {
  for (auto &hand : m_hands) {
    // A valid neutral transform satisfies the MTM_GENERIC contract before
    // the OpenXR action loop supplies live controller poses.
    hand.measured_cs.Position().Assign(vctFrm3::Identity());
    hand.measured_cs.PositionIsValid() = true;
    hand.measured_cs.Velocity().Assign(vct6(0.0));
    hand.measured_cs.VelocityIsValid() = false;
    hand.measured_cs.ForceIsValid() = false;
    hand.measured_cs.SetValid(true);
    hand.gripper_measured_js.SetSize(1);
    hand.gripper_measured_js.Name()[0] = "gripper";
    hand.gripper_measured_js.Position()[0] = 60.0 * cmnPI_180;
    hand.gripper_measured_js.SetValid(false);
    // Quest controllers are non-motorized input devices.  Their dVRK MTM
    // facade is always ready; operator presence is a separate console
    // safety input and must not be conflated with device homing.
    hand.operating_state.IsBusy() = false;
    hand.operating_state.IsHomed() = true;
    hand.operating_state.State() = prmOperatingState::ENABLED;
    hand.operating_state.Valid() = true;
  }
  ConfigureInterfaces();
}

void mtsOpenXR::ConfigureInterfaces(void) {
  const std::array<std::string, 2> hand_names{{"MTML", "MTMR"}};

  for (size_t hand_index = 0; hand_index < hand_names.size(); ++hand_index) {
    mtsInterfaceProvided *provided =
        AddInterfaceProvided(hand_names[hand_index]);

    if (!provided) {
      throw std::runtime_error("failed to create " + hand_names[hand_index] +
                               " interface");
    }

    HandData &hand = m_hands[hand_index];

    StateTable.AddData(hand.measured_cs,
                       hand_names[hand_index] + "/measured_cs");
    StateTable.AddData(hand.gripper_measured_js,
                       hand_names[hand_index] + "/gripper/measured_js");
    StateTable.AddData(hand.operating_state,
                       hand_names[hand_index] + "/operating_state");
    provided->AddMessageEvents();
    provided->AddCommandReadState(StateTable, hand.measured_cs, "measured_cs");
    provided->AddCommandFilteredReadState(
        StateTable, hand.measured_cs, prmStateCartesian::ToPositionCartesianGet,
        "measured_cp");
    provided->AddCommandReadState(StateTable, hand.gripper_measured_js,
                                  "gripper/measured_js");
    provided->AddCommandReadState(StateTable, hand.operating_state,
                                  "operating_state");
    provided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                  "period_statistics");
    m_operating_state_events[hand_index].Bind(
        provided->AddEventWrite("operating_state", prmOperatingState()));
    provided->AddCommandWrite(&mtsOpenXR::SetVirtualMTMState, this,
                              "state_command", std::string());
  }

  const std::array<std::string, 3> console_names{
      {"Console/operator_present", "Console/clutch", "Console/camera"}};

  for (size_t index = 0; index < console_names.size(); ++index) {
    mtsInterfaceProvided *provided = AddInterfaceProvided(console_names[index]);

    if (!provided) {
      throw std::runtime_error("failed to create " + console_names[index] +
                               " interface");
    }

    m_console_button_events[index].Bind(
        provided->AddEventWrite("Button", prmEventButton()));
  }

  mtsInterfaceProvided *test_interface = AddInterfaceProvided("Test");

  if (!test_interface) {
    throw std::runtime_error("failed to create Test interface");
  }

  test_interface->AddCommandWrite(&mtsOpenXR::SetTestSideTriggers, this,
                                  "side_triggers", std::string());
}

void mtsOpenXR::ConfigureVideoSource(const std::string &filename) {
  if (filename.empty()) {
    return;
  }

  std::ifstream stream(filename.c_str());

  if (!stream.is_open()) {
    throw std::runtime_error("unable to open configuration " + filename);
  }

  Json::Value root;
  Json::Reader reader;

  if (!reader.parse(stream, root)) {
    throw std::runtime_error("invalid JSON in " + filename);
  }

  const std::string input = root.get("gst_input", "").asString();

  if (input.empty()) {
    throw std::runtime_error("configuration requires a root gst_input string");
  }

  m_video_pipeline = ExpandDvrkSocket(input);
  m_video_description = "GStreamer input " + input;
}

void mtsOpenXR::Configure(const std::string &filename) {
  ConfigureVideoSource(filename);
  m_configured = true;
  CMN_LOG_CLASS_INIT_VERBOSE << "Configure: video source "
                             << m_video_description << std::endl;
}

void mtsOpenXR::Startup(void) {
  if (!m_configured) {
    CMN_LOG_CLASS_INIT_ERROR << "Startup: component has not been configured"
                             << std::endl;
    return;
  }
  // OpenXR/Vulkan/GStreamer ownership is created here in the next increment;
  // keeping it in this component guarantees a single XR session for video and
  // input.
  SetOperatorPresent(true, "virtual console startup");
  // Deadman semantics: teleoperation starts clutched and requires both
  // controller side triggers to remain pressed before motion is allowed.
  SetConsoleButton("clutch", true);

  StartOpenXRRuntime();
}

void mtsOpenXR::UpdateControllerSamples(const ControllerSample &left,
                                        const ControllerSample &right) {
  std::lock_guard<std::mutex> lock(m_samples_mutex);

  m_pending_samples[LEFT] = left;
  m_pending_samples[RIGHT] = right;
  m_pending_samples_available = true;
}

void mtsOpenXR::StartOpenXRRuntime(void) {
  m_runtime = std::make_unique<sawOpenXR::OpenXRRuntime>(
      m_video_pipeline,
      [this](const std::array<sawOpenXR::ControllerState, 2> &controllers) {
        HandleOpenXRControllers(controllers);
      },
      [this](const std::string &reason) { HandleOpenXRError(reason); });

  m_runtime_thread = std::thread([this]() { m_runtime->Run(); });
}

void mtsOpenXR::StopOpenXRRuntime(void) {
  if (m_runtime) {
    m_runtime->RequestStop();
  }

  if (m_runtime_thread.joinable()) {
    m_runtime_thread.join();
  }

  m_runtime.reset();
}

void mtsOpenXR::HandleOpenXRControllers(
    const std::array<sawOpenXR::ControllerState, 2> &controllers) {
  ControllerSample left;
  ControllerSample right;
  const std::array<ControllerSample *, 2> samples{{&left, &right}};

  for (size_t hand_index = 0; hand_index < samples.size(); ++hand_index) {
    const auto &controller = controllers[hand_index];
    auto &sample = *samples[hand_index];
    sample.tracked = controller.tracked;
    sample.side_trigger_pressed = controller.side_trigger_pressed;
    sample.front_trigger_active = controller.front_trigger_active;
    sample.front_trigger = controller.front_trigger;
    sample.a_pressed = controller.a_pressed;
    sample.x_pressed = controller.x_pressed;
    sample.position = controller.position;
    sample.orientation = controller.orientation;
    sample.timestamp = controller.timestamp;
  }

  UpdateControllerSamples(left, right);
}

void mtsOpenXR::HandleOpenXRError(const std::string &reason) {
  std::lock_guard<std::mutex> lock(m_samples_mutex);

  m_pending_runtime_error = reason;
}

void mtsOpenXR::ApplyControllerSamples(void) {
  std::array<ControllerSample, 2> samples;
  std::string runtime_error;

  {
    std::lock_guard<std::mutex> lock(m_samples_mutex);

    if (m_pending_samples_available) {
      samples = m_pending_samples;
      m_pending_samples_available = false;
    } else {
      samples = m_samples;
    }

    runtime_error.swap(m_pending_runtime_error);
  }

  if (!runtime_error.empty()) {
    ReportSessionFailure(runtime_error);

    return;
  }

  m_samples = samples;

  for (size_t hand_index = 0; hand_index < m_hands.size(); ++hand_index) {
    auto &hand = m_hands[hand_index];
    const auto &sample = m_samples[hand_index];

    if (sample.tracked) {
      // OpenXR uses X-right, Y-up, and -Z-forward. dVRK's surgeon-view
      // convention is X-left, Y-up, and Z-away. The proper basis change is a
      // 180-degree rotation about Y: C = diag(-1, 1, -1). Quaternion
      // conjugation by C maps (x, y, z, w) to (-x, y, -z, w).
      const vctQuatRot3 quaternion(
          -sample.orientation[0], sample.orientation[1], -sample.orientation[2],
          sample.orientation[3], VCT_NORMALIZE);

      hand.measured_cs.Position().Rotation().Assign(vctMatRot3(quaternion));
      hand.measured_cs.Position().Translation().Assign(
          -sample.position[0], sample.position[1], -sample.position[2]);
      hand.measured_cs.PositionIsValid() = true;
      hand.measured_cs.SetValid(true);
    } else {
      hand.measured_cs.PositionIsValid() = false;
      hand.measured_cs.SetValid(false);
    }

    hand.measured_cs.SetTimestamp(StateTable.GetTic());
    hand.measured_cs.VelocityIsValid() = false;
    hand.measured_cs.ForceIsValid() = false;

    // The Quest index trigger is 0.0 when released and 1.0 when fully
    // pressed.  Present it as a conventional MTM gripper angle: released is
    // open at 60 degrees and pulling the trigger closes toward zero.
    const double gripper_open_angle = 60.0 * cmnPI_180;
    hand.gripper_measured_js.Position()[0] =
        (1.0 - std::clamp(sample.front_trigger, 0.0, 1.0)) * gripper_open_angle;
    hand.gripper_measured_js.SetTimestamp(StateTable.GetTic());
    hand.gripper_measured_js.SetValid(sample.tracked &&
                                      sample.front_trigger_active);
  }
}

void mtsOpenXR::SetTestSideTriggers(const std::string &command) {
  ControllerSample left;
  ControllerSample right;
  left.tracked = true;
  right.tracked = true;
  left.side_trigger_pressed =
      command == "both_pressed" || command == "left_pressed";
  right.side_trigger_pressed =
      command == "both_pressed" || command == "right_pressed";

  if (command != "both_pressed" && command != "left_pressed" &&
      command != "right_pressed" && command != "both_released") {
    CMN_LOG_CLASS_RUN_WARNING
        << "side_triggers expects both_pressed, left_pressed, "
        << "right_pressed, or both_released; received " << command << std::endl;
    return;
  }

  UpdateControllerSamples(left, right);
  UpdateSafetyState();
  CMN_LOG_CLASS_RUN_VERBOSE
      << "test side triggers: " << command << ", clutch "
      << (m_console_button_states[ConsoleClutch] ? "pressed" : "released")
      << std::endl;
}

void mtsOpenXR::SetConsoleButton(const std::string &name, const bool pressed) {
  const size_t index = ConsoleButtonIndex(name);

  if (m_console_button_states[index] == pressed) {
    return;
  }

  m_console_button_states[index] = pressed;

  if (m_console_button_events[index].IsValid()) {
    m_console_button_events[index](prmEventButton(
        pressed ? prmEventButton::PRESSED : prmEventButton::RELEASED));
  }
}

void mtsOpenXR::ReportSessionFailure(const std::string &reason) {
  for (auto &sample : m_samples) {
    sample = ControllerSample();
  }

  SetOperatorPresent(false, reason);
  SetConsoleButton("clutch", true);
  SetConsoleButton("camera", false);
  for (auto &hand : m_hands) {
    hand.measured_cs.SetValid(false);
    hand.gripper_measured_js.SetValid(false);
    hand.operating_state.SetState(prmOperatingState::FAULT);
  }
}

void mtsOpenXR::SetOperatorPresent(const bool present,
                                   const std::string &reason) {
  const OperatorState requested = present ? ENABLED : DISABLED;

  if (requested == m_operator_state) {
    return;
  }

  m_operator_state = requested;

  SetConsoleButton("operator_present", present);
  CMN_LOG_CLASS_RUN_WARNING << "operator present "
                            << (present ? "enabled" : "disabled") << ": "
                            << reason << std::endl;
}

void mtsOpenXR::SetHandOperatingState(const bool enabled) {
  for (auto &hand : m_hands) {
    hand.operating_state.SetState(enabled ? prmOperatingState::ENABLED
                                          : prmOperatingState::DISABLED);
    hand.operating_state.SetIsHomed(enabled);
    hand.operating_state.SetValid(true);
  }
}

void mtsOpenXR::EmitHandOperatingStateEvents(void) {
  for (size_t hand_index = 0; hand_index < m_hands.size(); ++hand_index) {
    if (m_operating_state_events[hand_index].IsValid()) {
      m_operating_state_events[hand_index](m_hands[hand_index].operating_state);
    }
  }
}

void mtsOpenXR::SetVirtualMTMState(const std::string &command) {
  if (command == "enable" || command == "home") {
    SetHandOperatingState(true);
  } else if (command == "disable") {
    SetHandOperatingState(false);
  } else {
    CMN_LOG_CLASS_RUN_WARNING
        << "ignoring unsupported virtual MTM state command: " << command
        << std::endl;
  }

  EmitHandOperatingStateEvents();
}

void mtsOpenXR::UpdateSafetyState(void) {
  const bool both_side_triggers = m_samples[LEFT].tracked &&
                                  m_samples[RIGHT].tracked &&
                                  m_samples[LEFT].side_trigger_pressed &&
                                  m_samples[RIGHT].side_trigger_pressed;
  // A clutch event is PRESSED while the PSMs must be held still.
  SetConsoleButton("clutch", !both_side_triggers);
  SetConsoleButton("camera",
                   m_samples[LEFT].tracked && m_samples[LEFT].x_pressed);
}

void mtsOpenXR::Run(void) {
  ProcessQueuedCommands();

  ProcessQueuedEvents();

  ApplyControllerSamples();

  UpdateSafetyState();
}

void mtsOpenXR::Cleanup(void) {
  StopOpenXRRuntime();

  SetOperatorPresent(false, "cleanup");
}
