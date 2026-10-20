#include "LinuxHardwareSerial.hpp"
#include "SerialBridge.hpp"
#include "debug_msgs/msg/feedback.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_packets.hpp"
#include <natto_robot_hardware/pid_gain_path.hpp>
#include <std_msgs/msg/int64.hpp>
#include <memory>
#include "yaml-cpp/yaml.h"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::literals::chrono_literals;

class Stm32PosControlTest : public rclcpp::Node
{
public:
  Stm32PosControlTest()
  : Node("stm32_pos_control_test")
  {
    serial_path_ = declare_parameter<std::string>(
      "serial_path", "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF565653756687235135-if02");
    target_motor_index_ = declare_parameter<int>("target_motor_index", 0);
    pid_gain_path_ = declare_parameter<std::string>("pid_gain_path", "");
    max_motor_rps_override_ = declare_parameter<double>("max_motor_rps", 0.03);
    communication_period_seconds_ =
      declare_parameter<double>("communication_period_seconds", 0.01);
    csv_path_ = declare_parameter<std::string>("csv_path", "");
    direction_ = declare_parameter<bool>("direction", true);
    stop_mode_ = declare_parameter<bool>("stop_mode", false);
    reset_encoder_origin_ = declare_parameter<bool>("reset_encoder_origin", false);
    encoder_counts_per_revolution_ = declare_parameter<std::vector<double>>(
      "encoder_counts_per_revolution",
      {334506.6666666667, 334506.6666666667, 334506.6666666667, 334506.6666666667});
    encoder_pulses_per_revolution_ = declare_parameter<std::vector<int64_t>>(
      "encoder_pulses_per_revolution", {2048, 2048, 2048, 2048});

    encoder_polarities_ = declare_parameter<std::vector<int64_t>>(
      "encoder_polarities", std::vector<int64_t>(motor_count_, direction_ ? -1 : 1));
    allow_missing_pid_gains_ = declare_parameter<bool>("allow_missing_pid_gains", false);
    reset_encoder_origin_with_switch_ =
      declare_parameter<bool>("reset_encoder_origin_with_switch", false);
    encoder_origin_switch_state_ = declare_parameter<bool>("encoder_origin_switch_state", true);
    feedback_timeout_seconds_ = declare_parameter<double>("feedback_timeout_seconds", 1.0);
    if (!std::isfinite(feedback_timeout_seconds_) || feedback_timeout_seconds_ <= 0.0) {
      throw std::invalid_argument("feedback_timeout_seconds must be finite and positive");
    }
    if (encoder_polarities_.size() != motor_count_ ||
      std::any_of(encoder_polarities_.begin(), encoder_polarities_.end(),
      [](int64_t value) {return value != -1 && value != 1;}))
    {
      throw std::invalid_argument("encoder_polarities must contain four values of -1 or 1");
    }

    if (target_motor_index_ < 0 || target_motor_index_ >= static_cast<int>(motor_count_)) {
      throw std::invalid_argument("target_motor_index must be in the range 0..3");
    }
    if (
      !std::isfinite(max_motor_rps_override_) || max_motor_rps_override_ < 0.0 ||
      max_motor_rps_override_ > std::numeric_limits<float>::max())
    {
      throw std::invalid_argument(
              "max_motor_rps must be zero (use YAML) or a positive finite float");
    }
    if (!std::isfinite(communication_period_seconds_) || communication_period_seconds_ <= 0.0) {
      throw std::invalid_argument(
              "communication_period_seconds must be finite and greater than zero");
    }
    if (encoder_counts_per_revolution_.size() != motor_count_ ||
      encoder_pulses_per_revolution_.size() != motor_count_)
    {
      throw std::invalid_argument("encoder parameter arrays must each contain four values");
    }
    for (size_t i = 0; i < motor_count_; ++i) {
      if (!std::isfinite(encoder_counts_per_revolution_[i]) ||
        encoder_counts_per_revolution_[i] <= 0.0)
      {
        throw std::invalid_argument("encoder_counts_per_revolution values must be positive");
      }
      if (encoder_pulses_per_revolution_[i] <= 0 ||
        encoder_pulses_per_revolution_[i] > std::numeric_limits<uint16_t>::max())
      {
        throw std::invalid_argument(
                "encoder_pulses_per_revolution values must fit in a positive uint16");
      }
    }
    for (size_t i = 0; i < motor_count_; ++i) {
      feedback_publishers_.push_back(
        create_publisher<debug_msgs::msg::Feedback>("feedback" + std::to_string(i), 10));
    }

    for (size_t i = 0; i < motor_count_; ++i) {
      raw_count_publishers_.push_back(
        create_publisher<std_msgs::msg::Int64>("encoder_raw_count" + std::to_string(i), 10));
    }

    resolve_pid_gain_path();
    load_generated_pid_gains();
    initialize_packets();
    frames_.configuration.set_timeouts(
      std::chrono::milliseconds(declare_parameter<int>("initialization_retry_ms", 100)),
      std::chrono::milliseconds(declare_parameter<int>("initialization_timeout_ms", 2000)));
    open_csv_log();
    open_serial();
    register_serial_frames();
    send_initialization();

    terminal_timer_ = create_wall_timer(20ms, std::bind(&Stm32PosControlTest::poll_terminal, this));
    communication_timer_ = create_wall_timer(
      std::chrono::duration<double>(communication_period_seconds_),
      std::bind(&Stm32PosControlTest::communicate, this));

    disable_control();

    RCLCPP_INFO(
      get_logger(),
      "Ready: serial=%s, default motor=%d, unified gain=%s, velocity max=%.4f rps, position=%s",
      serial_path_.c_str(), target_motor_index_,
      pid_gain_path_.c_str(),
      velocity_max_rps_[static_cast<size_t>(target_motor_index_)],
      position_gain_available_[static_cast<size_t>(target_motor_index_)] ? "available" : "unavailable");
    RCLCPP_INFO(get_logger(), "Recording command and feedback to %s", csv_path_.c_str());
    print_help();
    print_prompt();
  }

  ~Stm32PosControlTest() override
  {
    disable_control();

    if (serial_ != nullptr) {
      unregister_serial_frames();
    }

    serial_.reset();
    dev_.reset();

    if (csv_log_.is_open()) {
      csv_log_.flush();
      csv_log_.close();
    }
  }

private:
  static constexpr size_t motor_count_ = stm32_protocol::motor_count;

  struct PidGain
  {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
  };

  enum class ControlMode : uint8_t
  {
    Off = 0,
    Velocity = 1,
    Position = 2,
  };

  void resolve_pid_gain_path()
  {
    pid_gain_path_ = natto_robot_hardware::resolve_pid_gain_path(pid_gain_path_);
  }

  static double read_finite_gain(const YAML::Node & node, const std::string & key)
  {
    if (!node[key]) {throw std::runtime_error("missing PID field: " + key);}
    const double value = node[key].as<double>();
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
      throw std::runtime_error("PID field must be a finite float: " + key);
    }
    return value;
  }

  static void validate_units(
    const YAML::Node & controller, const std::string & wheel_name,
    const char * label, const char * expected_input, const char * expected_output)
  {
    if (!controller["input_unit"] || controller["input_unit"].as<std::string>() != expected_input ||
      !controller["output_unit"] || controller["output_unit"].as<std::string>() != expected_output)
    {
      throw std::runtime_error(wheel_name + "." + label + " has incompatible controller units");
    }
  }

  void load_generated_pid_gains()
  {
    for (size_t i = 0; i < motor_count_; ++i) {
      encoder_ppr_[i] = static_cast<uint16_t>(encoder_pulses_per_revolution_[i]);
      encoder_gear_ratio_[i] = encoder_counts_per_revolution_[i] /
        (4.0 * static_cast<double>(encoder_ppr_[i]));
      if (!std::isfinite(encoder_gear_ratio_[i]) || encoder_gear_ratio_[i] <= 0.0 ||
        encoder_gear_ratio_[i] > std::numeric_limits<float>::max())
      {
        throw std::runtime_error("invalid encoder gear ratio");
      }
      velocity_max_rps_[i] = max_motor_rps_override_ > 0.0 ? max_motor_rps_override_ : 0.03;
      position_max_rps_[i] = velocity_max_rps_[i];
    }
    if (!std::filesystem::exists(pid_gain_path_) && allow_missing_pid_gains_) {
      RCLCPP_WARN(get_logger(),
        "No PID file at %s: feedback/stop only. Tune this board and restart to enable commands.",
        pid_gain_path_.c_str());
      return;
    }
    const YAML::Node config = YAML::LoadFile(pid_gain_path_);
    if (!config.IsMap()) {throw std::runtime_error("PID gain YAML must be a mapping");}
    if (config["schema_version"] && config["schema_version"].as<int>() != 2) {
      throw std::runtime_error("unsupported PID schema_version (expected 2)");
    }

    for (size_t i = 0; i < motor_count_; ++i) {
      const std::string name = "wheel" + std::to_string(i);
      const YAML::Node wheel = config[name];
      if (!wheel) {continue;}
      if (!wheel.IsMap()) {throw std::runtime_error(name + " must be a mapping");}
      const YAML::Node velocity = wheel["velocity"] ? wheel["velocity"] : wheel;
      const YAML::Node position = wheel["position"];
      if (!velocity["kp"] || !velocity["ki"]) {continue;}
      if (wheel["velocity"]) {
        validate_units(velocity, name, "velocity", "rps", "duty");
        if (!velocity["controller"] || velocity["controller"].as<std::string>() != "PI") {
          throw std::runtime_error(name + ".velocity.controller must be PI");
        }
      }
      const double kp = read_finite_gain(velocity, "kp");
      const double ki = read_finite_gain(velocity, "ki");
      const double kd = velocity["kd"] ? read_finite_gain(velocity, "kd") : 0.0;
      if (kp <= 0.0 || ki < 0.0) {
        throw std::runtime_error(name + " needs positive velocity Kp and non-negative Ki");
      }
      if (kd != 0.0) {
        RCLCPP_WARN(get_logger(), "%s saved velocity kd ignored; STM velocity PI uses kd=0", name.c_str());
      }
      velocity_gains_[i] = {kp, ki, 0.0};
      velocity_gain_available_[i] = true;
      const double saved_limit = velocity["max_rps"] ?
        read_finite_gain(velocity, "max_rps") : velocity_max_rps_[i];
      if (saved_limit <= 0.0) {throw std::runtime_error(name + " has invalid velocity max_rps");}
      velocity_max_rps_[i] = max_motor_rps_override_ > 0.0 ?
        std::min(max_motor_rps_override_, saved_limit) : saved_limit;

      // Original unified schema stores encoder settings at wheel.encoder.
      const YAML::Node encoder = wheel["encoder"];
      if (encoder) {
        if (encoder["pulses_per_revolution"]) {
          const auto ppr = encoder["pulses_per_revolution"].as<int64_t>();
          if (ppr <= 0 || ppr > 65535) {throw std::runtime_error(name + " has invalid PPR");}
          encoder_ppr_[i] = static_cast<uint16_t>(ppr);
        }
        const double cpr = encoder["counts_per_revolution"] ?
          read_finite_gain(encoder, "counts_per_revolution") : encoder_counts_per_revolution_[i];
        encoder_gear_ratio_[i] = encoder["gear_ratio"] ?
          read_finite_gain(encoder, "gear_ratio") : cpr / (4.0 * encoder_ppr_[i]);
        if (encoder["counts_per_revolution"] &&
          (cpr <= 0.0 || std::abs(4.0 * encoder_ppr_[i] * encoder_gear_ratio_[i] - cpr) / cpr > 1e-4))
        {
          throw std::runtime_error(name + " encoder PPR, gear ratio and CPR disagree");
        }
      }
      // Local position_checker records the calibration used during tuning.
      if (position) {
        const bool has_ppr = static_cast<bool>(position["encoder_ppr"]);
        const bool has_gear = static_cast<bool>(position["gear_rate"]);
        const bool has_polarity = static_cast<bool>(position["encoder_polarity"]);
        if ((has_ppr || has_gear || has_polarity) && !(has_ppr && has_gear && has_polarity)) {
          throw std::runtime_error(name + " has incomplete position encoder calibration");
        }
        if (has_ppr) {
          const auto ppr = position["encoder_ppr"].as<int64_t>();
          const auto polarity = position["encoder_polarity"].as<int64_t>();
          const auto gear = read_finite_gain(position, "gear_rate");
          if (ppr <= 0 || ppr > 65535 || gear <= 0.0 || (polarity != -1 && polarity != 1)) {
            throw std::runtime_error(name + " has invalid position encoder calibration");
          }
          if (encoder && (ppr != encoder_ppr_[i] ||
            std::abs(gear - encoder_gear_ratio_[i]) > 1e-4))
          {
            throw std::runtime_error(name + " encoder calibration conflicts with position tuning");
          }
          encoder_ppr_[i] = static_cast<uint16_t>(ppr);
          encoder_gear_ratio_[i] = gear;
          encoder_polarities_[i] = polarity;
        }
        validate_units(position, name, "position", "encoder_count", "rps");
        if (!position["controller"] || position["controller"].as<std::string>() != "P") {
          throw std::runtime_error(name + ".position.controller must be P");
        }
        const PidGain gain{read_finite_gain(position, "kp"),
          read_finite_gain(position, "ki"), read_finite_gain(position, "kd")};
        if (gain.kp <= 0.0 || gain.ki != 0.0 || gain.kd != 0.0) {
          throw std::runtime_error(name + " needs positive position Kp with Ki=Kd=0");
        }
        if (velocity["response_time_seconds"] && position["response_time_seconds"] &&
          read_finite_gain(position, "response_time_seconds") <=
          read_finite_gain(velocity, "response_time_seconds"))
        {
          throw std::runtime_error(name + " position response must be slower than velocity response");
        }
        position_gains_[i] = gain;
        position_gain_available_[i] = true;
        const double limit = position["max_rps"] ?
          read_finite_gain(position, "max_rps") : velocity_max_rps_[i];
        if (limit <= 0.0) {throw std::runtime_error(name + " has invalid position max_rps");}
        // A single STM initialization limit applies to both velocity and position modes.
        velocity_max_rps_[i] = std::min(velocity_max_rps_[i], limit);
      }
      position_max_rps_[i] = velocity_max_rps_[i];
      RCLCPP_INFO(get_logger(), "%s velocity PI=(%.9g, %.9g), limit=%.6g rps, position=%s",
        name.c_str(), kp, ki, velocity_max_rps_[i], position ? "available" : "unavailable");
    }
    for (size_t i = 0; i < motor_count_; ++i) {
      if (!std::isfinite(encoder_gear_ratio_[i]) || encoder_gear_ratio_[i] <= 0.0 ||
        encoder_gear_ratio_[i] > std::numeric_limits<float>::max())
      {
        throw std::runtime_error("invalid encoder gear ratio");
      }
    }
    if (!velocity_gain_available_[static_cast<size_t>(target_motor_index_)]) {
      throw std::runtime_error("default motor has no usable velocity gains; tune this board first");
    }
  }

  static uint8_t frame_id(uint8_t base, size_t motor)
  {
    return static_cast<uint8_t>(base + motor);
  }

  void initialize_packets()
  {
    for (size_t i = 0; i < motor_count_; ++i) {
      auto & init = initialization_msg_[i].data;
      std::memset(&init, 0, sizeof(init));
      init.reset_encoder_origin = reset_encoder_origin_;
      init.vel_kp = static_cast<float>(velocity_gains_[i].kp);
      init.vel_ki = static_cast<float>(velocity_gains_[i].ki);
      init.vel_kd = static_cast<float>(velocity_gains_[i].kd);
      init.pos_kp = static_cast<float>(position_gains_[i].kp);
      init.pos_ki = static_cast<float>(position_gains_[i].ki);
      init.pos_kd = static_cast<float>(position_gains_[i].kd);
      init.set_direction = encoder_polarities_[i] < 0;
      init.reset_encoder_origin_with_switch = reset_encoder_origin_with_switch_;
      init.encoder_origin_switch_state = encoder_origin_switch_state_;
      init.stop_mode = stop_mode_;
      init.pid_state = 0;
      init.max_motor_rps = static_cast<float>(position_gain_available_[i] ?
        position_max_rps_[i] : velocity_max_rps_[i]);
      init.encoder_ppr = encoder_ppr_[i];
      init.gear_rate = static_cast<float>(encoder_gear_ratio_[i]);

      velocity_msg_[i].data.target_motor_rps = 0.0f;
      position_msg_[i].data.target_pos = 0;
      duty_msg_[i].data.duty = 0.0f;
      std::memset(&feedback_msg_[i].data, 0, sizeof(feedback_msg_[i].data));
    }
  }

  void add_frame_or_throw(uint8_t id, sb::MessageInterface * message)
  {
    const int result = serial_->add_frame(id, message);
    if (result != 0) {
      throw std::runtime_error(
              "failed to register serial frame " + std::to_string(id) +
              " (result=" + std::to_string(result) + ")");
    }
  }

  void register_serial_frames()
  {
    frames_.register_all(*serial_);
  }

  void unregister_serial_frames()
  {
    for (uint8_t id = 0; id <= 24; ++id) serial_->rm_frame(id);
  }

  void send_initialization()
  {
    frames_.initialize(*serial_);
  }

  void open_serial()
  {
    dev_ = std::make_unique<LinuxHardwareSerial>(serial_path_.c_str(), B230400);
    serial_ = std::make_unique<SerialBridge>(dev_.get());
  }

  static std::string make_log_timestamp()
  {
    const std::time_t time = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now());
    std::tm local_time{};
    localtime_r(&time, &local_time);

    std::ostringstream timestamp;
    timestamp << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return timestamp.str();
  }

  void open_csv_log()
  {
    if (csv_path_.empty()) {
      std::filesystem::path log_directory;
      const char * ros_home = std::getenv("ROS_HOME");
      if (ros_home != nullptr && ros_home[0] != '\0') {
        log_directory = std::filesystem::path(ros_home) / "stm32_pos_control_test";
      } else {
        const char * user_home = std::getenv("HOME");
        if (user_home == nullptr || user_home[0] == '\0') {
          throw std::runtime_error(
                  "csv_path is empty and neither ROS_HOME nor HOME is available");
        }
        log_directory = std::filesystem::path(user_home) / ".ros" /
          "stm32_pos_control_test";
      }

      csv_path_ = (
        log_directory /
        ("motor_control_" + make_log_timestamp() + "_pid" +
        std::to_string(static_cast<long>(::getpid())) + ".csv")).string();
    }

    const std::filesystem::path path = std::filesystem::absolute(csv_path_).lexically_normal();
    if (path == std::filesystem::path(pid_gain_path_) ||
      (std::filesystem::exists(path) && std::filesystem::exists(pid_gain_path_) &&
      std::filesystem::equivalent(path, pid_gain_path_)))
    {
      throw std::invalid_argument("csv_path must differ from pid_gain_path");
    }
    if (path.has_parent_path()) {
      std::error_code error;
      std::filesystem::create_directories(path.parent_path(), error);
      if (error) {
        throw std::runtime_error(
                "failed to create CSV directory '" + path.parent_path().string() +
                "': " + error.message());
      }
    }

    csv_log_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_log_.is_open()) {
      throw std::runtime_error("failed to open CSV log '" + csv_path_ + "'");
    }

    csv_log_ <<
      "feedback_sequence,elapsed_seconds,system_time_ns,motor_index,command_sequence,"
      "control_active,pid_state,target_pos,encoder_raw_count,position_error,"
      "target_wheel_rps,measured_wheel_rps,feedback_output,duty,direction,encoder_count,"
      "kp,ki,kd,stop_mode\n";
    csv_log_.flush();
    if (!csv_log_) {
      throw std::runtime_error("failed to write CSV header to '" + csv_path_ + "'");
    }
    log_start_time_ = std::chrono::steady_clock::now();
  }

  void communicate()
  {
    if (serial_ == nullptr) {
      return;
    }

    const auto received = frames_.drain(*serial_);
    frames_.service_initialization(*serial_);
    if (received.configuration_lost) {
      disable_control();
      last_feedback_.fill({});
      RCLCPP_WARN(get_logger(), "STM configuration changed/reset; all commands cleared");
    }
    const auto &updated = received.motor_updated;
    bool any_updated = false;
    for (size_t i = 0; i < motor_count_; ++i) {
      any_updated = any_updated || updated[i];
      if (updated[i]) {last_feedback_[i] = std::chrono::steady_clock::now();}
    }
    if (any_updated) {
      publish_feedback(updated);
      record_feedback(updated);
    }
    if (control_active_) {
      for (size_t i = 0; i < motor_count_; ++i) {
        if (control_modes_[i] != ControlMode::Off && !feedback_is_fresh(i)) {
          disable_control();
          RCLCPP_ERROR(get_logger(), "Motor %zu feedback timed out; all motors stopped", i);
          return;
        }
      }
      write_active_commands();
    }
  }

  void publish_feedback(const std::array<bool, motor_count_> & updated)
  {
    for (size_t i = 0; i < feedback_publishers_.size(); ++i) {
      if (!updated[i]) {
        continue;
      }
      debug_msgs::msg::Feedback msg;
      const auto & feedback = feedback_msg_[i].data;

      msg.output = feedback.output;
      msg.current_rps = feedback.measured_wheel_rps;
      // This workspace's legacy Feedback message uses float32 for encoder_count.
      msg.encoder_count = static_cast<float>(feedback.encoder_raw_count);
      std_msgs::msg::Int64 raw;
      raw.data = feedback.encoder_raw_count;
      raw_count_publishers_[i]->publish(raw);
      msg.duty = feedback.duty;
      msg.dir = feedback.dir;
      if (control_modes_[i] == ControlMode::Position) {
        const double error = static_cast<double>(position_msg_[i].data.target_pos) -
          static_cast<double>(feedback.encoder_raw_count);
        msg.target_rps = static_cast<float>(std::clamp(
            position_gains_[i].kp * error, -position_max_rps_[i], position_max_rps_[i]));
      } else if (control_modes_[i] == ControlMode::Velocity) {
        msg.target_rps = velocity_msg_[i].data.target_motor_rps;
      } else {
        msg.target_rps = 0.0f;
      }

      feedback_publishers_[i]->publish(msg);
    }
  }

  void record_feedback(const std::array<bool, motor_count_> & updated)
  {
    if (!csv_log_.is_open() || csv_write_failed_) {
      return;
    }

    const double elapsed_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - log_start_time_).count();
    const auto system_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    csv_log_ << std::setprecision(17);
    for (size_t i = 0; i < motor_count_; ++i) {
      if (!updated[i]) {
        continue;
      }
      const auto & feedback = feedback_msg_[i].data;
      const ControlMode mode = control_modes_[i];
      const bool active = mode != ControlMode::Off;
      const int64_t target_pos = mode == ControlMode::Position ?
        position_msg_[i].data.target_pos : 0;
      const double position_error = mode == ControlMode::Position ?
        static_cast<double>(target_pos) - static_cast<double>(feedback.encoder_raw_count) : 0.0;
      double target_wheel_rps = 0.0;
      PidGain logged_gain{};
      if (mode == ControlMode::Position) {
        target_wheel_rps = std::clamp(
          position_gains_[i].kp * position_error,
          -position_max_rps_[i], position_max_rps_[i]);
        logged_gain = position_gains_[i];
      } else if (mode == ControlMode::Velocity) {
        target_wheel_rps = velocity_msg_[i].data.target_motor_rps;
        logged_gain = velocity_gains_[i];
      }

      csv_log_ << feedback_sequence_ << ',' << elapsed_seconds << ',' << system_time_ns << ',' <<
        i << ',' << command_sequence_ << ',' << (active ? 1 : 0) << ',' <<
        static_cast<unsigned>(mode) << ',' << target_pos << ',' <<
        feedback.encoder_raw_count << ',' << position_error << ',' << target_wheel_rps <<
        ',' << feedback.measured_wheel_rps << ',' << feedback.output << ',' << feedback.duty <<
        ',' << (feedback.dir ? 1 : 0) << ',' << feedback.encoder_count << ',' <<
        logged_gain.kp << ',' << logged_gain.ki << ',' <<
        logged_gain.kd << ',' << (stop_mode_ ? 1 : 0) << '\n';
    }
    ++feedback_sequence_;
    csv_log_.flush();

    if (!csv_log_) {
      csv_write_failed_ = true;
      RCLCPP_ERROR(
        get_logger(), "Failed to write CSV log '%s'; logging has stopped", csv_path_.c_str());
    }
  }

  void poll_terminal()
  {
    pollfd stdin_poll{};
    stdin_poll.fd = STDIN_FILENO;
    stdin_poll.events = POLLIN;

    const int poll_result = ::poll(&stdin_poll, 1, 0);
    if (poll_result < 0) {
      if (errno != EINTR) {
        RCLCPP_ERROR(get_logger(), "Failed to poll stdin: %s", std::strerror(errno));
        rclcpp::shutdown();
      }
      return;
    }
    if (poll_result == 0) {
      return;
    }

    if ((stdin_poll.revents & (POLLIN | POLLHUP)) == 0) {
      if ((stdin_poll.revents & (POLLERR | POLLNVAL)) != 0) {
        RCLCPP_ERROR(get_logger(), "stdin is not available");
        rclcpp::shutdown();
      }
      return;
    }

    std::array<char, 256> buffer{};
    const ssize_t bytes_read = ::read(STDIN_FILENO, buffer.data(), buffer.size());
    if (bytes_read < 0) {
      if (errno != EINTR && errno != EAGAIN) {
        RCLCPP_ERROR(get_logger(), "Failed to read stdin: %s", std::strerror(errno));
        rclcpp::shutdown();
      }
      return;
    }

    if (bytes_read == 0) {
      if (!input_buffer_.empty()) {
        process_line(input_buffer_);
        input_buffer_.clear();
      }
      RCLCPP_INFO(get_logger(), "stdin closed; shutting down");
      rclcpp::shutdown();
      return;
    }

    for (ssize_t i = 0; i < bytes_read; i++) {
      const char character = buffer[static_cast<size_t>(i)];
      if (character == '\n') {
        process_line(input_buffer_);
        input_buffer_.clear();
        if (!rclcpp::ok()) {
          return;
        }
        print_prompt();
      } else if (character != '\r') {
        input_buffer_ += character;
      }
    }
  }

  void process_line(const std::string & line)
  {
    std::istringstream input(line);
    std::vector<std::string> tokens;
    std::string token;
    while (input >> token) {
      tokens.push_back(token);
    }

    if (tokens.empty()) {
      return;
    }
    if (tokens[0] == "q" || tokens[0] == "quit" || tokens[0] == "exit") {
      rclcpp::shutdown();
      return;
    }
    if (tokens[0] == "h" || tokens[0] == "help") {
      print_help();
      return;
    }
    if (tokens[0] == "status") {
      for (size_t i = 0; i < motor_count_; ++i) {
        RCLCPP_INFO(get_logger(),
          "motor=%zu mode=%u raw_count=%ld measured=%.6g rps feedback=%s velocity=%s position=%s",
          i, static_cast<unsigned>(control_modes_[i]),
          static_cast<long>(feedback_msg_[i].data.encoder_raw_count),
          feedback_msg_[i].data.measured_wheel_rps,
          feedback_is_fresh(i) ? "fresh" : "missing/stale",
          velocity_gain_available_[i] ? "available" : "unavailable",
          position_gain_available_[i] ? "available" : "unavailable");
      }
      return;
    }
    if (tokens[0] == "stop") {
      ++command_sequence_;
      disable_control();
      RCLCPP_INFO(get_logger(), "Motor control disabled");
      return;
    }

    const bool position_command = tokens[0] == "pos" || tokens[0] == "position";
    const bool velocity_command = tokens[0] == "vel" || tokens[0] == "velocity";
    if (position_command) {
      if (tokens.size() == 2) {
        int64_t target_pos = 0;
        if (parse_integer(tokens[1], target_pos)) {
          set_motor_position(static_cast<size_t>(target_motor_index_), target_pos);
          return;
        }
      } else if (tokens.size() == 3) {
        int64_t motor_index = 0;
        int64_t target_pos = 0;
        if (parse_motor_index(tokens[1], motor_index) && parse_integer(tokens[2], target_pos)) {
          set_motor_position(static_cast<size_t>(motor_index), target_pos);
          return;
        }
      } else if (tokens.size() == motor_count_ + 2 && tokens[1] == "all") {
        std::array<int64_t, motor_count_> targets{};
        bool valid = true;
        for (size_t i = 0; i < motor_count_; ++i) {
          valid = valid && parse_integer(tokens[i + 2], targets[i]);
        }
        if (valid) {
          set_all_positions(targets);
          return;
        }
      }
    } else if (velocity_command) {
      if (tokens.size() == 2) {
        double target_rps = 0.0;
        if (parse_finite_double(tokens[1], target_rps)) {
          set_motor_velocity(static_cast<size_t>(target_motor_index_), target_rps);
          return;
        }
      } else if (tokens.size() == 3) {
        int64_t motor_index = 0;
        double target_rps = 0.0;
        if (parse_motor_index(tokens[1], motor_index) &&
          parse_finite_double(tokens[2], target_rps))
        {
          set_motor_velocity(static_cast<size_t>(motor_index), target_rps);
          return;
        }
      } else if (tokens.size() == motor_count_ + 2 && tokens[1] == "all") {
        std::array<double, motor_count_> targets{};
        bool valid = true;
        for (size_t i = 0; i < motor_count_; ++i) {
          valid = valid && parse_finite_double(tokens[i + 2], targets[i]);
        }
        if (valid) {
          set_all_velocities(targets);
          return;
        }
      }
    }

    // Backwards-compatible position command forms.
    if (tokens.size() == 1) {
      int64_t target_pos = 0;
      if (parse_integer(tokens[0], target_pos)) {
        set_motor_position(static_cast<size_t>(target_motor_index_), target_pos);
        return;
      }
    } else if (tokens.size() == 2) {
      int64_t motor_index = 0;
      int64_t target_pos = 0;
      if (parse_motor_index(tokens[0], motor_index) && parse_integer(tokens[1], target_pos)) {
        set_motor_position(static_cast<size_t>(motor_index), target_pos);
        return;
      }
    } else if (tokens.size() == motor_count_ + 1 && tokens[0] == "all") {
      std::array<int64_t, motor_count_> target_positions{};
      bool valid = true;
      for (size_t i = 0; i < motor_count_; i++) {
        valid = valid && parse_integer(tokens[i + 1], target_positions[i]);
      }
      if (valid) {
        set_all_positions(target_positions);
        return;
      }
    }

    RCLCPP_WARN(get_logger(), "Invalid input: '%s' (type 'help' for usage)", line.c_str());
  }

  static bool parse_integer(const std::string & text, int64_t & value)
  {
    std::istringstream stream(text);
    stream >> value;
    return stream && stream.eof();
  }

  static bool parse_finite_double(const std::string & text, double & value)
  {
    std::istringstream stream(text);
    stream >> value;
    return stream && stream.eof() && std::isfinite(value);
  }

  static bool parse_motor_index(const std::string & text, int64_t & value)
  {
    return parse_integer(text, value) && value >= 0 &&
           value < static_cast<int64_t>(motor_count_);
  }

  void set_motor_position(size_t motor_index, int64_t target_pos)
  {
    if (!feedback_is_fresh(motor_index)) {
      RCLCPP_WARN(get_logger(), "wheel%zu has no fresh feedback; command rejected", motor_index);
      return;
    }
    if (!position_gain_available_[motor_index] || !velocity_gain_available_[motor_index]) {
      RCLCPP_WARN(
        get_logger(),
        "wheel%zu needs usable position and velocity PID gains; command rejected", motor_index);
      return;
    }

    position_msg_[motor_index].data.target_pos = target_pos;
    control_modes_[motor_index] = ControlMode::Position;
    control_active_ = true;
    ++command_sequence_;

    if (write_position_command(motor_index)) {
      RCLCPP_INFO(
        get_logger(), "Sent target_pos: motor=%zu, pos=%ld", motor_index,
        static_cast<long>(target_pos));
    }
  }

  void set_all_positions(const std::array<int64_t, motor_count_> & target_positions)
  {
    for (size_t i = 0; i < motor_count_; ++i) {
      if (!position_gain_available_[i] || !velocity_gain_available_[i] || !feedback_is_fresh(i)) {
        RCLCPP_WARN(
          get_logger(), "wheel%zu needs usable position and velocity PID gains; all command rejected",
          i);
        return;
      }
    }

    for (size_t i = 0; i < motor_count_; i++) {
      position_msg_[i].data.target_pos = target_positions[i];
      control_modes_[i] = ControlMode::Position;
    }
    control_active_ = true;
    ++command_sequence_;

    if (write_active_commands()) {
      RCLCPP_INFO(
        get_logger(), "Sent target_pos: [%ld, %ld, %ld, %ld]", static_cast<long>(target_positions[0]),
        static_cast<long>(target_positions[1]), static_cast<long>(target_positions[2]),
        static_cast<long>(target_positions[3]));
    }
  }

  bool feedback_is_fresh(size_t motor) const
  {
    return frames_.configuration.ready() && last_feedback_[motor] != std::chrono::steady_clock::time_point{} &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() - last_feedback_[motor]).count() <
           feedback_timeout_seconds_;
  }

  bool velocity_target_is_safe(size_t motor_index, double target_rps) const
  {
    return feedback_is_fresh(motor_index) && velocity_gain_available_[motor_index] && std::isfinite(target_rps) &&
           std::fabs(target_rps) <= velocity_max_rps_[motor_index];
  }

  void set_motor_velocity(size_t motor_index, double target_rps)
  {
    if (!velocity_target_is_safe(motor_index, target_rps)) {
      RCLCPP_WARN(
        get_logger(),
        "wheel%zu velocity command %.6g rps rejected (available limit: +/-%.6g rps)",
        motor_index, target_rps, velocity_max_rps_[motor_index]);
      return;
    }

    velocity_msg_[motor_index].data.target_motor_rps = static_cast<float>(target_rps);
    control_modes_[motor_index] = ControlMode::Velocity;
    control_active_ = true;
    ++command_sequence_;

    if (write_velocity_command(motor_index)) {
      RCLCPP_INFO(
        get_logger(), "Sent target velocity: motor=%zu, rps=%.6g", motor_index, target_rps);
    }
  }

  void set_all_velocities(const std::array<double, motor_count_> & target_velocities)
  {
    for (size_t i = 0; i < motor_count_; ++i) {
      if (!velocity_target_is_safe(i, target_velocities[i])) {
        RCLCPP_WARN(
          get_logger(), "wheel%zu velocity %.6g rps is unavailable or exceeds +/-%.6g rps; all command rejected",
          i, target_velocities[i], velocity_max_rps_[i]);
        return;
      }
    }

    for (size_t i = 0; i < motor_count_; ++i) {
      velocity_msg_[i].data.target_motor_rps = static_cast<float>(target_velocities[i]);
      control_modes_[i] = ControlMode::Velocity;
    }
    control_active_ = true;
    ++command_sequence_;

    if (write_active_commands()) {
      RCLCPP_INFO(
        get_logger(), "Sent target velocities: [%.6g, %.6g, %.6g, %.6g] rps",
        target_velocities[0], target_velocities[1], target_velocities[2],
        target_velocities[3]);
    }
  }

  void disable_control()
  {
    if (serial_ == nullptr) {
      return;
    }

    bool success = true;
    for (size_t i = 0; i < motor_count_; ++i) {
      duty_msg_[i].data.duty = 0.0f;
      control_modes_[i] = ControlMode::Off;
      const int result = serial_->write(
        frame_id(stm32_protocol::command_duty_frame_id_base, i));
      if (result != duty_msg_[i].size() + 3) {
        success = false;
        RCLCPP_ERROR(
          get_logger(), "Failed to send stop frame for motor %zu (result=%d)", i, result);
      }
    }
    control_active_ = false;
    if (!success) {
      RCLCPP_ERROR(get_logger(), "One or more motor stop frames failed");
    }
  }

  bool write_position_command(size_t motor_index)
  {
    if (!frames_.configuration.ready()) { disable_control(); return false; }
    const int result = serial_->write(
      frame_id(stm32_protocol::command_position_frame_id_base, motor_index));
    if (result != position_msg_[motor_index].size() + 3) {
      disable_control();
      throw std::runtime_error("failed/short position serial write");
    }
    return true;
  }

  bool write_velocity_command(size_t motor_index)
  {
    if (!frames_.configuration.ready()) { disable_control(); return false; }
    const int result = serial_->write(
      frame_id(stm32_protocol::command_velocity_frame_id_base, motor_index));
    if (result != velocity_msg_[motor_index].size() + 3) {
      disable_control();
      throw std::runtime_error("failed/short velocity serial write");
    }
    return true;
  }

  bool write_active_commands()
  {
    bool success = true;
    for (size_t i = 0; i < motor_count_; ++i) {
      if (control_modes_[i] == ControlMode::Position) {
        success = write_position_command(i) && success;
      } else if (control_modes_[i] == ControlMode::Velocity) {
        success = write_velocity_command(i) && success;
      }
    }
    return success;
  }

  static void print_help()
  {
    std::cout << "Commands:\n"
              << "  pos <count>                         Position target for default motor\n"
              << "  pos <motor 0..3> <count>            Position target for one motor\n"
              << "  pos all <p0> <p1> <p2> <p3>         Position targets for all motors\n"
              << "  vel <rps>                           Velocity target for default motor\n"
              << "  vel <motor 0..3> <rps>              Velocity target for one motor\n"
              << "  vel all <v0> <v1> <v2> <v3>         Velocity targets for all motors\n"
              << "  <count> | <motor> <count> | all ... Legacy position forms\n"
              << "  stop                                Disable all motor control\n"
              << "  status                   Show per-motor feedback and gains\n"
              << "  help                     Show this help\n"
              << "  quit                     Stop and exit\n";
  }

  static void print_prompt()
  {
    std::cout << "motor_control> " << std::flush;
  }

  std::string serial_path_;
  std::string pid_gain_path_;
  std::string csv_path_;
  std::string input_buffer_;
  int target_motor_index_;
  double max_motor_rps_override_;
  std::array<double, motor_count_> velocity_max_rps_{};
  std::array<double, motor_count_> position_max_rps_{};
  std::vector<double> encoder_counts_per_revolution_;
  std::vector<int64_t> encoder_pulses_per_revolution_;
  std::array<uint16_t, motor_count_> encoder_ppr_{};
  std::array<double, motor_count_> encoder_gear_ratio_{};
  double communication_period_seconds_;
  bool direction_;
  bool allow_missing_pid_gains_;
  bool reset_encoder_origin_with_switch_;
  bool encoder_origin_switch_state_;
  double feedback_timeout_seconds_;
  std::vector<int64_t> encoder_polarities_;
  std::array<std::chrono::steady_clock::time_point, motor_count_> last_feedback_{};
  bool stop_mode_;
  bool reset_encoder_origin_;
  bool control_active_{false};
  bool csv_write_failed_{false};
  uint64_t feedback_sequence_{0};
  uint64_t command_sequence_{0};

  std::array<PidGain, motor_count_> position_gains_{};
  std::array<PidGain, motor_count_> velocity_gains_{};
  std::array<bool, motor_count_> position_gain_available_{};
  std::array<bool, motor_count_> velocity_gain_available_{};
  std::array<ControlMode, motor_count_> control_modes_{};

  rclcpp::TimerBase::SharedPtr terminal_timer_;
  rclcpp::TimerBase::SharedPtr communication_timer_;
  std::vector<rclcpp::Publisher<debug_msgs::msg::Feedback>::SharedPtr> feedback_publishers_;

  std::ofstream csv_log_;
  std::chrono::steady_clock::time_point log_start_time_;

  std::unique_ptr<LinuxHardwareSerial> dev_;
  std::unique_ptr<SerialBridge> serial_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr> raw_count_publishers_;
  natto_serial::SerialFrames frames_;
  std::array<stm32_protocol::InitializationMessage, motor_count_> &initialization_msg_{frames_.init};
  std::array<stm32_protocol::MotorCommandVelocityMessage, motor_count_> &velocity_msg_{frames_.velocity};
  std::array<stm32_protocol::MotorCommandPositionMessage, motor_count_> &position_msg_{frames_.position};
  std::array<stm32_protocol::MotorCommandDutyMessage, motor_count_> &duty_msg_{frames_.duty};
  std::array<stm32_protocol::MotorFeedbackMessage, motor_count_> &feedback_msg_{frames_.feedback};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    rclcpp::spin(std::make_shared<Stm32PosControlTest>());
  } catch (const std::exception & error) {
    exit_code = 1;
    std::cerr << "stm32_pos_control_test error: " << error.what() << std::endl;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
