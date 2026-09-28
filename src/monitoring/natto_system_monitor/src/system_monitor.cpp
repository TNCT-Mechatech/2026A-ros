#include <natto_system_monitor/health.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <natto_msgs/msg/navigation_status.hpp>
#include <natto_msgs/msg/navigation_path.hpp>
#include <natto_msgs/msg/navigation_feedback.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <functional>
#include <memory>
#include <map>
#include <set>
#include <vector>

namespace natto_system_monitor {
using Status = diagnostic_updater::DiagnosticStatusWrapper;
struct Target {
  std::string key, topic, type, node, error;
  bool enabled = true;
  double min_hz = 0, timeout = 1;
  Receipt receipt;
};

class SystemMonitor : public rclcpp::Node {
 public:
  SystemMonitor() : Node("system_monitor"), updater_(this, 1.0, Health::STALE) {
    updater_.setHardwareID("2026A");
    window_ = positive("frequency_window_sec", 2.0);
    grace_ = positive("startup_grace_sec", 3.0);
    path_warn_ = positive("path.warn_age_sec", 2.0);
    state_timeout_ = positive("state_timeout_sec", 1.0);
    quaternion_tolerance_ = positive("quaternion_norm_tolerance", 0.01);
    nan_fraction_ = parameter<double>("scan_max_nan_fraction", 0.5);
    if (!std::isfinite(nan_fraction_) || nan_fraction_ < 0 || nan_fraction_ > 1) throw std::invalid_argument("scan_max_nan_fraction must be in [0, 1]");
    managed_ = parameter<bool>("managed_navigation", false);
    front_ = target("front", "Sensors/Front LiDAR", "/sensing/lidar/laserscan/front", "sensor_msgs/msg/LaserScan", "", 24, 0.5);
    rear_ = target("rear", "Sensors/Rear LiDAR", "/sensing/lidar/laserscan/rear", "sensor_msgs/msg/LaserScan", "", 24, 0.5);
    pose_ = target("localization", "Localization/MCL", "/localization/current_pose", "geometry_msgs/msg/PoseStamped", "/localization/mcl/mcl", 30, 0.5);
    path_ = target("path", "Planning/Path", managed_ ? "/navigation/path" : "/planning/path", managed_ ? "natto_msgs/msg/NavigationPath" : "nav_msgs/msg/Path", "/planning/astar_planner", 0, 5);
    command_ = target("command", "Control/Pure Pursuit", managed_ ? "/navigation/feedback" : "/control/command_velocity", managed_ ? "natto_msgs/msg/NavigationFeedback" : "geometry_msgs/msg/TwistStamped", "/control/holonomic_pure_pursuit", 40, 0.5);
    for (auto *t : {front_, rear_}) {
      if (t->enabled) subscribe<sensor_msgs::msg::LaserScan>(t->topic, [this, t](const sensor_msgs::msg::LaserScan &m) {
        receive(*t, m.header.stamp); t->error = scan_error(m, nan_fraction_);
        scans_[t->key] = m;
      });
    }
    if (pose_->enabled) subscribe<geometry_msgs::msg::PoseStamped>(pose_->topic, [this](const geometry_msgs::msg::PoseStamped &m) {
      receive(*pose_, m.header.stamp); pose_msg_ = m;
      pose_->error = valid_pose(m.pose, quaternion_tolerance_) && !m.header.frame_id.empty() ? "" : "Invalid pose/frame/quaternion";
    });
    if (path_->enabled) {
      if (managed_) subscribe<natto_msgs::msg::NavigationPath>(path_->topic, [this](const natto_msgs::msg::NavigationPath &m) {
        path_request_ = m.request_id; on_path(m.path);
        path_valid_ = m.valid; path_reason_ = m.reason;
      });
      else subscribe<nav_msgs::msg::Path>(path_->topic, [this](const nav_msgs::msg::Path &m) { on_path(m); });
    }
    if (command_->enabled) {
      if (managed_) subscribe<natto_msgs::msg::NavigationFeedback>(command_->topic, [this](const natto_msgs::msg::NavigationFeedback &m) {
        command_request_ = m.request_id; on_command(m.command);
      });
      else subscribe<geometry_msgs::msg::TwistStamped>(command_->topic, [this](const geometry_msgs::msg::TwistStamped &m) { on_command(m); });
    }
    if (path_->enabled || command_->enabled) {
      if (managed_) {
        status_topic_ = parameter<std::string>("navigation_status_topic", "/navigation/status");
        manager_node_ = parameter<std::string>("navigation_manager_node", "/navigation/navigation_manager");
        subscribe<natto_msgs::msg::NavigationStatus>(status_topic_, [this](const natto_msgs::msg::NavigationStatus &m) {
          navigation_ = m; state_seen_ = true;
          active_ = m.state == "planning" || m.state == "moving";
        }, rclcpp::QoS(1).transient_local().reliable());
        subscribe<std_msgs::msg::Bool>(parameter<std::string>("allow_auto_drive_topic", "/controller/allow_auto_drive"), [this](const std_msgs::msg::Bool &) {
          heartbeat_.tick(now().seconds(), 0, window_);
        });
      } else {
        subscribe<geometry_msgs::msg::PoseStamped>(parameter<std::string>("goal_topic", "/goal_pose"), [this](const geometry_msgs::msg::PoseStamped &) {
          active_ = true; awaiting_path_ = true; goal_received_ = Clock::now();
        });
        subscribe<std_msgs::msg::Bool>(parameter<std::string>("goal_reached_topic", "/control/dynamic/goal_reached"), [this](const std_msgs::msg::Bool &m) {
          if (m.data) { active_ = false; awaiting_path_ = false; }
        });
      }
    }
    graph_monitor_node_ = parameter<std::string>("graph.monitor_node", "");
    graph_monitor_topic_ = parameter<std::string>("graph.monitor_topic", "/rosgraph");
    expected_nodes_ = parameter<std::vector<std::string>>("graph.expected_nodes", {});
    expected_publishers_ = parameter<std::vector<std::string>>("graph.publisher_topics", {});
    expected_subscribers_ = parameter<std::vector<std::string>>("graph.subscriber_topics", {});
    updater_.add("Control/ROS Graph expectations", this, &SystemMonitor::graph_status);
    for (const auto &key : {"inject", "roll", "mecanum"}) {
      Hardware h;
      h.key = key;
      h.enabled = parameter<bool>(h.key + ".enabled", false);
      const std::string source = h.key == "mecanum" ? "natto_robot_hardware/serial_motor_bridge" : h.key + "_serial_node: Hardware/Serial";
      h.source = parameter<std::string>(h.key + ".diagnostic_name", source);
      h.device = parameter<std::string>(h.key + ".device", "");
      h.timeout = positive(h.key + ".timeout_sec", 3.0);
      hardware_.push_back(h);
    }
    for (size_t i = 0; i < hardware_.size(); ++i) {
      updater_.add("Hardware/" + hardware_[i].key + " heartbeat", [this, i](Status &s) { hardware_status(hardware_[i], s); });
      updater_.add("Hardware/" + hardware_[i].key + " device", [this, i](Status &s) { device_status(hardware_[i], s); });
    }
    subscribe<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", [this](const diagnostic_msgs::msg::DiagnosticArray &m) {
      for (const auto &s : m.status) for (auto &h : hardware_) if (h.enabled && s.name == h.source) {
        h.receipt.tick(now().seconds(), rclcpp::Time(m.header.stamp).seconds(), window_); h.status = s;
      }
    }, rclcpp::QoS(100));
  }

 private:
  template<class T> T parameter(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;  // Explicit startup configuration; no silently ignored runtime writes.
    return declare_parameter<T>(name, value, descriptor);
  }
  double positive(const std::string &name, double value) {
    const double result = parameter<double>(name, value);
    if (!std::isfinite(result) || result <= 0) throw std::invalid_argument(name + " must be positive and finite");
    return result;
  }
  template<class M, class F> void subscribe(const std::string &topic, F callback, rclcpp::QoS qos = rclcpp::SensorDataQoS()) {
    subscriptions_.push_back(create_subscription<M>(topic, qos, [callback](typename M::ConstSharedPtr msg) { callback(*msg); }));
  }
  Target *target(const std::string &key, const std::string &name, const std::string &topic,
                 const std::string &type, const std::string &node, double hz, double timeout) {
    auto t = std::make_unique<Target>();
    t->key = key; t->type = type;
    t->enabled = parameter<bool>(key + ".enabled", true);
    t->topic = parameter<std::string>(key + ".topic", topic);
    t->node = parameter<std::string>(key + ".node", node);
    t->min_hz = parameter<double>(key + ".min_hz", hz);
    if (!std::isfinite(t->min_hz) || t->min_hz < 0) throw std::invalid_argument(key + ".min_hz must be nonnegative and finite");
    t->timeout = positive(key + ".timeout_sec", timeout);
    auto *ptr = t.get(); targets_.push_back(std::move(t));
    updater_.add(name, [this, ptr](Status &s) { topic_status(*ptr, s); });
    return ptr;
  }
  void receive(Target &t, const builtin_interfaces::msg::Time &stamp) {
    t.receipt.tick(now().seconds(), rclcpp::Time(stamp).seconds(), window_);
  }
  void on_path(const nav_msgs::msg::Path &m) {
    receive(*path_, m.header.stamp); path_msg_ = m; awaiting_path_ = false;
    path_->error.clear();
    if (!m.poses.empty()) {
      if (m.header.frame_id.empty() || m.poses.size() < 2) path_->error = "Invalid path frame/point count";
      for (const auto &p : m.poses) if (!valid_pose(p.pose, quaternion_tolerance_) ||
          (!p.header.frame_id.empty() && p.header.frame_id != m.header.frame_id)) path_->error = "Invalid path pose/frame/quaternion";
    }
    if (!managed_) active_ = !m.poses.empty();
  }
  void on_command(const geometry_msgs::msg::TwistStamped &m) {
    receive(*command_, m.header.stamp); command_msg_ = m;
    const auto &t = m.twist;
    command_->error = std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.linear.z) &&
      std::isfinite(t.angular.x) && std::isfinite(t.angular.y) && std::isfinite(t.angular.z) ? "" : "Non-finite command";
  }
  bool node_present(const std::string &node) const {
    if (node.empty()) return true;
    const auto names = get_node_names();
    return std::find(names.begin(), names.end(), node) != names.end();
  }
  bool publisher_present(const Target &t) const {
    const auto endpoints = get_publishers_info_by_topic(t.topic);
    return std::any_of(endpoints.begin(), endpoints.end(), [&t](const auto &e) { return e.topic_type() == t.type; });
  }
  bool state_known() const {
    const bool recognized = navigation_.state == "idle" || navigation_.state == "planning" ||
      navigation_.state == "moving" || navigation_.state == "arrived" ||
      navigation_.state == "canceled" || navigation_.state == "failed";
    return !managed_ || (state_seen_ && recognized && count_publishers(status_topic_) > 0 && node_present(manager_node_) &&
      heartbeat_.seen && heartbeat_.age() <= state_timeout_);
  }
  std::string operational() const {
    if (!state_known()) return "UNKNOWN";
    if (managed_ && navigation_.state == "planning") return "INITIALIZING";
    if (managed_ && (navigation_.state == "failed" || navigation_.state == "canceled")) return "STOPPED";
    return active_ ? "RUNNING" : "IDLE";
  }
  void timing(const Receipt &r, Status &s) const {
    const double hz = r.hz();
    s.add("received", r.seen); s.add("receive_hz", hz);
    s.add("receive_period_sec", hz > 0 ? 1.0 / hz : -1.0);
    s.add("age_sec", r.age()); s.add("last_received_ros_sec", std::to_string(r.ros_received));
    s.add("source_stamp_sec", std::to_string(r.stamp));
  }
  void topic_status(Target &t, Status &s) {
    timing(t.receipt, s); s.add("topic", t.topic); s.add("type", t.type); s.add("node", t.node);
    s.add("timeout_sec", t.timeout); s.add("min_hz", t.min_hz);
    const bool lifecycle = &t == path_ || &t == command_;
    std::string state = lifecycle ? operational() : "RUNNING";
    unsigned char level = Health::OK;
    std::string message = "Receiving valid data";
    const bool publisher = publisher_present(t), node = node_present(t.node);
    s.add("publisher_present", publisher); s.add("node_present", node);
    if (!t.enabled) { state = "DISABLED"; message = "Monitoring disabled"; }
    else if (!publisher || !node) { level = Health::STALE; state = "UNKNOWN"; message = "Expected node/publisher unavailable"; }
    else if (lifecycle && !state_known()) { level = Health::STALE; state = "UNKNOWN"; message = "Navigation state unavailable"; }
    else if (lifecycle && managed_ && navigation_.state == "failed") { level = Health::ERROR; message = navigation_.reason; }
    else if (t.receipt.seen && t.receipt.age() <= t.timeout && !t.error.empty() &&
             (!managed_ || !lifecycle || (&t == path_ ? path_request_ : command_request_) == navigation_.request_id)) {
      level = Health::ERROR; message = t.error;
    }
    else if (lifecycle && !active_) { message = "No active goal; periodic data not required"; }
    else {
      const bool matching = !managed_ || !lifecycle || (&t == path_ ? path_request_ : command_request_) == navigation_.request_id;
      if (!t.receipt.seen || t.receipt.age() > t.timeout || !matching || (&t == path_ && awaiting_path_)) {
        level = Health::STALE; state = "UNKNOWN"; message = "No current data / receive timeout";
        if (seconds(Clock::now() - started_) < grace_ || (awaiting_path_ && seconds(Clock::now() - goal_received_) < grace_)) state = "INITIALIZING";
      } else if (!t.error.empty()) { level = Health::ERROR; message = t.error; }
      else if (&t == path_ && (path_msg_.poses.empty() || (managed_ && !path_valid_))) {
        level = Health::ERROR; message = path_reason_.empty() ? "Active goal has invalid/empty path" : path_reason_;
      }
      else if (&t == path_ && t.receipt.age() > path_warn_) { level = Health::WARN; message = "Path update delayed"; }
      else if (t.min_hz > 0 && t.receipt.samples.size() > 1 && t.receipt.hz() < t.min_hz) { level = Health::WARN; message = "Receive frequency below minimum"; }
      else if (t.min_hz > 0 && t.receipt.samples.size() < 2) { level = Health::STALE; state = "INITIALIZING"; message = "Collecting frequency samples"; }
    }
    if (scans_.count(t.key)) {
      const auto &m = scans_.at(t.key); s.add("frame_id", m.header.frame_id);
      s.add("range_min", m.range_min); s.add("range_max", m.range_max); s.add("point_count", m.ranges.size());
    }
    if (&t == pose_) { s.add("frame_id", pose_msg_.header.frame_id); s.add("x", pose_msg_.pose.position.x); s.add("y", pose_msg_.pose.position.y); }
    if (&t == path_) { s.add("point_count", path_msg_.poses.size()); s.add("path_exists", !path_msg_.poses.empty()); s.add("path_valid", !path_msg_.poses.empty() && t.error.empty() && (!managed_ || path_valid_)); s.add("path_reason", path_reason_); s.add("frame_id", path_msg_.header.frame_id); }
    if (&t == command_) { s.add("vx", command_msg_.twist.linear.x); s.add("vy", command_msg_.twist.linear.y); s.add("wz", command_msg_.twist.angular.z); }
    if (lifecycle && managed_) { s.add("navigation_state", navigation_.state); s.add("request_id", navigation_.request_id); }
    s.add("operational_state", state); s.summary(level, message);
  }
  void graph_status(Status &s) {
    std::string missing;
    if (!graph_monitor_node_.empty() && (!node_present(graph_monitor_node_) || count_publishers(graph_monitor_topic_) == 0))
      missing += " graph_monitor:" + graph_monitor_node_;
    for (const auto &n : expected_nodes_) if (!node_present(n)) missing += " node:" + n;
    for (const auto &t : targets_) if (t->enabled && (!publisher_present(*t) || !node_present(t->node))) missing += " target:" + t->key;
    for (const auto &t : expected_publishers_) if (count_publishers(t) == 0) missing += " publisher:" + t;
    for (const auto &t : expected_subscribers_) {
      const auto endpoints = get_subscriptions_info_by_topic(t);
      const bool other = std::any_of(endpoints.begin(), endpoints.end(), [this](const auto &e) {
        return e.node_name() != get_name() || e.node_namespace() != get_namespace();
      });
      if (!other) missing += " subscriber:" + t;
    }
    s.add("missing", missing); s.add("node_count", get_node_names().size());
    s.add("operational_state", missing.empty() ? "RUNNING" : "UNKNOWN");
    s.summary(missing.empty() ? Health::OK : Health::STALE, missing.empty() ? "Expected graph entities present" : "Expected graph entities unavailable");
  }
  struct Hardware {
    std::string key, source, device;
    bool enabled = false;
    double timeout = 3;
    Receipt receipt;
    diagnostic_msgs::msg::DiagnosticStatus status;
  };
  void hardware_status(const Hardware &h, Status &s) {
    timing(h.receipt, s); s.add("source", h.source);
    if (!h.enabled) { s.summary(Health::OK, "Monitoring disabled"); s.add("operational_state", "DISABLED"); }
    else if (!h.receipt.seen || h.receipt.age() > h.timeout) { s.summary(Health::STALE, "Hardware diagnostics heartbeat missing"); s.add("operational_state", "UNKNOWN"); }
    else {
      s.summary(h.status.level, h.status.message);
      s.values.insert(s.values.end(), h.status.values.begin(), h.status.values.end());
    }
  }
  void device_status(const Hardware &h, Status &s) {
    s.add("device", h.device);
    if (!h.enabled || h.device.empty()) { s.summary(Health::OK, "Device path check disabled"); s.add("operational_state", "DISABLED"); return; }
    std::error_code ec;
    const bool exists = std::filesystem::exists(h.device, ec);
    s.summary(ec ? Health::STALE : (exists ? Health::OK : Health::ERROR), ec ? ec.message() : (exists ? "Device path exists (not an open/communication test)" : "Serial device path missing"));
    s.add("operational_state", exists ? "READY" : "UNKNOWN");
  }
  diagnostic_updater::Updater updater_;
  std::vector<std::unique_ptr<Target>> targets_;
  Target *front_, *rear_, *pose_, *path_, *command_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string, sensor_msgs::msg::LaserScan> scans_;
  geometry_msgs::msg::PoseStamped pose_msg_;
  nav_msgs::msg::Path path_msg_;
  geometry_msgs::msg::TwistStamped command_msg_;
  natto_msgs::msg::NavigationStatus navigation_;
  std::vector<Hardware> hardware_;
  std::vector<std::string> expected_nodes_, expected_publishers_, expected_subscribers_;
  std::string status_topic_, manager_node_, path_request_, command_request_, graph_monitor_node_, graph_monitor_topic_, path_reason_;
  Receipt heartbeat_;
  bool managed_ = false, state_seen_ = false, active_ = false, awaiting_path_ = false, path_valid_ = true;
  double window_, grace_, path_warn_, state_timeout_, quaternion_tolerance_, nan_fraction_;
  Clock::time_point started_ = Clock::now(), goal_received_{};
};
}  // namespace natto_system_monitor

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<natto_system_monitor::SystemMonitor>()); }
  catch (const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("system_monitor"), "%s", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
