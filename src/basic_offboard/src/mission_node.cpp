#include <cmath>
#include <chrono>
#include <functional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include "custom_interfaces/msg/waypoints.hpp"
#include "custom_interfaces/srv/req_mode.hpp"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// Mission sequence:
//   ENDURANCE  (FW preferred) -> desired_laps laps (this can be defined with the launch file) done  -> MAPPING
//   MAPPING    (FW preferred) -> drop area detected      -> DROPPING
//   DROPPING   (MC preferred) -> drop waypoint reached   -> MAPPING (resume)

class MissionNode : public rclcpp::Node
{
public:
  enum class Mission { ENDURANCE, MAPPING, DROPPING };

  MissionNode() : Node("mission_node")
  {
    declare_parameter("desired_laps", 3);
    desired_laps_ = get_parameter("desired_laps").as_int();

    rclcpp::QoS px4_qos(1);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

    // Publishers
    pos_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/offboard/setpoint/position", 10);

    rclcpp::QoS state_qos(1);
    state_qos.transient_local().reliable();
    mission_state_pub_ = create_publisher<std_msgs::msg::UInt8>("/mission_state", state_qos);

    // PX4 state subscriptions
    odometry_sub_ = create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      [this](VehicleOdometry::UniquePtr msg) {
        local_pos_[0] = msg->position[0];
        local_pos_[1] = msg->position[1];
        local_pos_[2] = msg->position[2];
      });

    vehicle_status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status", px4_qos,
      [this](VehicleStatus::UniquePtr msg) { vehicle_status_ = *msg; });

    // Waypoint subscriptions
    rclcpp::QoS wp_qos(1);
    wp_qos.transient_local().reliable();

    endu_wp_sub_ = create_subscription<custom_interfaces::msg::Waypoints>(
      "/waypoints/endu", wp_qos,
      std::bind(&MissionNode::endu_wp_callback, this, std::placeholders::_1));

    map_wp_sub_ = create_subscription<custom_interfaces::msg::Waypoints>(
      "/waypoints/map", wp_qos,
      std::bind(&MissionNode::map_wp_callback, this, std::placeholders::_1));

    drop_area_sub_ = create_subscription<custom_interfaces::msg::Waypoints>(
      "/drop_area", wp_qos,
      std::bind(&MissionNode::drop_area_callback, this, std::placeholders::_1));

    // Service client for mode requests
    req_mode_client_ = create_client<custom_interfaces::srv::ReqMode>(
      "/offboard/srv/req_mode");

    timer_ = create_wall_timer(100ms, std::bind(&MissionNode::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Mission Node started. Desired laps: %d", desired_laps_);
  }

private:
  using ReqMode   = custom_interfaces::srv::ReqMode;
  using Waypoints = custom_interfaces::msg::Waypoints;

  Mission  mission_{Mission::ENDURANCE};

  // Waypoint storage
  std::vector<geometry_msgs::msg::PoseStamped> endu_waypoints_;
  std::vector<geometry_msgs::msg::PoseStamped> map_waypoints_;
  geometry_msgs::msg::PoseStamped              drop_waypoint_;
  bool drop_pending_{false};

  // Indices
  size_t endu_idx_{0};
  size_t map_idx_{0};
  size_t map_resume_idx_{0};

  // Endurance lap tracking
  int lap_count_{0};
  int desired_laps_{3};

  // Vehicle state
  float        local_pos_[3]{0.0f, 0.0f, 0.0f};
  VehicleStatus vehicle_status_{};

  // Set to true while waiting for MC mode confirmation before flying to drop area
  bool waiting_for_mc_{false};

  // Acceptance radii in metres (NED 3-D distance)
  static constexpr float MC_RADIUS = 1.0f;
  static constexpr float FW_RADIUS = 10.0f;

  // Subscriptions
  rclcpp::Subscription<VehicleOdometry>::SharedPtr              odometry_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr                vehicle_status_sub_;
  rclcpp::Subscription<Waypoints>::SharedPtr                    endu_wp_sub_;
  rclcpp::Subscription<Waypoints>::SharedPtr                    map_wp_sub_;
  rclcpp::Subscription<Waypoints>::SharedPtr                    drop_area_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr            mission_state_pub_;
  rclcpp::Client<ReqMode>::SharedPtr                            req_mode_client_;
  rclcpp::TimerBase::SharedPtr                                  timer_;

  // --- Waypoint callbacks ---

  void endu_wp_callback(const Waypoints::SharedPtr msg)
  {
    endu_waypoints_ = msg->waypoints;
    endu_idx_ = 0;
    RCLCPP_INFO(get_logger(), "Received %zu endurance waypoints", endu_waypoints_.size());
  }

  void map_wp_callback(const Waypoints::SharedPtr msg)
  {
    map_waypoints_ = msg->waypoints;
    map_idx_ = 0;
    RCLCPP_INFO(get_logger(), "Received %zu mapping waypoints", map_waypoints_.size());
  }

  void drop_area_callback(const Waypoints::SharedPtr msg)
  {
    if (msg->waypoints.empty()) return;
    drop_waypoint_ = msg->waypoints[0];

    if (mission_ == Mission::MAPPING && !drop_pending_) {
      drop_pending_ = true;
      RCLCPP_INFO(get_logger(), "Drop area received — will divert after current waypoint");
    }
  }

  // --- Mode request (fire-and-forget async) ---

  void request_mode(uint8_t mode)
  {
    if (!vehicle_status_.is_vtol) return;  // mode switching only relevant for VTOL

    if (!req_mode_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "req_mode service not ready, skipping mode request");
      return;
    }
    auto req  = std::make_shared<ReqMode::Request>();
    req->mode = mode;
    req_mode_client_->async_send_request(
      req,
      [this, mode](rclcpp::Client<ReqMode>::SharedFuture future) {
        auto res = future.get();
        const char * mode_str = (mode == ReqMode::Request::MODE_FW) ? "FW" : "MC";
        if (res->result == ReqMode::Response::RESULT_OK) {
          RCLCPP_INFO(get_logger(), "Mode %s request accepted", mode_str);
        } else {
          RCLCPP_WARN(get_logger(), "Mode %s request rejected", mode_str);
        }
      });
  }

  // --- Timer: top-level dispatcher ---

  void timer_callback()
  {
    switch (mission_) {
      case Mission::ENDURANCE: run_endurance(); break;
      case Mission::MAPPING:   run_mapping();   break;
      case Mission::DROPPING:  run_dropping();  break;
    }
  }

  // --- Mission runners ---

  void run_endurance()
  {
    if (endu_waypoints_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No endurance waypoints received yet");
      return;
    }

    publish_waypoint(endu_waypoints_[endu_idx_]);
    request_mode(ReqMode::Request::MODE_FW);

    const float radius = is_fw_mode() ? FW_RADIUS : MC_RADIUS;
    if (distance_to(endu_waypoints_[endu_idx_]) > radius) return;

    // Reached current endurance waypoint
    endu_idx_++;
    if (endu_idx_ < endu_waypoints_.size()) return;

    // Completed a lap
    endu_idx_ = 0;
    lap_count_++;
    RCLCPP_INFO(get_logger(), "Endurance lap %d/%d complete", lap_count_, desired_laps_);

    if (lap_count_ >= desired_laps_) {
      RCLCPP_INFO(get_logger(), "Endurance done — switching to MAPPING");
      transition_to(Mission::MAPPING);
    }
  }

  void run_mapping()
  {
    if (map_waypoints_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No mapping waypoints received yet");
      return;
    }

    publish_waypoint(map_waypoints_[map_idx_]);

    const float radius = is_fw_mode() ? FW_RADIUS : MC_RADIUS;
    if (distance_to(map_waypoints_[map_idx_]) > radius) return;

    // Reached current mapping waypoint — advance (looping)
    map_idx_ = (map_idx_ + 1) % map_waypoints_.size();

    if (drop_pending_) {
      drop_pending_    = false;
      map_resume_idx_  = map_idx_;
      RCLCPP_INFO(get_logger(), "Diverting to DROPPING (map resume idx: %zu)", map_resume_idx_);
      transition_to(Mission::DROPPING);
    }
  }

  void run_dropping()
  {
    // Block until MC mode is confirmed
    if (waiting_for_mc_) {
      if (!is_mc_mode()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for MC mode before flying to drop area...");
        return;
      }
      waiting_for_mc_ = false;
      RCLCPP_INFO(get_logger(), "MC mode confirmed — flying to drop area");
    }

    publish_waypoint(drop_waypoint_);

    if (distance_to(drop_waypoint_) > MC_RADIUS) return;

    RCLCPP_INFO(get_logger(), "Drop area reached — resuming MAPPING at idx %zu", map_resume_idx_);
    map_idx_ = map_resume_idx_;
    transition_to(Mission::MAPPING);
  }

  // Mission transition

  void transition_to(Mission next)
  {
    RCLCPP_INFO(get_logger(), "Mission: %s -> %s",
      mission_to_str(mission_), mission_to_str(next));
    mission_ = next;

    std_msgs::msg::UInt8 state_msg;
    state_msg.data = static_cast<uint8_t>(next);
    mission_state_pub_->publish(state_msg);

    switch (next) {
      case Mission::ENDURANCE:
      case Mission::MAPPING:
        request_mode(ReqMode::Request::MODE_FW);
        break;
      case Mission::DROPPING:
        waiting_for_mc_ = true;
        request_mode(ReqMode::Request::MODE_MC);
        break;
    }
  }

  // Helpers

  void publish_waypoint(const geometry_msgs::msg::PoseStamped & wp)
  {
    geometry_msgs::msg::PoseStamped msg = wp;
    msg.header.stamp = now();
    pos_pub_->publish(msg);
  }

  float distance_to(const geometry_msgs::msg::PoseStamped & wp) const
  {
    const float dx = local_pos_[0] - static_cast<float>(wp.pose.position.x);
    const float dy = local_pos_[1] - static_cast<float>(wp.pose.position.y);
    const float dz = local_pos_[2] - static_cast<float>(wp.pose.position.z);
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  bool is_fw_mode() const
  {
    return vehicle_status_.vehicle_type == VehicleStatus::VEHICLE_TYPE_FIXED_WING;
  }

  bool is_mc_mode() const
  {
    return vehicle_status_.vehicle_type == VehicleStatus::VEHICLE_TYPE_ROTARY_WING
        && !vehicle_status_.in_transition_mode;
  }

  static const char * mission_to_str(Mission m)
  {
    switch (m) {
      case Mission::ENDURANCE: return "ENDURANCE";
      case Mission::MAPPING:   return "MAPPING";
      case Mission::DROPPING:  return "DROPPING";
      default:                 return "UNKNOWN";
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
