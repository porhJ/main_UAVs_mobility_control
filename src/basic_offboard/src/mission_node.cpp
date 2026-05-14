#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "basic_offboard/mission_planner.hpp"
#include "custom_interfaces/msg/waypoints.hpp"

// Thin ROS shell. All mission logic lives in basic_offboard::MissionPlanner.

using namespace std::chrono_literals;
using basic_offboard::MissionPlanner;

class MissionNode : public rclcpp::Node
{
public:
  MissionNode() : Node("mission_node")
  {
    declare_parameter("desired_laps", 3);
    planner_.set_desired_laps(static_cast<int>(get_parameter("desired_laps").as_int()));

    rclcpp::QoS px4_qos(1);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

    rclcpp::QoS latched(1);
    latched.transient_local().reliable();

    pos_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/offboard/setpoint/position", 10);
    mission_state_pub_ = create_publisher<std_msgs::msg::UInt8>("/mission_state", latched);

    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", px4_qos,
      [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        planner_.on_local_position(msg->position[0], msg->position[1], msg->position[2]);
      });

    endu_sub_ = create_subscription<custom_interfaces::msg::Waypoints>(
      "/waypoints/endu", latched,
      [this](custom_interfaces::msg::Waypoints::SharedPtr msg) {
        auto wps = convert(msg->waypoints);
        RCLCPP_INFO(get_logger(), "Received %zu endurance waypoints", wps.size());
        planner_.set_endurance_waypoints(std::move(wps));
      });

    map_sub_ = create_subscription<custom_interfaces::msg::Waypoints>(
      "/waypoints/map", latched,
      [this](custom_interfaces::msg::Waypoints::SharedPtr msg) {
        auto wps = convert(msg->waypoints);
        RCLCPP_INFO(get_logger(), "Received %zu mapping waypoints", wps.size());
        planner_.set_mapping_waypoints(std::move(wps));
      });

    drop_sub_ = create_subscription<custom_interfaces::msg::Waypoints>(
      "/drop_area", latched,
      [this](custom_interfaces::msg::Waypoints::SharedPtr msg) {
        if (msg->waypoints.empty()) return;
        planner_.on_drop_area(pose_to_wp(msg->waypoints[0]));
        RCLCPP_INFO(get_logger(), "Drop area received");
      });

    timer_ = create_wall_timer(100ms, [this]() { on_timer(); });

    RCLCPP_INFO(get_logger(), "MissionNode started. Desired laps: %d",
      static_cast<int>(get_parameter("desired_laps").as_int()));
  }

private:
  MissionPlanner planner_;
  MissionPlanner::Mission logged_mission_{MissionPlanner::Mission::ENDURANCE};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pos_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             mission_state_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Waypoints>::SharedPtr endu_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Waypoints>::SharedPtr map_sub_;
  rclcpp::Subscription<custom_interfaces::msg::Waypoints>::SharedPtr drop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  static MissionPlanner::Waypoint pose_to_wp(const geometry_msgs::msg::PoseStamped & p)
  {
    const auto & q = p.pose.orientation;
    const float yaw = static_cast<float>(
      std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
    return {
      static_cast<float>(p.pose.position.x),
      static_cast<float>(p.pose.position.y),
      static_cast<float>(p.pose.position.z),
      yaw};
  }

  static std::vector<MissionPlanner::Waypoint>
  convert(const std::vector<geometry_msgs::msg::PoseStamped> & ps)
  {
    std::vector<MissionPlanner::Waypoint> out;
    out.reserve(ps.size());
    for (const auto & p : ps) out.push_back(pose_to_wp(p));
    return out;
  }

  void publish_waypoint(const MissionPlanner::Waypoint & w)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.pose.position.x = w.x;
    msg.pose.position.y = w.y;
    msg.pose.position.z = w.z;
    const float half = 0.5f * w.yaw;
    msg.pose.orientation.w = std::cos(half);
    msg.pose.orientation.z = std::sin(half);
    pos_pub_->publish(msg);
  }

  static const char * mission_str(MissionPlanner::Mission m)
  {
    switch (m) {
      case MissionPlanner::Mission::ENDURANCE: return "ENDURANCE";
      case MissionPlanner::Mission::MAPPING:   return "MAPPING";
      case MissionPlanner::Mission::DROPPING:  return "DROPPING";
    }
    return "UNKNOWN";
  }

  void on_timer()
  {
    const auto t = planner_.tick();

    if (t.has_setpoint) publish_waypoint(t.setpoint);

    if (t.mission_changed) {
      RCLCPP_INFO(get_logger(), "Mission: %s -> %s",
        mission_str(logged_mission_), mission_str(t.mission));
      logged_mission_ = t.mission;

      std_msgs::msg::UInt8 m;
      m.data = static_cast<uint8_t>(t.mission);
      mission_state_pub_->publish(m);
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
