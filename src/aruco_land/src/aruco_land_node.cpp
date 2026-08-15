#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <custom_interfaces/msg/offboard_status.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "main_control/landing/precision_lander.hpp"

// Thin ROS shell. All decision logic lives in aruco_land::PrecisionLander.
// This node only converts between ROS/PX4 messages and the brain's plain
// structs, runs the timer, and publishes what tick() returns.
//
// It is a *requester*: it commands the drone only through the neutral offboard
// API (/offboard/setpoint/position, /offboard/cmd/land). It never writes /fmu/in/*.
// (It does subscribe to /fmu/out/vehicle_odometry, an OUT topic, because it needs
// the drone attitude quaternion for the optical->NED transform — /offboard/state
// carries only position.)

using namespace std::chrono_literals;
using main_control::landing::PrecisionLander;

class ArucoLandNode : public rclcpp::Node
{
public:
  ArucoLandNode() : Node("aruco_land_node")
  {
    // --- Params -> PrecisionLander::Config ---
    PrecisionLander::Config cfg;
    cfg.search_alt        = declare_parameter<double>("search_alt",        cfg.search_alt);
    cfg.approach_alt      = declare_parameter<double>("approach_alt",      cfg.approach_alt);
    cfg.land_trigger_alt  = declare_parameter<double>("land_trigger_alt",  cfg.land_trigger_alt);
    cfg.descent_vel       = declare_parameter<double>("descent_vel",       cfg.descent_vel);
    cfg.p_gain            = declare_parameter<double>("vel_p_gain",        cfg.p_gain);
    cfg.i_gain            = declare_parameter<double>("vel_i_gain",        cfg.i_gain);
    cfg.max_vel           = declare_parameter<double>("max_velocity",      cfg.max_vel);
    cfg.target_timeout    = declare_parameter<double>("target_timeout",    cfg.target_timeout);
    cfg.delta_pos         = declare_parameter<double>("delta_position",    cfg.delta_pos);
    cfg.wp_accept         = declare_parameter<double>("wp_accept",         cfg.wp_accept);
    cfg.spiral_points     = declare_parameter<int>("spiral_points",        cfg.spiral_points);
    cfg.spiral_r_step     = declare_parameter<double>("spiral_r_step",     cfg.spiral_r_step);
    cfg.spiral_angle_step = declare_parameter<double>("spiral_angle_step", cfg.spiral_angle_step);
    lander_ = PrecisionLander(cfg);

    // Camera mount offset (base_link frame, m) used in the optical->NED transform.
    cam_off_x_ = declare_parameter<double>("camera_offset_x", 0.0);
    cam_off_y_ = declare_parameter<double>("camera_offset_y", 0.0);
    cam_off_z_ = declare_parameter<double>("camera_offset_z", -0.1);

    rclcpp::QoS latched(1);
    latched.transient_local().reliable().keep_last(1);

    // --- Publishers: neutral offboard API only ---
    pos_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/offboard/setpoint/position", 10);
    land_pub_ = create_publisher<std_msgs::msg::Bool>("/offboard/cmd/land", 10);

    // --- Subscriptions ---
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", rclcpp::QoS(10).best_effort(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_target_pose(msg); });

    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        drone_pos_ = Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]);
        drone_q_   = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        const double yaw = std::atan2(
          2.0 * (drone_q_.w() * drone_q_.z() + drone_q_.x() * drone_q_.y()),
          1.0 - 2.0 * (drone_q_.y() * drone_q_.y() + drone_q_.z() * drone_q_.z()));
        lander_.on_local_position({static_cast<float>(drone_pos_.x()),
                                   static_cast<float>(drone_pos_.y()),
                                   static_cast<float>(drone_pos_.z())});
        lander_.on_yaw(static_cast<float>(yaw));
      });

    // Gate on offboard_master's authority state (only run while it honors our
    // setpoints, i.e. HOVER). Same pattern as mission_node.
    offboard_state_sub_ = create_subscription<custom_interfaces::msg::OffboardStatus>(
      "/offboard/state", latched,
      [this](custom_interfaces::msg::OffboardStatus::SharedPtr msg) {
        accepting_setpoints_ = msg->accepting_setpoints;
      });

    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/aruco_land/start", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          RCLCPP_INFO(get_logger(), "Precision-landing start requested");
          lander_.on_start();
        }
      });

    timer_ = create_wall_timer(50ms, [this]() { on_timer(); });

    RCLCPP_INFO(get_logger(), "aruco_land_node started. State: %s",
      PrecisionLander::state_str(lander_.state()));
  }

private:
  PrecisionLander lander_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             land_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr  odom_sub_;
  rclcpp::Subscription<custom_interfaces::msg::OffboardStatus>::SharedPtr offboard_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            start_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Mirrored drone pose (for the optical->NED transform).
  Eigen::Vector3d    drone_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond drone_q_{Eigen::Quaterniond::Identity()};

  double cam_off_x_{0.0}, cam_off_y_{0.0}, cam_off_z_{-0.1};
  bool   accepting_setpoints_{false};
  PrecisionLander::State logged_state_{PrecisionLander::State::IDLE};

  double now_seconds() const { return now().nanoseconds() * 1e-9; }

  // Convert the marker pose (camera optical frame) to world NED, then feed the
  // brain. Ported verbatim from px4_basics_control landing_node targetPoseCallback:
  //   T_world = T_drone * T_cam * T_tag
  void on_target_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const Eigen::Vector3d pos(msg->pose.position.x,
                              msg->pose.position.y,
                              msg->pose.position.z);
    const Eigen::Quaterniond q(msg->pose.orientation.w,
                               msg->pose.orientation.x,
                               msg->pose.orientation.y,
                               msg->pose.orientation.z);

    Eigen::Matrix3d R;
    R << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;
    const Eigen::Quaterniond q_opt_to_ned(R);

    const Eigen::Affine3d T_cam =
      Eigen::Translation3d(cam_off_x_, cam_off_y_, cam_off_z_) * q_opt_to_ned;
    const Eigen::Affine3d T_drone = Eigen::Translation3d(drone_pos_) * drone_q_;
    const Eigen::Affine3d T_tag   = Eigen::Translation3d(pos) * q;
    const Eigen::Affine3d T_world = T_drone * T_cam * T_tag;

    const Eigen::Vector3d tag = T_world.translation();
    lander_.on_target_world({static_cast<float>(tag.x()),
                             static_cast<float>(tag.y()),
                             static_cast<float>(tag.z())},
                            now_seconds());
    RCLCPP_INFO_ONCE(get_logger(), "Precision landing: target pose received (world NED): %.2f, %.2f, %.2f",
      tag.x(), tag.y(), tag.z());
  }

  void on_timer()
  {
    // Pause until offboard_master is flying and honoring external setpoints.
    if (!accepting_setpoints_) return;

    const auto t = lander_.tick(now_seconds());

    if (t.has_setpoint) publish_position(t);
    if (t.request_land) {
      std_msgs::msg::Bool b;
      b.data = true;
      land_pub_->publish(b);
      RCLCPP_INFO(get_logger(), "Precision landing: land command issued");
    }

    if (t.state_changed || lander_.state() != logged_state_) {
      RCLCPP_INFO(get_logger(), "State: %s -> %s",
        PrecisionLander::state_str(logged_state_),
        PrecisionLander::state_str(lander_.state()));
      logged_state_ = lander_.state();
    }
  }

  void publish_position(const PrecisionLander::Tick & t)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.pose.position.x = t.position.x;
    msg.pose.position.y = t.position.y;
    msg.pose.position.z = t.position.z;
    const float half = 0.5f * t.yaw;
    msg.pose.orientation.w = std::cos(half);
    msg.pose.orientation.z = std::sin(half);
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    pos_pub_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoLandNode>());
  rclcpp::shutdown();
  return 0;
}
