#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <custom_interfaces/msg/offboard_status.hpp>
#include <custom_interfaces/msg/mode_setpoint.hpp>

#include "basic_offboard/flight_controller.hpp"

// Thin ROS shell. All decision logic lives in basic_offboard::FlightController.
// This node only translates between ROS topics and the controller API,
// and publishes whatever the controller's tick() asks for.

using namespace std::chrono_literals;
using basic_offboard::FlightController;

class OffboardMaster : public rclcpp::Node
{
public:
  OffboardMaster() : Node("offboard_master")
  {
    // Geofence config from params (axis-aligned box in PX4 local NED).
    // Enabled by default here so deployed flights are protected; the pure
    // library default is fence-off.
    FlightController::Config cfg;
    cfg.fence_enabled   = declare_parameter("fence.enabled", true);
    cfg.fence_x_min     = declare_parameter<double>("fence.x_min", cfg.fence_x_min);
    cfg.fence_x_max     = declare_parameter<double>("fence.x_max", cfg.fence_x_max);
    cfg.fence_y_min     = declare_parameter<double>("fence.y_min", cfg.fence_y_min);
    cfg.fence_y_max     = declare_parameter<double>("fence.y_max", cfg.fence_y_max);
    cfg.fence_z_ceiling = declare_parameter<double>("fence.z_ceiling", cfg.fence_z_ceiling);
    cfg.fence_z_floor   = declare_parameter<double>("fence.z_floor", cfg.fence_z_floor);
    cfg.fence_margin    = declare_parameter<double>("fence.margin", cfg.fence_margin);
    cfg.takeoff_z       = declare_parameter<double>("takeoff_z", -2.5);
    fc_ = FlightController(cfg);

    rclcpp::QoS px4_qos(1);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

    // Latched QoS so late-joining requesters immediately get the current state.
    rclcpp::QoS latched(1);
    latched.transient_local().reliable().keep_last(1);

    // Publishers to PX4

    offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4_qos);
    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", px4_qos);
    traj_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_qos);

    // State feedback for external requesters (mission_node, imav26 missions, ...)
    status_pub_ = create_publisher<custom_interfaces::msg::OffboardStatus>(
      "/offboard/state", latched);

    // Subscriptions to sensors      
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        const bool disarmed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
        fc_.on_disarmed(disarmed);
        fc_.on_nav_state(msg->nav_state);
        armed_     = !disarmed;
        nav_state_ = msg->nav_state;
      });

    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        fc_.on_local_position({msg->position[0], msg->position[1], msg->position[2]});
      });

    // External commands from other nodes
    
    ext_land_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/offboard/cmd/land", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          RCLCPP_INFO(get_logger(), "Land requested");
          fc_.on_land_command();
        }
      });

    ext_pos_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/offboard/setpoint/position", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        const auto & q = msg->pose.orientation;
        const float yaw = static_cast<float>(
          std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
        fc_.on_pos_setpoint(
          {static_cast<float>(msg->pose.position.x),
           static_cast<float>(msg->pose.position.y),
           static_cast<float>(msg->pose.position.z)},
          yaw);
      });

    ext_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/offboard/setpoint/velocity", 10,
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        fc_.on_vel_setpoint(
          {static_cast<float>(msg->twist.linear.x),
           static_cast<float>(msg->twist.linear.y),
           static_cast<float>(msg->twist.linear.z),
           static_cast<float>(msg->twist.angular.z)},
          now_seconds());
      });

    ext_mode_sub_ = create_subscription<custom_interfaces::msg::ModeSetpoint>(
      "/offboard/setpoint/mode", 10,
      [this](custom_interfaces::msg::ModeSetpoint::SharedPtr msg) {
        fc_.on_mode(static_cast<uint8_t>(msg->mode));
        // Feed exactly one setpoint kind, per the message's type field. Feeding
        // both would let the velocity path clobber the position path in hover.
        if (msg->type == custom_interfaces::msg::ModeSetpoint::TYPE_VELOCITY) {
          fc_.on_vel_setpoint(
            {static_cast<float>(msg->velocity.x),
             static_cast<float>(msg->velocity.y),
             static_cast<float>(msg->velocity.z),
             static_cast<float>(msg->yawspeed)},
            now_seconds());
        } else {
          fc_.on_pos_setpoint(
            {static_cast<float>(msg->position.x),
             static_cast<float>(msg->position.y),
             static_cast<float>(msg->position.z)},
            static_cast<float>(msg->yaw));
        }
      });


    // main loop timer
    timer_ = create_wall_timer(100ms, std::bind(&OffboardMaster::on_timer, this));

    RCLCPP_INFO(get_logger(), "OffboardMaster started. State: %s",
      FlightController::state_str(fc_.state()));
  }

private:
  FlightController fc_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      cmd_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  traj_pub_;
  rclcpp::Publisher<custom_interfaces::msg::OffboardStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr    vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr  odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             ext_land_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ext_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ext_vel_sub_;
  rclcpp::Subscription<custom_interfaces::msg::ModeSetpoint>::SharedPtr ext_mode_sub_;
  rclcpp::TimerBase::SharedPtr                                     timer_;

  FlightController::State logged_state_{FlightController::State::LANDED};

  // Mirrored from /fmu/out/vehicle_status for the status message.
  bool    armed_{false};
  uint8_t nav_state_{0};
  int     status_decimator_{0};   // publish status every Nth tick (2 Hz at 10 Hz)

  double now_seconds() const { return now().nanoseconds() * 1e-9; }
  uint64_t now_us() const    { return now().nanoseconds() / 1000; }

  void on_timer()
  {
    const auto t = fc_.tick(now_seconds()); // tick is timer function inside flight_controller.cpp

    publish_heartbeat(t.heartbeat_use_velocity);

    if (t.send_arm)           send_arm();
    if (t.send_offboard_mode) send_offboard_mode();
    if (t.send_land)          send_land();

    if (t.publish_setpoint)   publish_trajectory(t);

    // Publish state feedback on every transition, plus a 2 Hz heartbeat so
    // requesters can detect a dead master via the stamp.
    if (t.state_changed || (++status_decimator_ % 5) == 0) {
      publish_status();
    }

    if (t.send_mode_transition) {
      send_vtol_transition((fc_.state() == FlightController::State::CRUISE)
                            ? FlightController::FlightMode::CRUISE
                            : FlightController::FlightMode::HOVER);
    }

    if (fc_.state() != logged_state_) {
      RCLCPP_INFO(get_logger(), "State: %s -> %s",
        FlightController::state_str(logged_state_),
        FlightController::state_str(fc_.state()));
      logged_state_ = fc_.state();
    }
  }

  void publish_status()
  {
    custom_interfaces::msg::OffboardStatus msg{};
    msg.stamp               = now();
    msg.state               = static_cast<uint8_t>(fc_.state());
    msg.armed               = armed_;
    msg.offboard_active     = (nav_state_ == FlightController::NAV_STATE_OFFBOARD);
    msg.accepting_setpoints = fc_.accepts_setpoints();
    msg.mode                = (fc_.state() == FlightController::State::CRUISE)
                                ? custom_interfaces::msg::OffboardStatus::MODE_CRUISE
                                : custom_interfaces::msg::OffboardStatus::MODE_HOVER;
    msg.geofence_status     = fc_.geofence_status();
    const auto p            = fc_.local_position();
    msg.position.x          = p.x;
    msg.position.y          = p.y;
    msg.position.z          = p.z;
    status_pub_->publish(msg);
  }

  void publish_heartbeat(bool use_velocity)
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position  = !use_velocity;
    msg.velocity  = use_velocity;
    msg.timestamp = now_us();
    offboard_pub_->publish(msg);
  }

  void publish_trajectory(const FlightController::Tick & t)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    if (t.setpoint_kind == FlightController::Tick::SP::VELOCITY) {
      msg.position = {NAN, NAN, NAN};
      msg.velocity = {t.velocity.vx, t.velocity.vy, t.velocity.vz};
      msg.yawspeed = t.velocity.yawspeed;
    } else {
      msg.position = {t.position.x, t.position.y, t.position.z};
      msg.yaw      = t.yaw;
    }
    msg.timestamp = now_us();
    traj_pub_->publish(msg);
  }

  void send_vehicle_command(uint32_t cmd, float p1 = 0.0f, float p2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.command          = cmd;
    msg.param1           = p1;
    msg.param2           = p2;
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = now_us();
    cmd_pub_->publish(msg);
  }

  void send_vtol_transition(FlightController::FlightMode target)
  {
    const float state = (target == FlightController::FlightMode::CRUISE)
                          ? 4.0f    // VEHICLE_VTOL_STATE_FW
                          : 3.0f;   // VEHICLE_VTOL_STATE_MC
    send_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION,
      state, /*param2 immediate=*/0.0f);
  }
  void send_arm()
  {
    send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(get_logger(), "Arm command sent");
  }

  void send_offboard_mode()
  {
    // param1=1 (custom mode), param2=6 (offboard sub-mode)
    send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    RCLCPP_INFO(get_logger(), "Offboard mode command sent");
  }

  void send_land()
  {
    send_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(get_logger(), "Land command sent");
  }
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardMaster>());
  rclcpp::shutdown();
  return 0;
}
