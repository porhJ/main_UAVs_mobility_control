#include <cmath>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include "custom_interfaces/srv/req_mode.hpp"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

// All position/velocity setpoints are expected in NED frame (North-East-Down).
// External nodes should publish coordinates in NED.
//
// VTOL mode values for ext_mode_sub_ (std_msgs::msg::UInt8):
//   0 = request MC (multicopter) mode
//   1 = request FW (fixed-wing) mode

class OffboardMaster : public rclcpp::Node
{
public:
  enum class State {
    TAKEOFF,    
    MC_HOVER,  
    FW_HOVER,   
    FW_CRUISE,  
    LANDING,    
    LANDED     
  };

  OffboardMaster() : Node("offboard_master")
  {
    rclcpp::QoS px4_qos(1);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

    // Publishers
    offboard_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", px4_qos);
    cmd_pub_      = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", px4_qos);
    traj_pub_     = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", px4_qos);

    declare_parameter("min_transition_airspeed_m_s", 10.0);

    // PX4 state subscriptions
    vehicle_status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(),
      std::bind(&OffboardMaster::vehicle_status_callback, this, std::placeholders::_1));

    vehicle_local_position_sub_ = this->create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      std::bind(&OffboardMaster::vehicle_odom_callback, this, std::placeholders::_1)
    );

    airspeed_validated_sub_ = create_subscription<AirspeedValidated>(
      "/fmu/out/airspeed_validated", rclcpp::SensorDataQoS(),
      std::bind(&OffboardMaster::airspeed_callback, this, std::placeholders::_1));

    // External command subscriptions
    ext_land_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/offboard/cmd/land", 10,
      std::bind(&OffboardMaster::ext_land_callback, this, std::placeholders::_1));

    ext_pos_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/offboard/setpoint/position", 10,
      std::bind(&OffboardMaster::ext_pos_callback, this, std::placeholders::_1));

    ext_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/offboard/setpoint/velocity", 10,
      std::bind(&OffboardMaster::ext_vel_callback, this, std::placeholders::_1));

    mode_srv_ = create_service<custom_interfaces::srv::ReqMode>(
      "/offboard/srv/req_mode",
      std::bind(&OffboardMaster::handle_req_mode, this,
                std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(100ms, std::bind(&OffboardMaster::timer_callback, this));

    RCLCPP_INFO(get_logger(), "OffboardMaster initialized. Publishing heartbeats...");
  }

private:
  // Publishers (to PX4)
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr      cmd_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_pub_;

  // PX4 state subscriptions
  rclcpp::Subscription<VehicleStatus>::SharedPtr           vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr    vehicle_local_position_sub_;
  rclcpp::Subscription<AirspeedValidated>::SharedPtr       airspeed_validated_sub_;

  // External command subscriptions
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                ext_land_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr    ext_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr   ext_vel_sub_;
  rclcpp::Service<custom_interfaces::srv::ReqMode>::SharedPtr         mode_srv_;

  rclcpp::TimerBase::SharedPtr timer_;

  // State
  State state_{State::LANDED};
  VehicleStatus vehicle_status_{};
  float vehicle_local_position_[3];
  AirspeedValidated airspeed_validated_{};

  // Latest position setpoint (NED frame)
  struct PositionSetpoint {
    float x{0.0f}, y{0.0f}, z{0.0f}, yaw{0.0f};
    bool valid{false};
  } pos_sp_;

  // Latest velocity setpoint (NED frame) with timeout
  struct VelocitySetpoint {
    float vx{0.0f}, vy{0.0f}, vz{0.0f}, yawspeed{0.0f};
    bool valid{false};
    rclcpp::Time last_received{0, 0, RCL_ROS_TIME};
  } vel_sp_;

  // Hover hold position (NED). Updated as drone moves.
  float hover_x_{0.0f};
  float hover_y_{0.0f};
  float hover_z_{TAKEOFF_HEIGHT_NED};
  float hover_yaw_{0.0f};

  static constexpr float TAKEOFF_HEIGHT_NED  = -5.0f; 
  static constexpr float TAKEOFF_THRESHOLD   =  0.3f;  // metres tolerance
  static constexpr double VEL_SP_TIMEOUT_S   =  0.5;   // velocity setpoint expiry
  static constexpr int   OFFBOARD_SETTLE_COUNT = 10;   // heartbeats before arming

  int offboard_counter_{0};

  // ----- Main timer callback -----

  void timer_callback()
  {
    update_state();

    const bool use_velocity = is_vel_sp_active();
    publish_offboard_heartbeat(use_velocity);

    // Phase 1: accumulate heartbeats before switching to offboard mode
    if (offboard_counter_ < OFFBOARD_SETTLE_COUNT) {
      publish_hover_setpoint();
      offboard_counter_++;
      return;
    }

    // Phase 2: exactly at the settle count — arm and switch mode once
    if (offboard_counter_ == OFFBOARD_SETTLE_COUNT) {
      engage_offboard_mode();
      arm();
      set_state(State::TAKEOFF);
      offboard_counter_++;
    }

    // Phase 3: run state machine
    switch (state_) {
      case State::TAKEOFF:
        publish_hover_setpoint();
        break;

      case State::MC_HOVER:
      case State::FW_HOVER:
      case State::FW_CRUISE:
        publish_active_setpoint();
        break;

      case State::LANDING:
      case State::LANDED:
        // PX4 handles landing autonomously; just keep sending heartbeat
        break;
    }
  }

  // ----- State transitions -----

  void update_state()
  {
    switch (state_) {
      case State::TAKEOFF:
        if (vehicle_local_position_[2] <= (TAKEOFF_HEIGHT_NED + TAKEOFF_THRESHOLD)) {
          hover_x_   = vehicle_local_position_[0];
          hover_y_   = vehicle_local_position_[1];
          hover_z_   = vehicle_local_position_[2];
          set_state(State::MC_HOVER);
        }
        break;

      case State::FW_HOVER:
        // Transition complete → FW_CRUISE
        if (!vehicle_status_.in_transition_mode &&
            vehicle_status_.vehicle_type == VehicleStatus::VEHICLE_TYPE_FIXED_WING) {
          set_state(State::FW_CRUISE);
        }
        // Back-transition complete → MC_HOVER
        if (!vehicle_status_.in_transition_mode &&
            vehicle_status_.vehicle_type == VehicleStatus::VEHICLE_TYPE_ROTARY_WING) {
          set_state(State::MC_HOVER);
        }
        break;

      case State::LANDING:
        if (vehicle_status_.arming_state == VehicleStatus::ARMING_STATE_DISARMED) {
          set_state(State::LANDED);
        }
        break;

      default:
        break;
    }
  }

  // ----- Publishing helpers -----

  void publish_offboard_heartbeat(bool use_velocity)
  {
    OffboardControlMode msg{};
    msg.position     = !use_velocity;
    msg.velocity     = use_velocity;
    msg.acceleration = false;
    msg.attitude     = false;
    msg.body_rate    = false;
    msg.timestamp    = now().nanoseconds() / 1000;
    offboard_pub_->publish(msg);
  }

  void publish_hover_setpoint()
  {
    TrajectorySetpoint msg{};
    msg.position  = {hover_x_, hover_y_, hover_z_};
    msg.yaw       = hover_yaw_;
    msg.timestamp = now().nanoseconds() / 1000;
    traj_pub_->publish(msg);
  }

  void publish_active_setpoint()
  {
    if (is_vel_sp_active()) {
      TrajectorySetpoint msg{};
      msg.position  = {NAN, NAN, NAN};
      msg.velocity  = {vel_sp_.vx, vel_sp_.vy, vel_sp_.vz};
      msg.yawspeed  = vel_sp_.yawspeed;
      msg.timestamp = now().nanoseconds() / 1000;
      traj_pub_->publish(msg);
      // Track position so hover hold is current when velocity expires
      hover_x_   = vehicle_local_position_[0];
      hover_y_   = vehicle_local_position_[1];
      hover_z_   = vehicle_local_position_[2];
    } else if (pos_sp_.valid) {
      TrajectorySetpoint msg{};
      msg.position  = {pos_sp_.x, pos_sp_.y, pos_sp_.z};
      msg.yaw       = pos_sp_.yaw;
      msg.timestamp = now().nanoseconds() / 1000;
      traj_pub_->publish(msg);
      // Mirror to hover hold so expiry is seamless
      hover_x_   = pos_sp_.x;
      hover_y_   = pos_sp_.y;
      hover_z_   = pos_sp_.z;
      hover_yaw_ = pos_sp_.yaw;
    } else {
      // hold last known position
      publish_hover_setpoint();
    }
  }

  // ----- Vehicle command helpers -----

  void publish_vehicle_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand msg{};
    msg.command          = command;
    msg.param1           = param1;
    msg.param2           = param2;
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = now().nanoseconds() / 1000;
    cmd_pub_->publish(msg);
  }

  void arm()
  {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(get_logger(), "Arm command sent");
  }

  void engage_offboard_mode()
  {
    // param1=1 (custom mode), param2=6 (offboard sub-mode)
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    RCLCPP_INFO(get_logger(), "Offboard mode command sent");
  }

  void initiate_landing()
  {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(get_logger(), "Land command sent");
  }

  void request_vtol_transition(bool to_fw)
  {
    // MAV_VTOL_STATE_MC = 3, MAV_VTOL_STATE_FW = 4
    const float target_state = to_fw ? 4.0f : 3.0f;
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, target_state);
    RCLCPP_INFO(get_logger(), "VTOL transition requested: %s", to_fw ? "MC->FW" : "FW->MC");
  }

  // ----- Predicate helpers -----

  bool takeoff_reached() const
  {
    return vehicle_local_position_[2] <= (TAKEOFF_HEIGHT_NED + TAKEOFF_THRESHOLD);
  }

  bool is_vel_sp_active() const
  {
    if (!vel_sp_.valid) return false;
    return (now() - vel_sp_.last_received).seconds() < VEL_SP_TIMEOUT_S;
  }

  bool is_airspeed_sufficient() const
  {
    const float cas = airspeed_validated_.calibrated_airspeed_m_s;
    if (std::isnan(cas)) return false;
    const double threshold = get_parameter("min_transition_airspeed_m_s").as_double();
    return cas >= static_cast<float>(threshold);
  }

  // ----- Logging helpers -----

  void set_state(State new_state)
  {
    if (state_ == new_state) return;
    RCLCPP_INFO(get_logger(), "State: %s -> %s", state_to_str(state_), state_to_str(new_state));
    state_ = new_state;
  }

  static const char * state_to_str(State s)
  {
    switch (s) {
      case State::TAKEOFF:   return "TAKEOFF";
      case State::MC_HOVER:  return "MC_HOVER";
      case State::FW_HOVER:  return "FW_HOVER";
      case State::FW_CRUISE: return "FW_CRUISE";
      case State::LANDING:   return "LANDING";
      case State::LANDED:    return "LANDED";
      default:               return "UNKNOWN";
    }
  }

  // ----- External command callbacks ----

  void vehicle_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // PX4 NED coordinates: z is negative when above ground.
    vehicle_local_position_[0] = msg->position[0];
    vehicle_local_position_[1] = msg->position[1];
    vehicle_local_position_[2] = msg->position[2];
  }

  void vehicle_status_callback(const VehicleStatus::SharedPtr msg)
  {
    vehicle_status_ = *msg;
  }

  void airspeed_callback(const AirspeedValidated::SharedPtr msg)
  {
    airspeed_validated_ = *msg;
  }

  void ext_land_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) return;

    switch (state_) {
      case State::TAKEOFF:
      case State::MC_HOVER:
      case State::FW_HOVER:
      case State::FW_CRUISE:
        initiate_landing();
        set_state(State::LANDING);
        break;
      default:
        RCLCPP_WARN(get_logger(), "Land command rejected in state %s", state_to_str(state_));
        break;
    }
  }

  void ext_pos_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (state_ != State::MC_HOVER &&
        state_ != State::FW_HOVER &&
        state_ != State::FW_CRUISE)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Position setpoint rejected in state %s", state_to_str(state_));
      return;
    }

    // Positions expected in NED frame
    pos_sp_.x = static_cast<float>(msg->pose.position.x);
    pos_sp_.y = static_cast<float>(msg->pose.position.y);
    pos_sp_.z = static_cast<float>(msg->pose.position.z);

    // Extract yaw from quaternion (assumes NED quaternion convention)
    const auto & q = msg->pose.orientation;
    pos_sp_.yaw = static_cast<float>(
      std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z)));

    pos_sp_.valid = true;
  }

  void ext_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    if (state_ != State::MC_HOVER &&
        state_ != State::FW_HOVER &&
        state_ != State::FW_CRUISE)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Velocity setpoint rejected in state %s", state_to_str(state_));
      return;
    }

    // Velocities expected in NED frame
    vel_sp_.vx       = static_cast<float>(msg->twist.linear.x);
    vel_sp_.vy       = static_cast<float>(msg->twist.linear.y);
    vel_sp_.vz       = static_cast<float>(msg->twist.linear.z);
    vel_sp_.yawspeed = static_cast<float>(msg->twist.angular.z);
    vel_sp_.valid    = true;
    vel_sp_.last_received = now();
  }

  void handle_req_mode(
    const custom_interfaces::srv::ReqMode::Request::SharedPtr  req,
    custom_interfaces::srv::ReqMode::Response::SharedPtr       res)
  {
    using Req = custom_interfaces::srv::ReqMode::Request;
    using Res = custom_interfaces::srv::ReqMode::Response;

    if (!vehicle_status_.is_vtol) {
      res->result  = Res::RESULT_REJECTED;
      return;
    }

    if (req->mode == Req::MODE_FW) {
      if (state_ == State::FW_HOVER || state_ == State::FW_CRUISE) {
        res->result = Res::RESULT_OK;
      } else if (state_ == State::MC_HOVER && is_airspeed_sufficient()) {
        request_vtol_transition(true);
        set_state(State::FW_HOVER);
        res->result = Res::RESULT_OK;
      } else {
        if (state_ == State::MC_HOVER) {
          RCLCPP_WARN(get_logger(),
            "FW transition rejected: CAS %.1f m/s below minimum %.1f m/s",
            static_cast<double>(airspeed_validated_.calibrated_airspeed_m_s),
            get_parameter("min_transition_airspeed_m_s").as_double());
        } else {
          RCLCPP_WARN(get_logger(), "FW transition not allowed in state %s", state_to_str(state_));
        }
        res->result = Res::RESULT_REJECTED;
      }
      return;
    }

    if (req->mode == Req::MODE_MC) {
      if (state_ == State::MC_HOVER) {
        res->result = Res::RESULT_OK;
      } else if (state_ == State::FW_CRUISE) {
        request_vtol_transition(false);
        set_state(State::FW_HOVER);
        res->result = Res::RESULT_OK;
      } else {
        RCLCPP_WARN(get_logger(), "MC transition not allowed in state %s", state_to_str(state_));
        res->result = Res::RESULT_REJECTED;
      }
      return;
    }

    res->result = Res::RESULT_REJECTED;
    RCLCPP_WARN(get_logger(), "Unknown mode value: %u", req->mode);
  }
};

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardMaster>());
  rclcpp::shutdown();
  return 0;
}
