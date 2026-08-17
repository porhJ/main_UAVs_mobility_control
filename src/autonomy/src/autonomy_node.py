#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool

from custom_interfaces.msg import OffboardStatus


class AutonomyShell(Node):
    """
    Thin ROS2 shell between Python autonomy logic and the existing
    C++ offboard_master.

    Python side:
        WorldModel
        DecisionAgent
        MissionExecutor

    ROS boundary:
        /offboard/setpoint/position
        /offboard/setpoint/velocity
        /offboard/cmd/land
        /offboard/state

    C++ side:
        offboard_master
        FlightController
        PX4
    """

    def __init__(self):
        super().__init__("autonomy_shell")

        # ---------------------------------------------------------
        # Publishers -> C++ offboard_master
        # ---------------------------------------------------------

        self.position_pub = self.create_publisher(
            PoseStamped,
            "/offboard/setpoint/position",
            10,
        )

        self.velocity_pub = self.create_publisher(
            TwistStamped,
            "/offboard/setpoint/velocity",
            10,
        )

        self.land_pub = self.create_publisher(
            Bool,
            "/offboard/cmd/land",
            10,
        )

        # ---------------------------------------------------------
        # Subscriber <- C++ offboard_master
        # ---------------------------------------------------------

        self.offboard_state_sub = self.create_subscription(
            OffboardStatus,
            "/offboard/state",
            self._on_offboard_state,
            10,
        )

        # ---------------------------------------------------------
        # Cached vehicle/offboard state
        # ---------------------------------------------------------

        self.accepting_setpoints = False
        self.offboard_state = None

        # ---------------------------------------------------------
        # Main autonomy tick
        # ---------------------------------------------------------

        self.timer_period = 0.1  # 10 Hz

        self.timer = self.create_timer(
            self.timer_period,
            self._tick,
        )

        self.get_logger().info("Autonomy shell started")

    # =============================================================
    # ROS callbacks
    # =============================================================

    def _on_offboard_state(self, msg: OffboardStatus):
        """
        Cache state reported by offboard_master.
        """

        self.offboard_state = msg
        self.accepting_setpoints = msg.accepting_setpoints

    # =============================================================
    # Main loop
    # =============================================================

    def _tick(self):
        """
        This is where the Python autonomy stack will eventually run.

        Example future flow:

            world = self.world_model.state()
            action = self.agent.tick(world)
            objective = self.executor.tick(action, world)

            self.execute_objective(objective)
        """

        if self.offboard_state is None:
            return

        if not self.accepting_setpoints:
            return

        # ---------------------------------------------
        # TEMPORARY TEST
        #
        # Hold / command position:
        #
        # x = 5 m North
        # y = 0 m East
        # z = -3 m = 3 m altitude
        #
        # Your basic_offboard convention is NED.
        # ---------------------------------------------

        self.send_position(
            x=5.0,
            y=0.0,
            z=-3.0,
        )

    # =============================================================
    # Output API
    # =============================================================

    def send_position(
        self,
        x: float,
        y: float,
        z: float,
    ):
        """
        Send NED position objective to offboard_master.

        x = North
        y = East
        z = Down

        Example:
            z = -3.0 -> 3 m above origin
        """

        if not self.accepting_setpoints:
            return

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        # This is a logical frame label only.
        # The actual convention used by this project is NED.
        msg.header.frame_id = "ned"

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        # Neutral orientation.
        msg.pose.orientation.w = 1.0

        self.position_pub.publish(msg)

    def send_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
    ):
        """
        Send NED velocity objective.

        vx = North velocity
        vy = East velocity
        vz = Down velocity
        """

        if not self.accepting_setpoints:
            return

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ned"

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz

        msg.twist.angular.z = yaw_rate

        self.velocity_pub.publish(msg)

    def request_land(self):
        """
        Request landing through offboard_master.

        Python never sends VehicleCommand directly to PX4.
        """

        msg = Bool()
        msg.data = True

        self.land_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = AutonomyShell()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()