
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import Trigger
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff

from .variant6_state import Variant6State


class Variant6Node(Node):

    GUIDED_MODE = 4
    LAND_MODE = 9

    TAKEOFF_ALTITUDE = 2.0
    DELTA_X = 8.2
    DELTA_Y = 0.2

    POSITION_TOLERANCE = 0.25
    MAX_SPEED = 0.6
    KP = 0.5

    def __init__(self):
        super().__init__("flight_variant6")

        self.state = Variant6State.WAITING_FOR_SYSTEM

        self.status = None
        self.pose = None
        self.last_pose_time = None

        self.velocity = None
        self.last_velocity_time = None

        self.start_x = None
        self.start_y = None
        self.start_z = None
        self.target_x = None
        self.target_y = None

        self.pending = None
        self.pending_action = None
        self.takeoff_requested = False
        self.state_started = time.monotonic()
        self.last_log = 0.0

        self.create_subscription(
            Status,
            "/ap/status",
            self.status_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/ap/pose/filtered",
            self.pose_callback,
            qos_profile_sensor_data,
        )

        self.velocity_subscription = self.create_subscription(
            TwistStamped,
            "/ap/twist/filtered",
            self.velocity_callback,
            qos_profile_sensor_data,
        )

        self.velocity_pub = self.create_publisher(
            TwistStamped,
            "/ap/cmd_vel",
            qos_profile_sensor_data,
        )

        self.prearm_client = self.create_client(
            Trigger, "/ap/prearm_check"
        )
        self.mode_client = self.create_client(
            ModeSwitch, "/ap/mode_switch"
        )
        self.arm_client = self.create_client(
            ArmMotors, "/ap/arm_motors"
        )
        self.takeoff_client = self.create_client(
            Takeoff, "/ap/experimental/takeoff"
        )

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Variant 6 node started")

    def status_callback(self, msg):
        self.status = msg

    def pose_callback(self, msg):
        self.pose = msg
        self.last_pose_time = time.monotonic()

    def velocity_callback(self, msg):
        self.velocity = msg
        self.last_velocity_time = time.monotonic()

    def change_state(self, state):
        self.state = state
        self.state_started = time.monotonic()

    def services_ready(self):
        return all([
            self.prearm_client.service_is_ready(),
            self.mode_client.service_is_ready(),
            self.arm_client.service_is_ready(),
            self.takeoff_client.service_is_ready(),
        ])

    def send_request(self, client, request, action):
        self.pending = client.call_async(request)
        self.pending_action = action
        self.get_logger().info(action)

    def publish_velocity(self, vx, vy):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = 0.0

        msg.twist.angular.z = 0.0
        self.velocity_pub.publish(msg)

    def request_land(self):
        request = ModeSwitch.Request()
        request.mode = self.LAND_MODE
        self.send_request(
            self.mode_client,
            request,
            "Requesting LAND mode",
        )
        self.change_state(Variant6State.LANDING)

    def fail(self, message):
        self.get_logger().error(message)

        if self.state in (
            Variant6State.LANDING,
            Variant6State.WAITING_FOR_LANDING,
        ):
            self.change_state(Variant6State.ERROR)
            return

        if self.status is not None and self.status.armed:
            self.publish_velocity(0.0, 0.0)

            if self.mode_client.service_is_ready():
                self.request_land()
            else:
                self.get_logger().error(
                    "LAND service unavailable"
                )
                self.change_state(Variant6State.ERROR)
        else:
            self.change_state(Variant6State.ERROR)

    def process_response(self):
        if self.pending is None or not self.pending.done():
            return

        action = self.pending_action

        try:
            response = self.pending.result()
        except Exception as error:
            self.pending = None
            self.fail(f"{action} failed: {error}")
            return

        self.pending = None
        self.pending_action = None

        if self.state == Variant6State.PREARM_CHECK:
            if not response.success:
                self.fail(f"Pre-arm failed: {response.message}")
                return

            self.get_logger().info("Pre-arm check passed")
            self.change_state(Variant6State.SETTING_GUIDED)

        elif self.state == Variant6State.SETTING_GUIDED:
            if not response.status or response.curr_mode != self.GUIDED_MODE:
                self.fail("GUIDED mode rejected")
                return

            self.get_logger().info("GUIDED mode enabled")
            self.change_state(Variant6State.ARMING)

        elif self.state == Variant6State.ARMING:
            if not response.result:
                self.fail("Arming rejected")
                return

            self.get_logger().info("Motors armed")
            self.change_state(Variant6State.TAKING_OFF)

        elif self.state == Variant6State.TAKING_OFF:
            if not response.status:
                self.fail("Takeoff rejected")
                return

            self.get_logger().info("Takeoff accepted")

        elif self.state == Variant6State.LANDING:
            if not response.status or response.curr_mode != self.LAND_MODE:
                self.get_logger().error("LAND mode was not confirmed")
                self.change_state(Variant6State.ERROR)
                return

            self.get_logger().info("LAND mode enabled")
            self.change_state(Variant6State.WAITING_FOR_LANDING)

    def control_loop(self):
        now = time.monotonic()

        if self.state in (
            Variant6State.FINISHED,
            Variant6State.ERROR,
        ):
            return

        if self.pending is not None:
            self.process_response()
            return

        if self.state == Variant6State.WAITING_FOR_SYSTEM:
            if self.status is None or self.pose is None:
                return

            if self.last_pose_time is None or now - self.last_pose_time > 3.0:
                self.get_logger().warning("Waiting for fresh position data")
                return

            if not self.services_ready():
                return

            if self.status.armed or self.status.flying:
                self.change_state(Variant6State.ERROR)
                self.get_logger().error(
                    "Drone must be disarmed before starting"
                )
                return

            self.start_x = self.pose.pose.position.x
            self.start_y = self.pose.pose.position.y
            self.start_z = self.pose.pose.position.z

            self.target_x = self.start_x - self.DELTA_X
            self.target_y = self.start_y - self.DELTA_Y

            self.get_logger().info(
                f"START: x={self.start_x:.2f}, "
                f"y={self.start_y:.2f}"
            )
            self.get_logger().info(
                f"TARGET: x={self.target_x:.2f}, "
                f"y={self.target_y:.2f}"
            )

            self.change_state(Variant6State.PREARM_CHECK)

        elif self.state == Variant6State.PREARM_CHECK:
            self.send_request(
                self.prearm_client,
                Trigger.Request(),
                "Running pre-arm check",
            )

        elif self.state == Variant6State.SETTING_GUIDED:
            request = ModeSwitch.Request()
            request.mode = self.GUIDED_MODE

            self.send_request(
                self.mode_client,
                request,
                "Requesting GUIDED mode",
            )

        elif self.state == Variant6State.ARMING:
            request = ArmMotors.Request()
            request.arm = True

            self.send_request(
                self.arm_client,
                request,
                "Requesting ARM",
            )

        elif self.state == Variant6State.TAKING_OFF:
            if self.status is None or not self.status.armed:
                if now - self.state_started > 10.0:
                    self.fail("Arming confirmation timeout")
                return

            if (
                self.last_pose_time is None
                or now - self.last_pose_time > 3.0
            ):
                self.fail("Position data lost during takeoff")
                return

            if self.pose is not None:
                altitude = (
                    self.pose.pose.position.z - self.start_z
                )

                stable = (
                    1.8 <= altitude <= 2.2
                    and self.velocity is not None
                    and self.last_velocity_time is not None
                    and now - self.last_velocity_time < 1.0
                    and abs(self.velocity.twist.linear.z) <= 0.15
                )

                if stable:
                    if getattr(self, "hover_started", None) is None:
                        self.hover_started = now

                    if now - self.hover_started >= 2.0:
                        self.get_logger().info(
                            f"Takeoff stabilized: {altitude:.2f} m"
                        )
                        self.change_state(
                            Variant6State.MOVING_TO_TARGET
                        )
                        return
                else:
                    self.hover_started = None

            if now - self.state_started > 45.0:
                self.fail("Takeoff timeout")
                return

            if not self.takeoff_requested:
                self.takeoff_requested = True

                request = Takeoff.Request()
                request.alt = float(self.TAKEOFF_ALTITUDE)

                self.send_request(
                    self.takeoff_client,
                    request,
                    "Requesting takeoff to 2 m",
                )

        elif self.state == Variant6State.MOVING_TO_TARGET:
            if self.status is None or not self.status.armed:
                self.fail("Drone disarmed during movement")
                return

            if (
                self.pose is None
                or self.last_pose_time is None
                or now - self.last_pose_time > 3.0
            ):
                self.fail("Position data is unavailable")
                return

            if now - self.state_started > 90.0:
                self.fail("Movement timeout")
                return

            x = self.pose.pose.position.x
            y = self.pose.pose.position.y

            error_x = self.target_x - x
            error_y = self.target_y - y

            distance = math.hypot(error_x, error_y)

            if (
                self.velocity is None
                or self.last_velocity_time is None
                or now - self.last_velocity_time > 3.0
            ):
                self.fail("Velocity data is unavailable")
                return

            speed = math.hypot(
                self.velocity.twist.linear.x,
                self.velocity.twist.linear.y,
            )

            if distance <= self.POSITION_TOLERANCE and speed <= 0.15:
                self.publish_velocity(0.0, 0.0)

                self.get_logger().info(
                    f"TARGET REACHED: x={x:.2f}, "
                    f"y={y:.2f}, error={distance:.3f} m"
                )
                self.request_land()
                return

            vx = max(
                -self.MAX_SPEED,
                min(self.MAX_SPEED, self.KP * error_x),
            )
            vy = max(
                -self.MAX_SPEED,
                min(self.MAX_SPEED, self.KP * error_y),
            )

            self.publish_velocity(vx, vy)

            if now - self.last_log >= 2.0:
                self.get_logger().info(
                    f"POSITION: x={x:.2f}, y={y:.2f}, "
                    f"distance={distance:.2f} m"
                )
                self.last_log = now

        elif self.state == Variant6State.WAITING_FOR_LANDING:
            if self.status is not None and not self.status.flying:
                if self.pose is not None:
                    x = self.pose.pose.position.x
                    y = self.pose.pose.position.y
                    error = math.hypot(
                        self.target_x - x,
                        self.target_y - y,
                    )

                    self.get_logger().info(
                        f"FINAL: x={x:.2f}, y={y:.2f}"
                    )
                    self.get_logger().info(
                        f"FINAL POSITION ERROR: {error:.3f} m"
                    )

                self.get_logger().info(
                    "Variant 6 completed successfully"
                )
                self.change_state(Variant6State.FINISHED)

            elif now - self.state_started > 90.0:
                self.get_logger().error(
                    "Landing not confirmed"
                )
                self.change_state(Variant6State.ERROR)


def main(args=None):
    rclpy.init(args=args)
    node = Variant6Node()

    try:
        while rclpy.ok() and node.state not in (
            Variant6State.FINISHED,
            Variant6State.ERROR,
        ):
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().warning(
            "Interrupted. Verify drone state in simulator."
        )

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
