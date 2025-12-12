import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)

# QoS Profile for PX4 communication (Best Effort, Transient Local)
QOS_RCL = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OffboardControl(Node):
    def __init__(self, drone_id: int):
        super().__init__(f"offboard_control_{drone_id}")
        self.drone_id = drone_id
        self.target_system_id = self.drone_id + 1
        if drone_id == 0:
            # Drone 0 uses the default /fmu/ namespace
            namespace = "/fmu"
        else:
            # Drone 1 uses the /px4_1/fmu/ namespace
            namespace = f"/px4_{drone_id}/fmu"

        self.get_logger().info(
            f"Pilot for Drone {drone_id} initialized. Namespace: {namespace}. Target System ID: {self.target_system_id}"
        )

        # Publishers to PX4
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, f"{namespace}/in/offboard_control_mode", QOS_RCL
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f"{namespace}/in/trajectory_setpoint", QOS_RCL
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, f"{namespace}/in/vehicle_command", QOS_RCL
        )

        # Subscribers from PX4
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            f"{namespace}/out/vehicle_status_v1",
            self.vehicle_status_callback,
            QOS_RCL,
        )

        # Subscribers from the Collision Manager
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            f"/drone_{drone_id}_setpoint",
            self.setpoint_callback,
            10,
        )

        # State Variables
        self.current_setpoint = TrajectorySetpoint()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0

        # Timers: 10 Hz control loop
        self.offboard_timer = self.create_timer(0.1, self.offboard_timer_callback)

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def setpoint_callback(self, msg):
        self.current_setpoint = msg

    def publish_vehicle_command(
        self, command: int, param1: float = 0.0, param2: float = 0.0
    ):
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command

        msg.target_system = self.target_system_id

        msg.target_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("Arm Command Sent")

    def set_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("Offboard Mode Request Sent")

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)

    def offboard_timer_callback(self):
        self.publish_offboard_mode()  # 1. Send Heartbeat

        if self.offboard_setpoint_counter < 10:
            # 2. Send initial setpoints (required before mode switch)
            self.trajectory_setpoint_pub.publish(self.current_setpoint)
            self.offboard_setpoint_counter += 1
            return

        # 3. Arming Logic
        if self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm()

        # 4. Mode Switching Logic
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.set_offboard_mode()

        # 5. Execute Command
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.trajectory_setpoint_pub.publish(self.current_setpoint)


def main_0(args=None):
    rclpy.init(args=args)
    pilot = OffboardControl(drone_id=0)
    rclpy.spin(pilot)
    pilot.destroy_node()
    rclpy.shutdown()


def main_1(args=None):
    rclpy.init(args=args)
    pilot = OffboardControl(drone_id=1)
    rclpy.spin(pilot)
    pilot.destroy_node()
    rclpy.shutdown()
