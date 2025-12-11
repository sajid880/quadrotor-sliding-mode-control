import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import time

class OffboardNode(Node):

    def __init__(self):
        super().__init__('offboard_node')

        # Correct QoS for MAVROS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher → PX4 setpoints
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            qos
        )

        # Services
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Timer for periodic publishing
        self.timer = self.create_timer(0.05, self.publish_setpoint)  # 20 Hz

        self.get_logger().info("Offboard node started.")

        # For circular trajectory
        self.start_time = time.time()

        # Wait for service availability
        self.wait_for_services()

        # Start offboard mode
        self.arm_and_set_mode()

    def wait_for_services(self):
        self.get_logger().info("Waiting for MAVROS services...")
        self.arming_client.wait_for_service()
        self.mode_client.wait_for_service()
        self.get_logger().info("MAVROS services ready.")

    def arm_and_set_mode(self):
        """Arms the drone and puts it into OFFBOARD mode."""
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        self.mode_client.call_async(req)
        self.get_logger().info("Requested OFFBOARD mode")

        arm_req = CommandBool.Request()
        arm_req.value = True
        self.arming_client.call_async(arm_req)
        self.get_logger().info("Requested ARM")

    def publish_setpoint(self):
        """Publishes continuous 3D trajectory (circular path)."""

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        t = time.time() - self.start_time

        # Circle: radius=2 m, height=2 m
        msg.pose.position.x = 2.0 * math.cos(t)
        msg.pose.position.y = 2.0 * math.sin(t)
        msg.pose.position.z = 2.0

        self.setpoint_pub.publish(msg)

        # Uncomment to debug
        # self.get_logger().info(f"Setpoint: {msg.pose.position}")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
