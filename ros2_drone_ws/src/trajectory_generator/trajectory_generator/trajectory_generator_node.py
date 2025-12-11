#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__("trajectory_generator")

        self.declare_parameter("hz", 30.0)
        self.declare_parameter("type", "helix")   # helix / circle / line
        self.declare_parameter("radius", 3.0)
        self.declare_parameter("height", 3.0)
        self.declare_parameter("speed", 0.5)

        self.hz = float(self.get_parameter("hz").value)
        self.dt = 1.0 / self.hz

        self.traj_type = self.get_parameter("type").value
        self.radius = float(self.get_parameter("radius").value)
        self.height = float(self.get_parameter("height").value)
        self.speed = float(self.get_parameter("speed").value)

        qos = QoSProfile(depth=10)

        self.pub = self.create_publisher(PoseStamped, "/trajectory/desired", qos)

        self.t = 0.0
        self.timer = self.create_timer(self.dt, self.update_traj)

        self.get_logger().info(f"Trajectory generator running: {self.traj_type}")

    def update_traj(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        # ---- Select trajectory type ----
        if self.traj_type == "circle":
            x = self.radius * math.cos(self.speed * self.t)
            y = self.radius * math.sin(self.speed * self.t)
            z = 2.0

        elif self.traj_type == "helix":
            x = self.radius * math.cos(self.speed * self.t)
            y = self.radius * math.sin(self.speed * self.t)
            z = 1.0 + (self.height * self.speed * self.t) / (2 * math.pi)

            # Limit max height
            if z > self.height:
                z = self.height

        elif self.traj_type == "line":
            x = 0.5 * self.t
            y = 0.5 * self.t
            z = 2.0

        else:
            x = y = z = 2.0

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        self.pub.publish(msg)

        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
