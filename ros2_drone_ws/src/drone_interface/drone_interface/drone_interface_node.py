#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import ActuatorControl

class DroneInterface(Node):
    def __init__(self):
        super().__init__("drone_interface")
        self.sub = self.create_subscription(Float32MultiArray, "/controller/commands", self.cb, 10)
        self.pub = self.create_publisher(ActuatorControl, "/mavros/actuator_control", 10)

    def cb(self, msg):
        uF, tx, ty, tz = msg.data
        ac = ActuatorControl()
        ac.controls = [uF, tx, ty, tz, 0.0, 0.0, 0.0, 0.0]
        self.pub.publish(ac)

def main(args=None):
    rclpy.init(args=args)
    node = DroneInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
