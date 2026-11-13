"""Publish a bounded demo JointTrajectory. Simulation/demo only; not a hardware driver."""
import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryDemo(Node):
    def __init__(self):
        super().__init__("three_r_trajectory_demo")
        self.publisher = self.create_publisher(JointTrajectory, "/joint_trajectory", 10)
        self.timer = self.create_timer(0.05, self.publish_sample)
        self.t = 0.0

    def publish_sample(self):
        msg = JointTrajectory()
        msg.joint_names = ["joint1", "joint2", "joint3"]
        point = JointTrajectoryPoint()
        point.positions = [
            0.35 * math.sin(self.t),
            -0.55 + 0.25 * math.sin(self.t),
            0.25 * math.sin(self.t + 0.5),
        ]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100_000_000
        msg.points = [point]
        self.publisher.publish(msg)
        self.t += 0.05


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
