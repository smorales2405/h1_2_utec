#!/usr/bin/env python3
"""ROS 2 node: software coupling of the RH56DFTP hands, for a URDF built with coupling:=software.

That URDF has no <mimic>, so robot_state_publisher needs all 12 joints of each hand. This node
computes them with the curves measured on the real hand (config/hand_coupling.yaml, through
hand_coupling.py) from either input:

  joint_states_input (sensor_msgs/JointState)
      A full robot state whose hands carry only the 6 actuated joints (what
      joint_state_publisher_gui gives with the coupled joints declared as dependent_joints, see
      launch/). Every other joint passes through unchanged; for each hand that carries its 6
      actuated joints, the 6 coupled ones are computed (any value they had is replaced) and
      the actuated ones are kept on their measured curve (clipped to its range).
  hand_angle_set/left, hand_angle_set/right (std_msgs/Int32MultiArray)
      The 6 ANGLE_SET or ANGLE_ACT values of one hand, SDK order: little, ring, middle, index,
      thumb_bend, thumb_rot. Publishes that hand's 12 joints. For mirroring the real hands.

Output: joint_states (sensor_msgs/JointState), positions only.

Parameters: input_topic, output_topic, angle_set_topic (prefix; /left and /right are appended),
config (curves file; empty = the package's config/hand_coupling.yaml).
"""
from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
from hand_coupling import SIDES, HandCoupling, joint_name  # noqa: E402


class HandCouplingNode(Node):
    def __init__(self):
        super().__init__("hand_coupling")
        input_topic = self.declare_parameter("input_topic", "joint_states_input").value
        output_topic = self.declare_parameter("output_topic", "joint_states").value
        angle_topic = self.declare_parameter("angle_set_topic", "hand_angle_set").value
        config = self.declare_parameter("config", "").value
        self.hc = HandCoupling(config or None)
        self.actuated = {side: [joint_name(side, j) for j in self.hc.actuated] for side in SIDES}
        self.warned = set()
        self.pub = self.create_publisher(JointState, output_topic, 10)
        self.create_subscription(JointState, input_topic, self.on_joint_states, 10)
        for side in SIDES:
            self.create_subscription(Int32MultiArray, f"{angle_topic}/{side}",
                                     lambda msg, side=side: self.on_angle_set(msg, side), 10)
        self.get_logger().info(f"curves: {self.hc.config}; {input_topic} and {angle_topic}/{{left,right}} -> {output_topic}")

    def on_joint_states(self, msg: JointState) -> None:
        q = dict(zip(msg.name, msg.position))
        for side in SIDES:
            present = [n for n in self.actuated[side] if n in q]
            if len(present) == 6:
                q.update(self.hc.couple(q, side))
            elif present and side not in self.warned:
                self.warned.add(side)
                missing = sorted(set(self.actuated[side]) - set(present))
                self.get_logger().warning(f"{side} hand: actuated joints missing {missing}; passed through uncoupled")
        out = JointState()
        out.header = msg.header
        out.name = list(q)
        out.position = [float(v) for v in q.values()]
        self.pub.publish(out)

    def on_angle_set(self, msg: Int32MultiArray, side: str) -> None:
        if len(msg.data) != 6:
            self.get_logger().warning(f"{side}: expected 6 ANGLE_SET values, got {len(msg.data)}", throttle_duration_sec=5.0)
            return
        q = self.hc.joints_from_angle_set(list(msg.data), side)
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(q)
        out.position = list(q.values())
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandCouplingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
