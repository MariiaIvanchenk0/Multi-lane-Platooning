#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
#
# virtual_safety_net: polytope_safety_node
#
# - Loads agent names from an external YAML file (agents.yaml)
# - Loads boundary points from an external YAML file (boundary.yaml)
#   and builds a convex hull and polytope Ax + b <= 0.
# - For each agent:
#     * subscribes to  [name]/pose        (geometry_msgs/PoseStamped)
#     * subscribes to  [name]/raw_cmd_vel (geometry_msgs/Twist, body frame)
#     * publishes to   [name]/cmd_vel     (geometry_msgs/Twist, safe)
#
# Safety logic:
#   1) Convert raw_cmd_vel (body frame) to global frame using pose orientation.
#   2) Compute distance to each polytope facet.
#   3) Find "active" facets with distance <= boundary_epsilon (or outside).
#   4) For each active facet i, compute outward normal component a_i^T v_global.
#   5) If ANY active facet has a_i^T v_global > 0  -> STOP (publish zero twist).
#      Else -> pass through raw_cmd_vel unchanged as cmd_vel.
#
# This guarantees that agents never move outward across/near the safety polytope
# boundary, but they are allowed to move inward even if currently outside.

import math
from typing import Dict, List

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, Twist


def load_agent_names(agents_file: str) -> List[str]:
    """
    Load agent names from the shared bringup agents YAML.

    Preferred format:
        agents:
          - vrpn_name: dji_robot_1
            name: agent_1
            serial_number: 3JKDH6C0012Z43

    The older format is also accepted:
        agents:
          - agent_1
          - agent_2
    """
    try:
        with open(agents_file, 'r') as f:
            agents_data = yaml.safe_load(f) or {}
    except Exception as e:
        raise RuntimeError(f"Failed to load agents_file '{agents_file}': {e}")

    agents = agents_data.get('agents')
    if not isinstance(agents, list):
        raise ValueError("agents_file must contain a key 'agents' with a list.")

    names = []
    for index, agent in enumerate(agents):
        if isinstance(agent, str):
            names.append(agent)
            continue

        if isinstance(agent, dict) and 'name' in agent:
            names.append(str(agent['name']))
            continue

        raise ValueError(
            "Each agent must either be a name string or a dict with a 'name' key. "
            f"Invalid entry at agents[{index}]: {agent}"
        )

    if not names:
        raise ValueError("agents_file does not contain any agents.")

    return names


def convex_hull_2d(points: np.ndarray) -> np.ndarray:
    """
    Compute 2D convex hull of an arbitrary set of points using the monotone
    chain algorithm. Input points can be in any order and need not form a
    convex polygon.

    Returns hull vertices in counter-clockwise order as a (M, 2) array.
    """
    assert points.shape[1] == 2, "Points must be 2D"
    pts = sorted(points.tolist())  # sort lexicographically by (x, y)

    if len(pts) <= 1:
        return np.array(pts)

    def cross(o, a, b):
        # z-component of (a - o) x (b - o)
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    # Lower hull
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    # Upper hull
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Concatenate lower and upper, removing the last point of each because it's
    # repeated at the beginning of the other list.
    hull = lower[:-1] + upper[:-1]
    return np.array(hull)


def polytope_from_hull(points: np.ndarray):
    """
    Given 2D convex polygon vertices in CCW order, construct (A, b, norms)
    such that the polytope is:
        { x | A x + b <= 0 }.

    Returns:
        A: (m, 2)
        b: (m,)
        norms: (m,) Euclidean norm of each row of A
    """
    n = points.shape[0]
    assert n >= 3, "Need at least 3 points for a polygon"

    center = points.mean(axis=0)

    A_rows = []
    b_vals = []

    for i in range(n):
        p0 = points[i]
        p1 = points[(i + 1) % n]

        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]

        # Edge normal candidate
        a = dy
        b = -dx
        c = -(a * p0[0] + b * p0[1])

        # Ensure 'inside' is A x + b <= 0 by checking the polygon center
        val_center = a * center[0] + b * center[1] + c
        if val_center > 0.0:
            a = -a
            b = -b
            c = -c

        A_rows.append([a, b])
        b_vals.append(c)

    A = np.array(A_rows, dtype=float)
    b = np.array(b_vals, dtype=float)
    norms = np.linalg.norm(A, axis=1)
    return A, b, norms


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """
    Extract yaw (rotation about Z) from a unit quaternion (x, y, z, w).
    """
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class PolytopeSafetyNode(Node):
    def __init__(self):
        super().__init__('polytope_safety_node')

        # Parameters: paths to YAML files and margin
        self.declare_parameter('agents_file', '')
        self.declare_parameter('boundary_file', '')
        self.declare_parameter('boundary_epsilon', 0.05)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('polygon_topic', '/safety_net/polygon')

        agents_file = self.get_parameter('agents_file').get_parameter_value().string_value
        boundary_file = self.get_parameter('boundary_file').get_parameter_value().string_value
        self.eps = float(self.get_parameter('boundary_epsilon').value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.polygon_topic = self.get_parameter('polygon_topic').get_parameter_value().string_value

        if not agents_file:
            raise RuntimeError("Parameter 'agents_file' is empty. Please set it in the launch file.")
        if not boundary_file:
            raise RuntimeError("Parameter 'boundary_file' is empty. Please set it in the launch file.")

        self.agents = load_agent_names(agents_file)

        # Load boundary points from boundary_file
        try:
            with open(boundary_file, 'r') as f:
                boundary_data = yaml.safe_load(f)
        except Exception as e:
            raise RuntimeError(f"Failed to load boundary_file '{boundary_file}': {e}")

        if 'points' not in boundary_data or not isinstance(boundary_data['points'], list):
            raise ValueError("boundary_file must contain a key 'points' with a list of {x,y} dicts.")

        raw_points = []
        for p in boundary_data['points']:
            if not isinstance(p, dict) or 'x' not in p or 'y' not in p:
                raise ValueError("Each point in 'points' must be a dict with keys 'x' and 'y'.")
            raw_points.append([float(p['x']), float(p['y'])])

        boundary_points = np.array(raw_points, dtype=float)
        if boundary_points.shape[0] < 3:
            raise ValueError("Need at least 3 boundary points.")

        # Build convex hull from arbitrary boundary points
        hull_points = convex_hull_2d(boundary_points)
        self.hull_points = hull_points

        # Build polytope from convex hull
        self.A, self.b, self.A_norms = polytope_from_hull(hull_points)

        polygon_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.boundary_polygon_pub = self.create_publisher(
            PolygonStamped,
            self.polygon_topic,
            polygon_qos,
        )

        self.get_logger().info(f"Loaded {len(self.agents)} agents from {agents_file}.")
        self.get_logger().info(f"Boundary hull has {hull_points.shape[0]} vertices and "
                               f"{self.A.shape[0]} facets.")
        self.get_logger().info(f"boundary_epsilon = {self.eps} m")
        self.get_logger().info(
            f"Publishing safety polygon to {self.polygon_topic} in frame '{self.frame_id}'."
        )

        # Per-agent state
        # pose: latest PoseStamped
        # raw_cmd_vel: latest Twist (body frame)
        # pub_cmd: publisher for safe cmd_vel
        self.agent_states: Dict[str, Dict] = {}
        for name in self.agents:
            self.agent_states[name] = {
                'pose': None,
                'raw_cmd_vel': None,
                'pub_cmd': self.create_publisher(Twist, f'/{name}/cmd_vel', 10),
            }

            # Pose subscriber: [name]/pose
            self.create_subscription(
                PoseStamped,
                f'/{name}/pose',
                self._make_pose_callback(name),
                qos_profile_sensor_data,
            )

            # Raw command velocity subscriber: [name]/raw_cmd_vel (body frame)
            self.create_subscription(
                Twist,
                f'/{name}/raw_cmd_vel',
                self._make_raw_cmd_callback(name),
                10,
            )

        # Timer for safety evaluation (e.g. 20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def _make_pose_callback(self, name: str):
        def cb(msg: PoseStamped):
            self.agent_states[name]['pose'] = msg
        return cb

    def _make_raw_cmd_callback(self, name: str):
        def cb(msg: Twist):
            self.agent_states[name]['raw_cmd_vel'] = msg
        return cb

    def timer_callback(self):
        self.publish_boundary_polygon()

        # For each agent, compute safe cmd_vel
        for name, state in self.agent_states.items():
            pose_msg = state['pose']
            raw_cmd = state['raw_cmd_vel']
            pub = state['pub_cmd']

            if pose_msg is None or raw_cmd is None:
                # Not enough info yet
                continue

            # 1) Extract 2D position
            x = pose_msg.pose.position.x
            y = pose_msg.pose.position.y
            p = np.array([x, y], dtype=float)

            # 2) Convert body-frame raw_cmd_vel to global-frame velocity
            q = pose_msg.pose.orientation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

            v_body = np.array([raw_cmd.linear.x, raw_cmd.linear.y], dtype=float)
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)

            # Rotation: v_global = R * v_body
            # R = [[cos, -sin],
            #      [sin,  cos]]
            v_global = np.array([
                cos_y * v_body[0] - sin_y * v_body[1],
                sin_y * v_body[0] + cos_y * v_body[1],
            ])

            # 3) Compute signed distances to each facet
            # Polytope: A p + b <= 0 inside
            signed = self.A @ p + self.b  # shape (m,)
            # Distance along outward normal: d_i = -signed_i / ||A_i||
            dists = -signed / self.A_norms  # >0 inside, 0 on, <0 outside
            min_dist = float(np.min(dists))

            # 4) Determine "active" facets:
            #    those within boundary_epsilon of the boundary, or outside.
            #    d_i <= eps  <=>  either near boundary (small positive) or outside (negative)
            active_mask = dists <= self.eps
            active_indices = np.where(active_mask)[0]

            # Default: pass-through (safe = raw_cmd)
            safe_cmd = Twist()
            safe_cmd.linear.x = raw_cmd.linear.x
            safe_cmd.linear.y = raw_cmd.linear.y
            safe_cmd.linear.z = raw_cmd.linear.z
            safe_cmd.angular.x = raw_cmd.angular.x
            safe_cmd.angular.y = raw_cmd.angular.y
            safe_cmd.angular.z = raw_cmd.angular.z

            stop = False

            if active_indices.size > 0:
                # 5) Check outward normal component for each active facet
                for i in active_indices:
                    a_i = self.A[i, :]  # outward normal
                    outward_component = float(a_i @ v_global)
                    # If outward_component > 0, this velocity pushes farther outside
                    if outward_component > 0.0:
                        stop = True
                        break

            # 6) If we must stop -> publish zero twist; else pass through raw_cmd_vel
            if stop:
                safe_cmd = Twist()  # all zeros

            pub.publish(safe_cmd)

    def publish_boundary_polygon(self):
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        for x, y in self.hull_points:
            point = Point32()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.0
            msg.polygon.points.append(point)

        self.boundary_polygon_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PolytopeSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
