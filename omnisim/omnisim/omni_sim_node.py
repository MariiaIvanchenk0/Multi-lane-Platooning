#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from typing import Dict, List

import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger


def s1_exp(theta: float) -> complex:
    """Exponential map from the S1 Lie algebra to the unit complex group."""
    return complex(math.cos(theta), math.sin(theta))


def normalize_s1(q: complex) -> complex:
    norm = abs(q)
    if norm == 0.0:
        raise ValueError('Cannot normalize zero as a unit complex number.')
    return q / norm


def yaw_to_s1(yaw: float) -> complex:
    return s1_exp(yaw)


def s1_to_yaw(q: complex) -> float:
    return math.atan2(q.imag, q.real)


def yaw_to_quaternion_z(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


@dataclass
class AgentState:
    t: float
    p: complex
    q: complex


@dataclass
class AgentCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    stamp_sec: float = -1.0


class OmniSimNode(Node):
    """
    Simulates planar omni robots.

    For each agent name in agents.yaml:
      subscribes: /<name>/cmd_vel
      publishes:  /<name>/pose

    Services:
      /start : start/resume the simulator engine
      /pause : pause the simulator engine
      /reset : reset states to initial poses and restart the simulator

    State:
      p = x + i y in the world frame
      q = cos(yaw) + i sin(yaw) on S1

    Dynamics when use_body_frame_cmd is true:
      p_dot = q * (vx_body + i vy_body)
      yaw_dot = wz

    The heading q is integrated using a Lie-group RK4 update on S1. The
    position p is integrated with the coupled RK4 stages.
    """

    def __init__(self):
        super().__init__('omni_sim_node')

        self.declare_parameter('agents_file', '')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('initial_spacing', 1.0)
        self.declare_parameter('use_body_frame_cmd', True)
        self.declare_parameter('start_running', True)

        self.agents_file = self.get_parameter('agents_file').get_parameter_value().string_value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.initial_spacing = float(self.get_parameter('initial_spacing').value)
        self.use_body_frame_cmd = bool(self.get_parameter('use_body_frame_cmd').value)
        self.is_running = bool(self.get_parameter('start_running').value)

        if not self.agents_file:
            raise RuntimeError("Parameter 'agents_file' is empty.")
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive.')
        if self.cmd_timeout < 0.0:
            raise ValueError('cmd_timeout must be non-negative.')

        self.agent_names, self.initial_poses = self._load_agents(self.agents_file)

        self.states: Dict[str, AgentState] = {}
        self.commands: Dict[str, AgentCommand] = {}
        self.pose_pubs = {}
        self.cmd_subs = []

        now_sec = self._now_sec()
        self._initialize_states(now_sec)
        self._initialize_commands(now_sec)

        for name in self.agent_names:
            cmd_topic = f'/{name}/cmd_vel'
            pose_topic = f'/{name}/pose'

            self.cmd_subs.append(
                self.create_subscription(
                    Twist,
                    cmd_topic,
                    lambda msg, agent_name=name: self._cmd_cb(agent_name, msg),
                    10,
                )
            )

            self.pose_pubs[name] = self.create_publisher(PoseStamped, pose_topic, 10)

            self.get_logger().info(f'Subscribing to {cmd_topic}')
            self.get_logger().info(f'Publishing to {pose_topic}')

        self.start_srv = self.create_service(Trigger, '/start', self._start_cb)
        self.pause_srv = self.create_service(Trigger, '/pause', self._pause_cb)
        self.reset_srv = self.create_service(Trigger, '/reset', self._reset_cb)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._timer_cb)

        state_text = 'running' if self.is_running else 'paused'

        self.get_logger().info(
            f'Loaded {len(self.agent_names)} agents from {self.agents_file}; '
            f'simulating at {self.publish_rate_hz:.1f} Hz; '
            f'initial state: {state_text}.'
        )

        self.get_logger().info('Services available: /start, /pause, /reset')

    @staticmethod
    def _load_agents(agents_file: str):
        path = os.path.expandvars(os.path.expanduser(agents_file))

        if not os.path.exists(path):
            raise FileNotFoundError(f'Agents file not found: {path}')

        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}

        raw_agents = data.get('agents', [])

        if not isinstance(raw_agents, list) or not raw_agents:
            raise ValueError("agents_file must contain key 'agents' with a non-empty list.")

        names: List[str] = []
        initial_poses = {}

        for agent in raw_agents:
            if isinstance(agent, dict):
                if 'name' not in agent:
                    raise ValueError(f"Agent entry is missing 'name': {agent}")

                name = str(agent['name'])
                names.append(name)

                if isinstance(agent.get('initial_pose'), dict):
                    initial_poses[name] = agent['initial_pose']
            else:
                names.append(str(agent))

        return names, initial_poses

    def _initialize_states(self, now_sec: float):
        self.states.clear()

        for index, name in enumerate(self.agent_names):
            pose = self.initial_poses.get(name)

            if pose is None:
                x = self.initial_spacing * float(index)
                y = 0.0
                yaw = 0.0
            else:
                x = float(pose.get('x', 0.0))
                y = float(pose.get('y', 0.0))
                yaw = float(pose.get('yaw', 0.0))

            self.states[name] = AgentState(
                t=now_sec,
                p=complex(x, y),
                q=yaw_to_s1(yaw),
            )

    def _initialize_commands(self, now_sec: float):
        self.commands.clear()

        for name in self.agent_names:
            self.commands[name] = AgentCommand(stamp_sec=now_sec)

    def _start_cb(self, request, response):
        del request

        now_sec = self._now_sec()

        # Avoid a large dt jump after being paused.
        for name in self.agent_names:
            state = self.states[name]
            self.states[name] = AgentState(
                t=now_sec,
                p=state.p,
                q=state.q,
            )

        self.is_running = True

        response.success = True
        response.message = 'Simulator started.'
        self.get_logger().info(response.message)
        return response

    def _pause_cb(self, request, response):
        del request

        now_sec = self._now_sec()

        # Keep state timestamps current so resuming does not integrate across pause time.
        for name in self.agent_names:
            state = self.states[name]
            self.states[name] = AgentState(
                t=now_sec,
                p=state.p,
                q=state.q,
            )

        self.is_running = False

        response.success = True
        response.message = 'Simulator paused.'
        self.get_logger().info(response.message)
        return response

    def _reset_cb(self, request, response):
        del request

        now_sec = self._now_sec()

        self._initialize_states(now_sec)
        self._initialize_commands(now_sec)

        # "Restart" means reset and run again.
        # Change this to False if you want reset to leave the simulator paused.
        self.is_running = True

        for name in self.agent_names:
            self._publish_pose(name)

        response.success = True
        response.message = 'Simulator reset to initial conditions and restarted.'
        self.get_logger().info(response.message)
        return response

    def _cmd_cb(self, agent_name: str, msg: Twist):
        self.commands[agent_name] = AgentCommand(
            vx=float(msg.linear.x),
            vy=float(msg.linear.y),
            wz=float(msg.angular.z),
            stamp_sec=self._now_sec(),
        )

    def _timer_cb(self):
        now_sec = self._now_sec()

        for name in self.agent_names:
            state = self.states[name]

            if self.is_running:
                dt = now_sec - state.t

                if dt > 0.0:
                    cmd = self._fresh_command(name, now_sec)
                    self.states[name] = self._rk4_step(state, cmd, dt)
            else:
                # While paused, keep the timestamp fresh but freeze p and q.
                self.states[name] = AgentState(
                    t=now_sec,
                    p=state.p,
                    q=state.q,
                )

            self._publish_pose(name)

    def _fresh_command(self, name: str, now_sec: float) -> AgentCommand:
        cmd = self.commands[name]

        if self.cmd_timeout == 0.0:
            return cmd

        if now_sec - cmd.stamp_sec > self.cmd_timeout:
            return AgentCommand(stamp_sec=now_sec)

        return cmd

    def _rk4_step(self, state: AgentState, cmd: AgentCommand, h: float) -> AgentState:
        t = state.t
        p = state.p
        q = normalize_s1(state.q)

        k1_p = self._p_dot(p, q, cmd)
        k1_th = cmd.wz

        p2 = p + 0.5 * h * k1_p
        q2 = s1_exp(0.5 * h * k1_th) * q
        k2_p = self._p_dot(p2, q2, cmd)
        k2_th = cmd.wz

        p3 = p + 0.5 * h * k2_p
        q3 = s1_exp(0.5 * h * k2_th) * q
        k3_p = self._p_dot(p3, q3, cmd)
        k3_th = cmd.wz

        p4 = p + h * k3_p
        q4 = s1_exp(h * k3_th) * q
        k4_p = self._p_dot(p4, q4, cmd)
        k4_th = cmd.wz

        p_next = p + (h / 6.0) * (
            k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p
        )

        delta_theta = (h / 6.0) * (
            k1_th + 2.0 * k2_th + 2.0 * k3_th + k4_th
        )

        q_next = normalize_s1(s1_exp(delta_theta) * q)

        return AgentState(t=t + h, p=p_next, q=q_next)

    def _p_dot(self, p: complex, q: complex, cmd: AgentCommand) -> complex:
        del p

        v = complex(cmd.vx, cmd.vy)

        if self.use_body_frame_cmd:
            return q * v

        return v

    def _publish_pose(self, name: str):
        state = self.states[name]
        yaw = s1_to_yaw(state.q)
        qx, qy, qz, qw = yaw_to_quaternion_z(yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(state.p.real)
        msg.pose.position.y = float(state.p.imag)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_pubs[name].publish(msg)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = OmniSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()