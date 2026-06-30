#!/usr/bin/env python3
"""
RK4 integrator for the kinematic bicycle model.

The node maintains the robot state [s, l, psi, v] and advances it at a fixed
time step using the classical fourth-order Runge-Kutta (RK4) method.

Based on paper: MeereFidanHeemels2023_IFAC

Kinematic bicycle model (reference point at the front axle):

    s_dot = u_s = v * cos(psi)
    l_dot = u_l = v * sin(psi)
    psi_dot = (v / L) * tan(phi)
    v_dot = alpha * T + beta * v^2 + delta

    u = [u_s, u_l] - the kinematic (velocity) input

The integrated pose is broadcasted on /tf and /model_marker so it can be visualised directly in RViz.
"""

import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Quaternion


class ModelSimulationNode(Node):
    def __init__(self):
        super().__init__('model_simulation_node')

        # Declare parameters
        self.declare_parameter('wheelbase', 0.5)
        self.declare_parameter('frequency', 50.0)

        self.declare_parameter('alpha', 0.012)
        self.declare_parameter('beta', -0.01)
        self.declare_parameter('delta', -0.1)

        self.declare_parameter('s0', 0.0)
        self.declare_parameter('l0', 0.0)
        self.declare_parameter('psi0', 0.0)
        self.declare_parameter('v0', 0.0)

        self.declare_parameter('torque', 100.0)
        self.declare_parameter('phi', 0.05)
        self.declare_parameter('base_frame', 'robot_1')

        # Get parameters
        self.L = self.get_parameter('wheelbase').value
        self.dt = 1.0 / self.get_parameter('frequency').value # period 

        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.delta = self.get_parameter('delta').value

        self.state = [
            self.get_parameter('s0').value,
            self.get_parameter('l0').value,
            self.get_parameter('psi0').value,
            self.get_parameter('v0').value,
        ]

        self.base_frame = self.get_parameter('base_frame').value

        # Callbacks
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(self.dt, self.step)
        self.marker_pub = self.create_publisher(Marker, '/model_marker', 10)

    # RK4 Integration Layer:
    def dynamics(self, state, phi):
        "Return the state derivative [s_dot, l_dot, psi_dot, v_dot]."
        _, _, psi, v = state
        s_dot = v * math.cos(psi)
        l_dot = v * math.sin(psi)
        psi_dot = (v / self.L) * math.tan(phi)
        v_dot = self.alpha * self.T + self.beta * (v**2) + self.delta
        return [s_dot, l_dot, psi_dot, v_dot]

    def rk4_step(self, state, phi, dt):
        "Advance `state` by one RK4 step of length `dt`."
        def add(s, k, scale):
            return [s[i] + scale * k[i] for i in range(len(s))]

        k1 = self.dynamics(state, phi)
        k2 = self.dynamics(add(state, k1, dt / 2.0), phi)
        k3 = self.dynamics(add(state, k2, dt / 2.0), phi)
        k4 = self.dynamics(add(state, k3, dt), phi)

        return [
            state[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
            for i in range(len(state))
        ]

    def step(self):
        "Timer callback: integrate one step and publish the pose."
        self.T = self.get_parameter('torque').value
        self.phi = self.get_parameter('phi').valuemodel_simulation_node:
  ros__parameters:
    alpha: 0.012                # the unknown input gain
    beta: -0.01               # the unknown damping coefficient
    delta: -0.1                 # a fixed unknown disturbance
    wheelbase: 0.5              # wheelbase (L)
    base_frame: "robot_1"       # name for this robot instance

    frequency: 50.0             # integration rate [Hz]
    s0: 0.0                     # initial s (longitudinal position)
    l0: 0.0                     # initial l (lateral position)
    psi0: 0.0                   # initial psi (yaw/heading angle)
    v0: 0.0                     # initial velocity

    # Dynamic parameters
    torque: 100.0               # the input torque
    phi: 0.05                   # the input steering angle

        self.state = self.rk4_step(self.state, self.phi, self.dt)
        self.state[2] = math.atan2(math.sin(self.state[2]),math.cos(self.state[2]))  # wrap heading to [-pi, pi]
        self.publish_to_sim(self.state)


    def frenet_to_cartesian(self, state):
        "Converts Frenet coordinates (s, l) to Global Cartesian (x, y, theta)"

        s, l, psi, v = state
        R = 175.0  # Constant road curve radius from MeereFidanHeemels2023_IFAC.pdf
        kappa_r = 1.0 / R

        v_lat = v * math.sin(psi)
        
        # Compute reference path properties at 's'
        theta_r = s / R
        x_r = R * math.sin(theta_r)
        y_r = R - R * math.cos(theta_r)
        
        # Position Formulas
        x = x_r - l * math.sin(theta_r)
        y = y_r + l * math.cos(theta_r)
        
        # Orientation & Velocity Formulas
        # s_dot = v_long, d_dot = v_lat
        denominator = 1.0 - kappa_r * l
        
        if abs(denominator) < 1e-6:
            denominator = 1e-6
            
        global_theta = theta_r + math.atan2(v_lat, denominator)
        
        return x, y, global_theta

    def publish_to_sim(self, state):
        # x, y, theta = self.frenet_to_cartesian(state)
        x, y, theta, _ = state 
        now = self.get_clock().now().to_msg()

        # Construct Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(theta / 2.0)
        q.w = math.cos(theta / 2.0)

        # Broadcast TF
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "map"
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf)

        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = now
        marker.ns = self.base_frame
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
    
        marker.pose.position.x = - (self.L / 2.0)
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.7
    
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
    
        marker.scale.x = 4.5  # Length
        marker.scale.y = 1.8  # Width
        marker.scale.z = 1.4  # Height
        
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0  # Nice light blue car
        marker.color.a = 1.0  # Fully opaque
        
        marker.lifetime.sec = 0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ModelSimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
