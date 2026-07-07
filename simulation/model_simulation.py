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
from std_msgs.msg import Float64, Float64MultiArray
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Quaternion, Point


class ModelSimulationNode(Node):
    def __init__(self):
        super().__init__('model_simulation_node')

        # Declare parameters
        self.declare_parameter('id')
        self.declare_parameter('wheelbase', 0.5)
        self.declare_parameter('frequency', 20.0)
        self.declare_parameter('R', 20.0)

        self.declare_parameter('alpha', 0.012)
        self.declare_parameter('beta', -0.01)
        self.declare_parameter('delta', -0.1)

        self.declare_parameter('s0', 0.0)
        self.declare_parameter('l0', 0.0)
        self.declare_parameter('psi0', 0.0)
        self.declare_parameter('v0', 0.0)

        self.declare_parameter('base_frame', 'robot_bs')

        # Get parameters
        self.id = self.get_parameter('id').value
        self.L = self.get_parameter('wheelbase').value
        self.dt = 1.0 / self.get_parameter('frequency').value # period 
        self.R = self.get_parameter('R').value

        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.delta = self.get_parameter('delta').value

        self.state = [
            self.get_parameter('s0').value,
            self.get_parameter('l0').value,
            self.get_parameter('psi0').value,
            self.get_parameter('v0').value,
        ]

        self.base_frame = self.get_parameter('base_frame').value + f"_{self.id}"
        self.T = 0.0
        self.phi = 0.0

        # Callbacks
        self.torque_sub = self.create_subscription(Float64, 'cntl_torque', self.torque_callback, 10)
        self.phi_sub = self.create_subscription(Float64, 'cntl_phi', self.phi_callback, 10)

        self.lane_pub = self.create_publisher(Marker, 'lane_marker', 10)
        self.marker_pub = self.create_publisher(Marker, 'model_marker', 10)
        self.state_pub = self.create_publisher(Float64MultiArray, 'vehicle_state', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(self.dt, self.step)

    # RK4 Integration Layer:
    def dynamics(self, state):
        "Return the state derivative [s_dot, l_dot, psi_dot, v_dot]."
        _, l, psi, v = state

        kappa_r = 1.0 / self.R
        
        denominator = 1.0 - kappa_r * l
        if abs(denominator) < 1e-6:
            denominator = 1e-6
            
        s_dot = (v * math.cos(psi)) / denominator
        l_dot = v * math.sin(psi)
        psi_dot = ((v / self.L) * math.tan(self.phi)) - (kappa_r * s_dot)
        # self.get_logger().info(f"v: {v}, T: {self.T}")
        v_dot = self.alpha * self.T + self.beta * (v**2) + self.delta

        return [s_dot, l_dot, psi_dot, v_dot]

    def rk4_step(self, state, dt):
        "Advance `state` by one RK4 step of length `dt`."
        def add(s, k, scale):
            return [s[i] + scale * k[i] for i in range(len(s))]

        k1 = self.dynamics(state)
        k2 = self.dynamics(add(state, k1, dt / 2.0))
        k3 = self.dynamics(add(state, k2, dt / 2.0))
        k4 = self.dynamics(add(state, k3, dt))

        return [
            state[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
            for i in range(len(state))
        ]
    
    def torque_callback(self, msg):
        self.T = msg.data

    def phi_callback(self, msg):
        self.phi = msg.data

    def step(self):
        "Timer callback: integrate one step and publish the pose."

        self.state = self.rk4_step(self.state, self.dt)
        self.state[2] = math.atan2(math.sin(self.state[2]),math.cos(self.state[2]))  # wrap heading to [-pi, pi]

        msg = Float64MultiArray()
        msg.data = self.state
        self.state_pub.publish(msg)
        self.publish_to_sim(self.state)
        self.publish_lane_centerline()


    def frenet_to_cartesian(self, state):
        "Converts Frenet coordinates (s, l) to Global Cartesian (x, y, theta)"

        s, l, psi, v = state
          # Constant road curve radius from MeereFidanHeemels2023_IFAC.pdf
        kappa_r = 1.0 / self.R

        v_lat = v * math.sin(psi)
        
        # Compute reference path properties at 's'
        theta_r = s /  self.R
        x_r = self.R * math.sin(theta_r)
        y_r = self.R - self.R * math.cos(theta_r)
        
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
    
    def publish_lane_centerline(self):
        "Publishes a static circular centerline to RViz representing the lane"
        now = self.get_clock().now().to_msg()
        marker = Marker()
        marker.header.frame_id = "map"  # Must match your simulation frame
        marker.header.stamp = now
        marker.ns = "environment"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Line width thickness (0.3 meters wide)
        marker.scale.x = 0.3 
        
        # Color: Bright semi-transparent green
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.6
        num_points = 200
        
        # Loop 360 degrees around the circle to generate the track points
        for i in range(num_points + 1):
            theta_r = (2.0 * math.pi / num_points) * i
            p = Point()
            p.x = self.R * math.sin(theta_r)
            p.y = self.R - self.R * math.cos(theta_r)
            p.z = 0.0
            marker.points.append(p)
            
        self.lane_pub.publish(marker)

    def publish_to_sim(self, state):
        x, y, theta = self.frenet_to_cartesian(state)
        # x, y, theta, _ = state
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