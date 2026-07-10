#!/usr/bin/env python3
"""
RK4 integrator for the kinematic bicycle model.

The node maintains the robot state [s, l, psi] and advances it at a fixed
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
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float64MultiArray
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Quaternion, Point, PoseStamped

class ModelSimulationNode(Node):
    def __init__(self):
        super().__init__('model_simulation_node')

        # Declare parameters
        self.declare_parameter('id', 1)
        self.declare_parameter('frequency', 100.0)

        # True plant dynamics values from paper
        self.declare_parameter('alpha', 0.0012)
        self.declare_parameter('beta', -0.0001)
        self.declare_parameter('delta', -0.1)

        self.declare_parameter('s0', 0.0)
        self.declare_parameter('l0', 0.0)
        self.declare_parameter('psi0', 0.0)
        self.declare_parameter('v0', 0.0)

        self.declare_parameter('wheelbase', 2.5)
        self.declare_parameter('base_frame', 'robot_bs')

        # Get parameters
        self.id = self.get_parameter('id').value
        self.dt = 1.0 / self.get_parameter('frequency').value 

        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.delta = self.get_parameter('delta').value

        # Physics State Vector must include velocity: [s, l, psi, v]
        self.state = [
            self.get_parameter('s0').value,
            self.get_parameter('l0').value,
            self.get_parameter('psi0').value,
            self.get_parameter('v0').value
        ]

        self.L = self.get_parameter('wheelbase').value
        self.base_frame = self.get_parameter('base_frame').value + f"_{self.id}"

        self.T = 0.0
        self.phi = 0.0
        self.reference_path = self.generate_custom_path()

        # Callbacks
        self.torque_sub = self.create_subscription(Float64, 'cntl_torque', self.torque_callback, 10)
        self.phi_sub = self.create_subscription(Float64, 'cntl_phi', self.phi_callback, 10)

        self.lane_pub = self.create_publisher(Marker, 'lane_marker', 10)
        self.path_pub = self.create_publisher(Path, 'global_reference_path', 10)
        self.marker_pub = self.create_publisher(Marker, 'model_marker', 10)
        self.state_pub = self.create_publisher(Float64MultiArray, 'vehicle_state', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(self.dt, self.step)

    def dynamics(self, state):
        """Returns the true continuous-time derivatives [s_dot, l_dot, psi_dot, v_dot]"""
        _, _, psi, v = state
        
        # 2D Kinematics
        s_dot = v * math.cos(psi)
        l_dot = v * math.sin(psi)
        psi_dot = (v / self.L) * math.tan(self.phi)

        # Longitudinal Dynamics: This equals v_dot (acceleration)
        v_dot = self.alpha * self.T + self.beta * (v**2) + self.delta

        return [s_dot, l_dot, psi_dot, v_dot]

    def rk4_step(self, state, dt):
        """Advances full state vector by one proper mathematical RK4 step."""
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

    def step(self):
        """Timer callback: integrate state components and publish out."""
        self.state = self.rk4_step(self.state, self.dt)
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))  # wrap heading angle to [-pi, pi]

        # Publish state array [s, l, psi, v]
        msg = Float64MultiArray()
        msg.data = self.state
        self.state_pub.publish(msg)
        
        self.publish_to_sim(self.state)
        
        # if self.id == 1:
        self.publish_lane_centerline()

    def torque_callback(self, msg):
        self.T = msg.data

    def phi_callback(self, msg):
        self.phi = msg.data

    def yaw_to_quaternion(self, yaw):
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    def frenet_to_cartesian(self, state):
        """Converts internal Frenet vector safely to Cartesian coordinates"""
        s, l, psi, _ = state # Correctly unpacks from the 4-element state array
        
        # Look up nearest waypoint along generated spatial curve
        closest_wp = min(self.reference_path, key=lambda wp: abs(wp['s'] - s))
        
        x_r = closest_wp['x']
        y_r = closest_wp['y']
        theta_r = closest_wp['theta']   

        # Position Projections
        x = x_r - l * math.sin(theta_r)
        y = y_r + l * math.cos(theta_r)
        global_theta = theta_r + psi
        
        return x, y, global_theta
    
    def generate_custom_path(self):
        """Generates an arbitrary reference track array."""
        path_points = []
        num_points = 500
        R = 20.0 
        
        for i in range(num_points + 1):
            theta_r = (2.0 * math.pi / num_points) * i
            s = R * theta_r
            
            path_points.append({
                'x': R * math.sin(theta_r),
                'y': R - R * math.cos(theta_r),
                'theta': theta_r,
                's': s
            })
        return path_points

    def publish_lane_centerline(self):
        """Publishes the generalized reference path to RViz and topics."""
        now = self.get_clock().now().to_msg()
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = now
        marker.ns = "environment"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.3 
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.6

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = now

        for wp in self.reference_path:
            p = Point()
            p.x = wp['x']
            p.y = wp['y']
            p.z = 0.0
            marker.points.append(p)
            
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = now
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = 0.0
            
            q = self.yaw_to_quaternion(wp['theta'])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path_msg.poses.append(pose)
            
        self.lane_pub.publish(marker)
        self.path_pub.publish(path_msg)

    def publish_to_sim(self, state):
        x, y, theta = self.frenet_to_cartesian(state)
        now = self.get_clock().now().to_msg()

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(theta / 2.0)
        q.w = math.cos(theta / 2.0)

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
    
        marker.scale.x = 4.5  
        marker.scale.y = 1.8  
        marker.scale.z = 1.4  
        
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0  
        marker.color.a = 1.0  
        
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