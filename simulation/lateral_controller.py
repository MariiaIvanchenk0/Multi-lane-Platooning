"""
Lateral Controller
Based on paper: MeereFidanHeemels2023_IFAC

Step 1: Calculate Errors
    e_psi = - psi
    e_lat = l_lane - l_des - l

Step 2: Calculate steering angle
    numerator = - cos(e_psi) * e_lat - (k_a1 + k_a2) * sin(e_psi)
    denominator = k_a1 - (k_a1 + k_a2) * cos(e_psi) + sin(e_psi) * e_lat
    phi = arctan(num, denom)

"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class LateralControllerNode(Node):
    def __init__(self):
        super().__init__('lateral_controller_node')
        
        # Declare parameters
        self.declare_parameter('l_lane', 0.0)
        self.declare_parameter('k_a1', 1.5)
        self.declare_parameter('k_a2', 3.0)
        self.declare_parameter('R', 20.0)
        self.declare_parameter('wheelbase', 0.5)
        self.declare_parameter('frequency', 20.0)

        # Get parameters
        self.state = [0.0, 0.0, 0.0, 0.0]
        self.dt = 1.0 / self.get_parameter('frequency').value # period   
        self.l_lane = self.get_parameter('l_lane').value   
        self.k_a1 = self.get_parameter('k_a1').value
        self.k_a2 = self.get_parameter('k_a2').value
        self.R = self.get_parameter('R').value
        self.L = self.get_parameter('wheelbase').value
        
        self.l_des = 0.0
        
        # Publisher & Timer
        self.state_sub = self.create_subscription(Float64MultiArray, 'vehicle_state', self.state_callback, 10)
        self.ldes_sub = self.create_subscription(Float64, 'l_des', self.ldes_callback, 10)
        self.state_pub = self.create_publisher(Float64MultiArray, 'updated_state', 10)
        self.steering_pub = self.create_publisher(Float64, 'cntl_phi', 10)
        self.timer = self.create_timer(self.dt, self.control_loop_callback)
        # self.get_logger().info("Lateral Geometric Controller Node Initialized.")
    
    def state_callback(self, msg):
        self.state = msg.data

    def ldes_callback(self, msg):
        self.l_des = msg.data

    def control_loop_callback(self):
        l = self.state[1]
        psi = self.state[2]
        
        # --- Step 1: Calculate Errors ---
        e_psi = -psi
        e_lat = self.l_lane - self.l_des - l  # self.l_des - l 
        
        # --- Step 2: Calculate Steering Angle Components ---
        numerator = -math.cos(e_psi) * e_lat - (self.k_a1 + self.k_a2) * math.sin(e_psi)
        denominator = self.k_a1 - (self.k_a1 + self.k_a2) * math.cos(e_psi) + math.sin(e_psi) * e_lat

        if abs(denominator) < 1e-6:
            denominator = 1e-6 if denominator >= 0 else -1e-6

        phi_feedback = math.atan(numerator / denominator)

        phi_feedforward = math.atan(self.L / self.R)
        phi = phi_feedback + phi_feedforward
        
        MAX_STEER = math.radians(35.0)
        phi = max(min(phi, MAX_STEER), -MAX_STEER)
        
        # (Closing the Loop)
        # l_dot = v * math.sin(psi)
        # psi_dot = (v / self.L) * math.tan(phi)
        
        # Advance the physical simulation states using Forward Euler integration[cite: 1]
        # self.current_l += l_dot * self.dt
        # self.current_psi += psi_dot * self.dt
        
        msg = Float64()
        msg.data = phi
        self.steering_pub.publish(msg)

        # self.get_logger().info(f"phi: {phi}, l_des: {self.l_des}")
        
        # self.get_logger().info(
        #     f"Lat Error: {e_lat:.3f}m | Yaw Error: {math.degrees(e_psi):.1f}° | "
        #     f"Steer Output (phi): {math.degrees(phi):.1f}°"
        # )

def main(args=None):
    rclpy.init(args=args)
    node = LateralControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()