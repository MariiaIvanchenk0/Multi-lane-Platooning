"""
Longitudinal Controller
Based on paper: MeereFidanHeemels2023_IFAC

Step 1: Velocity Error
    e_v = v - v_des
    omega_dot += e_v (?)

Step 2: Adaptive laws
    alpha_bar_hat_dot = - gamma_alpha * e_v * tau
    beta_hat_dot = gamma_beta * v^2 e_v
    delta_hat_dot = gamma_delta * e_v

    tau = - k_1 * e_v - k_2 * omega - beta_hat * v^2 - delta_hat + v_des_dot

Step 3: Torque
    T = alpha_bar_hat * tau
    T = alpha_bar_hat * (- k_1 * e_v - k_2 * omega - beta_hat * v^2 - delta_hat + v_des_dot)

    
Lateral Controller

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
from rclpy.qos import QoSProfile, HistoryPolicy


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Declare parameters (Longitudinal)
        self.declare_parameter('k_1', 25.0)
        self.declare_parameter('k_2', 1.5)
        self.declare_parameter('frequency', 20.0)

        self.declare_parameter('gamma_alpha', 0.001)
        self.declare_parameter('gamma_beta', 0.0001)
        self.declare_parameter('gamma_delta', 0.01)
        
        self.declare_parameter('alpha_bar_hat', 833.33)    # Adaptive guess for (1 / alpha)
        self.declare_parameter('beta_hat', -0.0001)     # Adaptive guess for aerodynamic drag coefficient
        self.declare_parameter('delta_hat', -0.1)       # Adaptive guess for constant disturbance/friction

        # Declare parameters (Lateral)
        self.declare_parameter('k_a1', 1.5)
        self.declare_parameter('k_a2', 3.0)
        self.declare_parameter('l_lane', 0.0)
        self.declare_parameter('R', 20.0)
        self.declare_parameter('wheelbase', 2.5)

        # Get parameters (Longitudinal)
        self.k_1 = self.get_parameter('k_1').value
        self.k_2 = self.get_parameter('k_2').value

        self.gamma_alpha = self.get_parameter('gamma_alpha').value
        self.gamma_beta = self.get_parameter('gamma_beta').value
        self.gamma_delta = self.get_parameter('gamma_delta').value

        self.alpha_bar_hat = self.get_parameter('alpha_bar_hat').value
        self.beta_hat = self.get_parameter('beta_hat').value
        self.delta_hat = self.get_parameter('delta_hat').value

        # Get parameters (Lateral)
        self.k_a1 = self.get_parameter('k_a1').value
        self.k_a2 = self.get_parameter('k_a2').value
        self.l_lane = self.get_parameter('l_lane').value   
        self.R = self.get_parameter('R').value 
        self.L = self.get_parameter('wheelbase').value

        self.prev_v_des = 0.0
        self.v_des = 0.0
        self.l_des = 0.0
        self.omega = 0.0          # Accumulated velocity error state
        self.state = [0.0, 0.0, 0.0, 0.0]
        self.dt = 1.0 / self.get_parameter('frequency').value # period 

        qos_profile = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)

        # Subscriptions & Publisher & Timer
        self.kinematic_sub = self.create_subscription(Float64MultiArray, 'kinematic_input', self.kinematic_callback, qos_profile)
        self.state_sub = self.create_subscription(Float64MultiArray, 'vehicle_state', self.state_callback, qos_profile)

        # self.steering_pub = self.create_publisher(Float64, 'cntl_phi', qos_profile)
        # self.torque_pub = self.create_publisher(Float64, 'cntl_torque', qos_profile)

        self.control_pub = self.create_publisher(Float64MultiArray, 'cntl_vector', qos_profile)
        self.timer = self.create_timer(self.dt, self.control_loop_callback)
        # self.get_logger().info("Longitudinal Adaptive Controller Node Initialized.")
    
    def state_callback(self, msg):
        self.state = msg.data
    
    def kinematic_callback(self, msg):
        self.v_des = msg.data[0]
        self.l_des = msg.data[1]

    def control_loop_callback(self):
        # Longitudinal Controller
        v = self.state[3]
        v_des_dot = (self.v_des - self.prev_v_des) / self.dt
        
        # --- Step 1: Calculate Velocity Error ---
        e_v = v - self.v_des
        
        # --- Step 2: Calculate Tau (Intermediate Control Action) ---
        tau = (- self.k_1 * e_v 
               - self.k_2 * self.omega 
               - self.beta_hat * (v ** 2) 
               - self.delta_hat
               + v_des_dot)
        
        # --- Step 3: Compute Adaptive Law Derivatives (ODEs) ---
        omega_dot = e_v
        alpha_bar_hat_dot = -self.gamma_alpha * e_v * tau
        beta_hat_dot = self.gamma_beta * (v ** 2) * e_v
        delta_hat_dot = self.gamma_delta * e_v
        
        # --- Step 4: Discrete Numerical Integration (Forward Euler) ---
        self.omega += omega_dot * self.dt
        self.alpha_bar_hat += alpha_bar_hat_dot * self.dt
        self.beta_hat += beta_hat_dot * self.dt
        self.delta_hat += delta_hat_dot * self.dt

        if self.alpha_bar_hat < 0.0001:
            self.alpha_bar_hat = 0.0001

        self.omega = max(min(self.omega, 20.0), -20.0)

        self.beta_hat = max(min(self.beta_hat, 0.0), -0.01)
        self.delta_hat = max(min(self.delta_hat, 0.0), -5.0)               

        # --- Step 5: Calculate Final Torque ---
        torque = self.alpha_bar_hat * tau

        MAX_TORQUE = 2500.0
        MIN_TORQUE = -1500.0
        torque = max(min(torque, MAX_TORQUE), MIN_TORQUE)
        self.prev_v_des = self.v_des

        # self.get_logger().info(f"v: {v}, v_des: {self.v_des}")

        # Lateral Controller
        l = self.state[1]
        psi = self.state[2]
        
        # --- Step 1: Calculate Errors ---
        e_psi = -psi
        e_lat = self.l_des - l # self.l_lane - 
        
        # --- Step 2: Calculate Steering Angle Components ---
        numerator = -math.cos(e_psi) * e_lat - (self.k_a1 + self.k_a2) * math.sin(e_psi)
        denominator = self.k_a1 - (self.k_a1 + self.k_a2) * math.cos(e_psi) + math.sin(e_psi) * e_lat

        if abs(denominator) < 1e-6:
            denominator = 1e-6 if denominator >= 0 else -1e-6

        # phi_feedforward = math.atan2(self.L, self.R)
        phi = math.atan(numerator / denominator) # + phi_feedforward
        # phi = math.atan2(numerator, denominator) 
        
        MAX_STEER = math.radians(45.0)
        phi = max(min(phi, MAX_STEER), -MAX_STEER)
        
        # self.get_logger().info(f"phi: {phi}, l_des: {self.l_des}")

        # Publishing data
        msg = Float64MultiArray()
        msg.data = [torque, phi]
        self.control_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down controller node cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
