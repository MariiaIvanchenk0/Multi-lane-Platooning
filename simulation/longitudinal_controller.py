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

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

class LongitudinalSimNode(Node):
    def __init__(self):
        super().__init__('longitudinal_controller_node')
        
        # Declare parameters
        self.declare_parameter('k_1')
        self.declare_parameter('k_2')
        # self.declare_parameter('v_des', 25.0)
        self.declare_parameter('frequency')

        self.declare_parameter('gamma_alpha')
        self.declare_parameter('gamma_beta')
        self.declare_parameter('gamma_delta')
        
        self.declare_parameter('alpha_bar_hat')    # Adaptive guess for (1 / alpha)
        self.declare_parameter('beta_hat')     # Adaptive guess for aerodynamic drag coefficient
        self.declare_parameter('delta_hat')       # Adaptive guess for constant disturbance/friction

        self.omega = 0.0          # Accumulated velocity error state
        self.state = [0.0, 0.0, 0.0, 0.0]

        # Get parameters
        self.k_1 = self.get_parameter('k_1').value
        self.k_2 = self.get_parameter('k_2').value
        # self.v_des = self.get_parameter('v_des').value

        self.gamma_alpha = self.get_parameter('gamma_alpha').value
        self.gamma_beta = self.get_parameter('gamma_beta').value
        self.gamma_delta = self.get_parameter('gamma_delta').value

        self.alpha_bar_hat = self.get_parameter('alpha_bar_hat').value
        self.beta_hat = self.get_parameter('beta_hat').value
        self.delta_hat = self.get_parameter('delta_hat').value

        # Publisher & Timer
        self.state_sub = self.create_subscription(Float64MultiArray, 'vehicle_state', self.state_callback, 10)
        self.torque_pub = self.create_publisher(Float64, 'cntl_torque', 10)
        self.dt = 1.0 / self.get_parameter('frequency').value # period 
        self.timer = self.create_timer(self.dt, self.control_loop_callback)
        self.get_logger().info("Longitudinal Adaptive Controller Node Initialized.")
    
    def state_callback(self, msg):
        self.state = msg.data

    def control_loop_callback(self):
        # NOTE: In production, these inputs should be fetched dynamically 
        v = self.state[3]
        v_des = 15.0  # Current velocity (m/s)
        v_des_dot = 0.0   # Target acceleration (m/s^2)
        
        # --- Step 1: Calculate Velocity Error ---
        e_v = v - v_des
        
        # --- Step 2: Calculate Tau (Intermediate Control Action) ---
        tau = (-self.k_1 * e_v 
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

        self.beta_hat = max(min(self.beta_hat, 0.0), -0.01)
        self.delta_hat = max(min(self.delta_hat, 0.0), -5.0)               

        # --- Step 5: Calculate Final Torque ---
        torque = self.alpha_bar_hat * tau

        MAX_TORQUE = 250.0
        torque = max(min(torque, MAX_TORQUE), -MAX_TORQUE)

        # v_dot = (self.true_alpha * torque) + (self.true_beta * (v **2)) + self.true_delta
        # self.v += v_dot * self.dt
        
        msg = Float64()
        msg.data = torque
        self.torque_pub.publish(msg)
        # self.get_logger().info(
        #     f"e_v: {e_v:.2f} | current_v: {self.state[3]} | Torque: {torque:.2f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = LongitudinalSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down controller node cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
