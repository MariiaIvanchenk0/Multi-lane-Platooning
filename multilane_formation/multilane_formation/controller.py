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
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, HistoryPolicy, qos_profile_sensor_data

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

        self.declare_parameter('alpha', 0.0012)
        self.declare_parameter('beta', -0.0001)
        self.declare_parameter('delta', -0.1)

        # Declare parameters (Lateral)
        self.declare_parameter('k_a1', 1.5)
        self.declare_parameter('k_a2', 3.0)
        self.declare_parameter('l_lane', 0.0)
        self.declare_parameter('R', 20.0)
        self.declare_parameter('wheelbase', 2.5)
        # self.declare_parameter('wheel_radius', 0.035)
        # self.declare_parameter('mass', 2.5)

        # Get parameters (Longitudinal)
        self.k_1 = self.get_parameter('k_1').value
        self.k_2 = self.get_parameter('k_2').value

        self.gamma_alpha = self.get_parameter('gamma_alpha').value
        self.gamma_beta = self.get_parameter('gamma_beta').value
        self.gamma_delta = self.get_parameter('gamma_delta').value

        self.alpha_bar_hat = self.get_parameter('alpha_bar_hat').value
        self.beta_hat = self.get_parameter('beta_hat').value
        self.delta_hat = self.get_parameter('delta_hat').value

        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.delta = self.get_parameter('delta').value

        # Get parameters (Lateral)
        self.k_a1 = self.get_parameter('k_a1').value
        self.k_a2 = self.get_parameter('k_a2').value
        self.l_lane = self.get_parameter('l_lane').value   
        self.R = self.get_parameter('R').value 
        self.L = self.get_parameter('wheelbase').value
        # self.wheel_R = self.get_parameter('wheel_radius').value 
        # self.mass = self.get_parameter('mass').value

        self.xc, self.yc = 0.0, 0.0
        self.prev_x, self.prev_y = None, None
        self.last_pose_stamp = None
        self.last_control_time = None
        self.current_v = 0.0
        self.prev_v_des = 0.0
        self.v_des = 0.0
        self.l_des = 0.0
        self.omega = 0.0          # Accumulated velocity error state
        self.state = [0.0, 0.0, 0.0, 0.0]
        self.dt = 1.0 / self.get_parameter('frequency').value # nominal period, used as fallback only

        qos_profile = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)

        # Subscriptions & Publisher & Timer
        self.kinematic_sub = self.create_subscription(Float64MultiArray, 'kinematic_input', self.kinematic_callback, qos_profile_sensor_data)
        # self.state_sub = self.create_subscription(Float64MultiArray, 'vehicle_state', self.state_callback, qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(PoseStamped, 'pose', self.pose_callback, qos_profile_sensor_data)

        # self.control_pub = self.create_publisher(Float64MultiArray, 'cntl_vector', qos_profile_sensor_data)
        self.raw_cmd_pub = self.create_publisher(Twist, 'raw_cmd_vel', qos_profile_sensor_data)

        self.timer = self.create_timer(self.dt, self.control_loop_callback)

        # Startup readout of the effective (post-parameter) config. If any of
        # these are wrong, params.yaml is not being applied to this node.
        self.get_logger().info(
            f"[EFFECTIVE PARAMS] k_1={self.k_1} k_2={self.k_2} "
            f"k_a1={self.k_a1} k_a2={self.k_a2} alpha={self.alpha} "
            f"alpha_bar_hat={self.alpha_bar_hat} R={self.R} L={self.L}"
        )

    # def state_callback(self, msg):
    #     self.state = msg.data
    
    def kinematic_callback(self, msg):
        self.v_des = msg.data[0]
        self.l_des = msg.data[1]

    def pose_callback(self, msg):
        "Recieve the current pose of the vehicle and convert pose to Frenet coordinates (state)."
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation

        # 1. Heading Error (psi)
        theta = quaternion_to_yaw(q)
        theta_center = math.atan2(y - self.yc, x - self.xc)
        theta_r = theta_center + (math.pi / 2.0)
        psi = normalize_angle(theta - theta_r)

        # 2. Arc Length (s)
        theta_pos = theta_center % (2.0 * math.pi)  # Wrap to [0, 2pi)
        s = self.R * theta_pos

        # 3. Lateral error (l)
        dist_from_center = math.hypot(x - self.xc, y - self.yc)
        l = dist_from_center - self.R

        # 4. Linear velocity (v) -- uses the actual elapsed time between pose
        # messages (from the message timestamp), not the nominal control-loop
        # dt, since the pose publish rate can jitter relative to that nominal
        # period.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = 0.0
        if self.prev_x is not None and self.prev_y is not None and self.last_pose_stamp is not None:
            dt_pose = stamp - self.last_pose_stamp
            if dt_pose > 0.0:
                dist_moved = math.hypot(x - self.prev_x, y - self.prev_y)
                v = dist_moved / dt_pose

        self.prev_x = x
        self.prev_y = y
        self.last_pose_stamp = stamp

        self.state = [s, l, psi, v]


    def control_loop_callback(self):
        # Use the actual elapsed wall/ROS time since the last call, not the
        # nominal 1/frequency period, since ROS timers can jitter under load.
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_control_time is None:
            dt = self.dt
        else:
            dt = now - self.last_control_time
            if dt <= 0.0:
                dt = self.dt
        self.last_control_time = now

        # Longitudinal Controller
        v = self.state[3]
        v_des_dot = (self.v_des - self.prev_v_des) / dt

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
        self.omega += omega_dot * dt
        self.alpha_bar_hat += alpha_bar_hat_dot * dt
        self.beta_hat += beta_hat_dot * dt
        self.delta_hat += delta_hat_dot * dt

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

        # Curvature feed-forward. The paper's feedback law assumes a straight /
        # gently-curved lane, so at zero error it commands zero steering. On a
        # curved path the Ackermann car must hold a steady steering angle
        # atan(L / R_lane) just to stay on the curve, otherwise it drives
        # straight and drifts outward. R_lane is the radius of the lane the
        # robot is tracking (R + l_des); positive => steer left for CCW travel.
        r_lane = self.R + self.l_des
        phi_feedforward = math.atan(self.L / r_lane) if abs(r_lane) > 1e-6 else 0.0

        phi = math.atan(numerator / denominator) + phi_feedforward
        # phi = math.atan2(numerator, denominator)

        # Yahboom R2 Ackermann steering servo maxes out around +/-30 deg, not 45.
        MAX_STEER = math.radians(30.0)
        phi = max(min(phi, MAX_STEER), -MAX_STEER)
        
        # self.get_logger().info(f"phi: {phi}, l_des: {self.l_des}")

        # # Publishing data (cntr_vector)
        # msg = Float64MultiArray()
        # msg.data = [torque, phi]
        # self.control_pub.publish(msg)

        # Publishing data (raw_cmd_vel)
        msg = Twist()
        w = self.convert_to_twist(torque, phi, dt)
        msg.linear.x = self.current_v
        msg.angular.z = w
        self.raw_cmd_pub.publish(msg)

        # Throttled runtime readout of the tracking signals.
        self.get_logger().info(
            f"v={v:.3f} v_des={self.v_des:.3f} e_v={e_v:.3f} "
            f"l={l:.3f} l_des={self.l_des:.3f} psi={psi:.3f} "
            f"torque={torque:.1f} phi={phi:.3f} -> cmd v={self.current_v:.3f} w={w:.3f}",
            throttle_duration_sec=1.0,
        )

    def convert_to_twist(self, torque, steering_angle, dt):
        # 1. Torque -> Acceleration -> Linear Velocity (v_x)
        acceleration = self.alpha * torque + self.beta * (self.current_v ** 2) + self.delta
        self.current_v += acceleration * dt

        MAX_V = 1.0
        MIN_V = 0.0
        self.current_v = max(min(self.current_v, MAX_V), MIN_V)

        # 2. Steering Angle (phi) -> Angular Velocity (omega_z)
        if abs(self.L) > 1e-5:
            omega_z = (self.current_v / self.L) * math.tan(steering_angle)
        else:
            omega_z = 0.0

        return float(omega_z)

def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

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
