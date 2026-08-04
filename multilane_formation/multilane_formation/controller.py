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
from rclpy.qos import qos_profile_sensor_data

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('R', 1.0)
        self.declare_parameter('frequency', 20.0)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('wheelbase', 0.145)
        
        # Declare parameters (Longitudinal)
        self.declare_parameter('k_1', 1.0)
        self.declare_parameter('k_2', 0.1)
        self.declare_parameter('V_MAX', 1.0)

        self.declare_parameter('alpha', 0.01)
        self.declare_parameter('beta', -1.0)
        self.declare_parameter('delta', 0.0)
        
        self.declare_parameter('alpha_bar_hat', 83.33)    # Adaptive guess for (1 / alpha)
        self.declare_parameter('beta_hat', -0.0001)     # Adaptive guess for aerodynamic drag coefficient
        self.declare_parameter('delta_hat', -0.1)       # Adaptive guess for constant disturbance/friction

        self.declare_parameter('gamma_alpha', 0.001)
        self.declare_parameter('gamma_beta', 0.0001)
        self.declare_parameter('gamma_delta', 0.01)

        # Declare parameters (Lateral)
        self.declare_parameter('k_a1', 0.5)
        self.declare_parameter('k_a2', 0.5)
        self.declare_parameter('ki', 0.5)
        self.declare_parameter('PHI_MAX', 17.0)
        self.declare_parameter('l_lane', 0.0)

        # Get parameters (Longitudinal)
        self.k_1 = self.get_parameter('k_1').value
        self.k_2 = self.get_parameter('k_2').value
        self.V_MAX = self.get_parameter('V_MAX').value
        self.MAX_TORQUE = 150.0 #TODO: convert into parameter
        self.MIN_TORQUE = 0.0   #TODO: convert into parameter

        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.delta = self.get_parameter('delta').value

        # self.alpha_bar_hat = 1.0 / self.alpha
        # self.beta_hat = self.beta
        # self.delta_hat = self.delta

        self.alpha_bar_hat = self.get_parameter('alpha_bar_hat').value
        self.beta_hat = self.get_parameter('beta_hat').value
        self.delta_hat = self.get_parameter('delta_hat').value

        self.gamma_alpha = self.get_parameter('gamma_alpha').value
        self.gamma_beta = self.get_parameter('gamma_beta').value
        self.gamma_delta = self.get_parameter('gamma_delta').value

        # Get parameters (Lateral)
        self.k_a1 = self.get_parameter('k_a1').value
        self.k_a2 = self.get_parameter('k_a2').value
        self.ki = self.get_parameter('ki').value
        self.PHI_MAX = math.radians(self.get_parameter('PHI_MAX').value)
        self.l_lane = self.get_parameter('l_lane').value 

        self.R = self.get_parameter('R').value 
        self.L = self.get_parameter('wheelbase').value
        self.xc, self.yc = self.get_parameter('center_x').value, self.get_parameter('center_y').value
        self.dt = 1.0 / self.get_parameter('frequency').value # nominal period, used as fallback only


        self.prev_x, self.prev_y = None, None
        self.last_pose_stamp = None
        self.last_control_time = None

        self.prev_theta = None
        self.meas_radius = None

        self.prev_v_des = 0.0
        self.l_integral = 0.0

        self.v_des = 0.0
        self.l_des = 0.0
        self.omega = 0.0          # Accumulated velocity error state
        self.state = [0.0, 0.0, 0.0, 0.0]

        # Subscriptions & Publisher & Timer
        self.kinematic_sub = self.create_subscription(Float64MultiArray, 'kinematic_input', self.kinematic_callback, qos_profile_sensor_data)

        self.pose_sub = self.create_subscription(PoseStamped, 'pose', self.pose_callback, qos_profile_sensor_data)
        self.raw_cmd_pub = self.create_publisher(Twist, 'raw_cmd_vel', 10)

        self.timer = self.create_timer(self.dt, self.control_loop_callback)

        # self.get_logger().info(
        #     "[EFFECTIVE PARAMS] "
        #     f"R={self.R}  wheelbase(L)={self.L}  "
        #     f"center=({self.xc:.4f}, {self.yc:.4f})  "
        #     f"k_a1={self.k_a1}  k_a2={self.k_a2}  l_lane={self.l_lane}  "
        #     f"frequency={self.get_parameter('frequency').value}  "
        #     f"expected_ff_steer={math.degrees(math.atan(self.L / self.R)):.2f}deg (at l_des=0)"
        # )

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
       
        # self.get_logger().info(
        #     f"pose x={x:.3f} y={y:.3f}  "
        #     f"theta={math.degrees(theta):.1f}  theta_c={math.degrees(theta_center):.1f}  "
        #     f"theta_r={math.degrees(theta_r):.1f}  psi={math.degrees(psi):.1f}  l={l:.3f}",
        #     throttle_duration_sec=0.3,
        # )

        # 4. Linear velocity (v)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = 0.0
        if self.prev_x is not None and self.prev_y is not None and self.last_pose_stamp is not None:
            dt_pose = stamp - self.last_pose_stamp
            if dt_pose > 0.0:
                dist_moved = math.hypot(x - self.prev_x, y - self.prev_y)
                v = dist_moved / dt_pose

                if self.prev_theta is not None and dist_moved > 1e-4:
                    dtheta = normalize_angle(theta - self.prev_theta)
                    if abs(dtheta) > 1e-4:
                        r_inst = dist_moved / dtheta
                        if self.meas_radius is None:
                            self.meas_radius = r_inst
                        else:
                            self.meas_radius = 0.7 * self.meas_radius + 0.3 * r_inst

        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta
        self.last_pose_stamp = stamp

        self.state = [s, l, psi, v]

    def longitudinal_controller(self):
        v = self.state[3]
        v_des_dot = (self.v_des - self.prev_v_des) / self.dt
        e_v = v - self.v_des

        tau = (- self.k_1 * e_v
               - self.k_2 * self.omega
               - self.beta_hat * (v ** 2)
               - self.delta_hat
               + v_des_dot)

        omega_dot = e_v
        alpha_bar_hat_dot = -self.gamma_alpha * e_v * tau
        beta_hat_dot = self.gamma_beta * (v ** 2) * e_v
        delta_hat_dot = self.gamma_delta * e_v

        torque_raw = self.alpha_bar_hat * tau
        torque = max(min(torque_raw, self.MAX_TORQUE), self.MIN_TORQUE)
        saturated = abs(torque - torque_raw) > 1e-9

        if not saturated:
            # self.omega += e_v * self.dt
            self.omega         += omega_dot * self.dt
            self.alpha_bar_hat += alpha_bar_hat_dot * self.dt
            self.beta_hat      += beta_hat_dot * self.dt
            self.delta_hat     += delta_hat_dot * self.dt
        # self.omega = max(min(self.omega, 20.0), -20.0)
        # self.alpha_bar_hat = max(self.alpha_bar_hat, 1e-4)
        # self.beta_hat  = max(min(self.beta_hat, -1e-6), -0.01)
        # self.delta_hat = max(min(self.delta_hat, 0.0), -5.0)    

        self.prev_v_des = self.v_des
        return torque

    def lateral_controller(self):
        l = self.state[1]
        psi = self.state[2]

        e_psi = -psi
        e_lat = l - self.l_des # self.l_lane - 

        self.l_integral += e_lat * self.dt
        phi_integral = self.ki * self.l_integral
        phi_integral = max(min(phi_integral, self.PHI_MAX), -self.PHI_MAX)
        self.l_integral = max(min(self.l_integral, self.PHI_MAX/self.ki), -self.PHI_MAX/self.ki)

        numerator = -math.cos(e_psi) * e_lat - (self.k_a1 + self.k_a2) * math.sin(e_psi)
        denominator = self.k_a1 - (self.k_a1 + self.k_a2) * math.cos(e_psi) + math.sin(e_psi) * e_lat

        if abs(denominator) < 1e-6:
            denominator = 1e-6 if denominator >= 0 else -1e-6

        r_lane = self.R + self.l_des
        phi_feedforward = math.atan(self.L / r_lane) if abs(r_lane) > 1e-6 else 0.0

        phi = math.atan(numerator / denominator) + phi_feedforward + phi_integral
        # phi = math.atan2(numerator, denominator)

        MAX_STEER = math.radians(30.0)
        phi = max(min(phi, MAX_STEER), -MAX_STEER)
        return phi

    def control_loop_callback(self):
        # Longitudinal
        # torque = self.longitudinal_controller()
        # arg = -(self.alpha * torque + self.delta) / self.beta   # use when frozen
        # arg = -((1.0 / self.alpha_bar_hat) * torque + self.delta_hat) / self.beta_hat   # use when adapting
        # v_cmd = math.sqrt(arg) if arg > 0.0 else 0.0
        # velocity = min(v_cmd, self.V_MAX) 
        velocity = self.v_des

        # self.get_logger().info(
        #     f"LONG v_des={self.v_des:.3f}  v={self.state[3]:.3f}  v_cmd={velocity:.3f}  "
        #     f"T={torque:.2f}  omega={self.omega:.3f}"
        #     f"{'   [omega NOT ~0 -> feed-forward off]' if abs(self.omega) > 0.5 else ''}",
        #     throttle_duration_sec=0.5,
        # )

        # Lateral
        phi = self.lateral_controller()
        angular = (velocity / self.L) * math.tan(phi) if abs(self.L) > 1e-5 else 0.0

        # Dynamic test: Radius/steering diagnostic
        tan_phi = math.tan(phi)
        r_cmd = self.L / tan_phi if abs(tan_phi) > 1e-6 else float('inf')
        r_expected = self.R + self.l_des
        phi_expected = math.atan(self.L / r_expected) if abs(r_expected) > 1e-6 else 0.0
        meas = self.meas_radius if self.meas_radius is not None else float('nan')
        # self.get_logger().info(
        #     f"RADIUS cmd={r_cmd:.3f} meas={meas:.3f} expected={r_expected:.3f}  "
        #     f"phi={math.degrees(phi):.1f}deg phi_ff_expected={math.degrees(phi_expected):.1f}deg",
        #     throttle_duration_sec=0.5,
        # )

        # Send command
        msg = Twist()
        msg.linear.x = velocity
        msg.angular.z = angular
        self.raw_cmd_pub.publish(msg)

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
