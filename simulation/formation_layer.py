"""
Formation Control layer
Based on paper: MeereFidanHeemels2023_IFAC

Step 1: Get position of topological neighbors

Step 2: Calculate kinematic velocity input
    w_ji_s = deg_i * (sum(|d_ji_l|) - |d_ji_l|) / (deg_i - 1) * sum(|d_ji_l|)
    w_ji_l = deg_i * (sum(|d_ji_s|) - |d_ji_s|) / (deg_i - 1) * sum(|d_ji_s|)

    u_is = k_s * T_deg_i * n_bar * (sum(w_ji_s) * (s_j - s_i + n_ji - D_ji)) + v_f
    u_il = k_l * T_deg_i * n_bar * (sum(w_ji_l) * (l_j - l_i + n_ji - D_ji))

"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class FormationControllerNode(Node):
    def __init__(self):
        super().__init__('formation_controller_node')
        
        # Declare Parameters 
        self.declare_parameter('id', 1)
        self.declare_parameter('neighbor_ids', [1])
        self.declare_parameter('frequency', 20.0)
        self.declare_parameter('namespace', 'robot')
        self.declare_parameter('k_s', 0.6)
        self.declare_parameter('k_l', 0.1)
        self.declare_parameter('k_n', 0.06)
        self.declare_parameter('v_f', 15.0)
        self.declare_parameter('n_bar', 0.4)
        
        # Get Parameters
        self.id = self.get_parameter('id').value
        self.neighbor_ids = self.get_parameter('neighbor_ids').value
        self.k_s = self.get_parameter('k_s').value
        self.k_l = self.get_parameter('k_l').value
        self.v_f = self.get_parameter('v_f').value
        self.n_bar = self.get_parameter('n_bar').value
        self.k_n = self.get_parameter('k_n').value
        self.namespace = self.get_parameter('namespace').value
        
        self.deg_i = len(self.neighbor_ids)
        self.desired_offsets = {
            1: [0.0, 0.0],
            2: [50.0, 0.0],
            # 3: [50.0, 0.0],
            # 4: [10.0, -3.4],
            # 5: [45.0, -4.0]
        }

        self.dt = 1.0 / self.get_parameter('frequency').value # period
        self.state = [0.0, 0.0, 0.0, 0.0]  # [s, l, psi, v]
        self.neighbor_states = {nid: [0.0, 0.0, 0.0, 0.0] for nid in self.neighbor_ids}
        
        # Subscriptions
        self.state_sub = self.create_subscription(Float64MultiArray, 'vehicle_state', self.state_callback, 10) 
        self.neighbor_subs = []
        for nid in self.neighbor_ids:
            topic_name = f'/{self.namespace}_{nid}/vehicle_state'
            sub = self.create_subscription(
                Float64MultiArray, 
                topic_name, 
                lambda msg, nid=nid: self.neighbor_state_callback(msg, nid), 
                10
            )
            self.neighbor_subs.append(sub)
        
        # Publisher & Timer
        self.kinematic_pub = self.create_publisher(Float64MultiArray, 'kinematic_input', 10)
        self.timer = self.create_timer(self.dt, self.control_loop_callback)
        # self.get_logger().info(f"Formation Controller Layer Initialized for Robot {self.id} (deg: {self.deg_i}).")

    def state_callback(self, msg):
        self.state = msg.data

    def neighbor_state_callback(self, msg, neighbor_id):
        self.neighbor_states[neighbor_id] = msg.data

    def smooth_threshold_function(self, x):
        """Implements the robust smooth deadzone filter T_n(x) from Equation 21."""
        abs_x = abs(x)
        threshold_bound = self.deg_i * self.n_bar
        
        # Zone 1: Error sits entirely inside the noise floor -> block it
        if abs_x <= threshold_bound:
            return 0.0
            
        # Zone 2: Error exceeds the noise floor + smooth buffer -> pass it fully linear
        elif abs_x > (threshold_bound + self.k_n):
            return x
            
        # Zone 3: Smooth transition ramp mapping the gap between deadzone and linear behavior
        else:
            sgn = 1.0 if x >= 0 else -1.0
            numerator = (abs_x - threshold_bound) * sgn
            return (numerator / self.k_n) * abs_x

    def control_loop_callback(self):
        if self.deg_i == 0:
            return

        s_i = self.state[0]
        l_i = self.state[1]
        
        # --- Step 1: Calculate Gaps & Absolute Metric Sums for Weights ---
        sum_abs_d_l = 0.0
        sum_abs_d_s = 0.0
        
        # Temporary storage for calculated distances to avoid double computation
        d_ji_s_dict = {}
        d_ji_l_dict = {}
        
        for nid in self.neighbor_ids:
            s_j = self.neighbor_states[nid][0]
            l_j = self.neighbor_states[nid][1]
            
            # Simulated measurement noise (set to 0.0 for initial clean testing phase)
            n_ji_s = 0.0
            n_ji_l = 0.0
            
            # Relative distance states (Eq. 4 variant from paper framework)
            d_ji_s = s_j - s_i + n_ji_s
            d_ji_l = l_j - l_i + n_ji_l
            
            d_ji_s_dict[nid] = d_ji_s
            d_ji_l_dict[nid] = d_ji_l
            
            sum_abs_d_s += abs(d_ji_s)
            sum_abs_d_l += abs(d_ji_l)

        # --- Step 2: Calculate Direction-Aware Dynamic Weights (Eq. 26 & 27) ---
        w_s = {}
        w_l = {}
        
        for nid in self.neighbor_ids:
            # Protection constraint: If the vehicle has only 1 neighbor, the denominator 
            # (deg_i - 1) equals zero. We safely assign an even weight distribution of 1.0.
            if self.deg_i == 1:
                w_s[nid] = 1.0
                w_l[nid] = 1.0
            else:
                # Protect against division by zero if total sum of deviations drops to absolute 0
                denom_s = (self.deg_i - 1) * sum_abs_d_s
                denom_l = (self.deg_i - 1) * sum_abs_d_l
                
                w_s[nid] = (self.deg_i * (sum_abs_d_l - abs(d_ji_l_dict[nid]))) / denom_l if denom_l > 1e-6 else 1.0
                w_l[nid] = (self.deg_i * (sum_abs_d_s - abs(d_ji_s_dict[nid]))) / denom_s if denom_s > 1e-6 else 1.0

        # --- Step 3: Compute Summed Weighted Position Errors ---
        total_error_s = 0.0
        total_error_l = 0.0

        # ego_offset_s, ego_offset_l = self.desired_offsets.get(self.id, [0.0, 0.0])
        
        for nid in self.neighbor_ids:
            # Fetch target formation offset from configuration matrix
            D_ji_s, D_ji_l = self.desired_offsets.get(nid, [0.0, 0.0])

            # neighbor_offset_s, neighbor_offset_l = self.desired_offsets.get(nid, [0.0, 0.0])
            # D_ji_s = neighbor_offset_s - ego_offset_s
            # D_ji_l = neighbor_offset_l - ego_offset_l

            total_error_s += w_s[nid] * (d_ji_s_dict[nid] - D_ji_s)  #(D_ji_s - d_ji_s_dict[nid]) #
            total_error_l += w_l[nid] * (d_ji_l_dict[nid] - D_ji_l)  #(D_ji_l - d_ji_l_dict[nid]) # 

        # --- Step 4: Apply Noise Filter & Generate Kinematic Velocity Outputs ---
        filtered_error_s = self.smooth_threshold_function(total_error_s)
        filtered_error_l = self.smooth_threshold_function(total_error_l)
        
        u_is = self.k_s * filtered_error_s + self.v_f
        u_il = self.k_l * filtered_error_l

        U_MIN = 10.0
        U_MAX = 40.0
        u_is = max(min(u_is, U_MAX), U_MIN)
        # u_il = max(min(u_il, 5.0), -5.0)
        
        # --- Step 5: Publish Control Vector ---
        input_msg = Float64MultiArray()
        input_msg.data = [u_is, u_il]
        self.kinematic_pub.publish(input_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FormationControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()