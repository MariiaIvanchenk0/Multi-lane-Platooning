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
import random
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, HistoryPolicy
from geometry_msgs.msg import TransformStamped, Quaternion, Point, PoseStamped


class FormationControllerNode(Node):
    def __init__(self):
        super().__init__('formation_controller_node')
        
        # Declare Parameters 
        self.declare_parameter('id', 1)
        self.declare_parameter('neighbor_ids', [1])
        self.declare_parameter('frequency', 20.0)
        self.declare_parameter('namespace', 'agent')  # must match omnisim/safety-net agent names
        self.declare_parameter('k_s', 0.1)
        self.declare_parameter('k_l', 0.1)
        self.declare_parameter('k_n', 0.06)
        self.declare_parameter('v_f', 15.0)
        self.declare_parameter('n_bar', 0.4)
        self.declare_parameter('l0', 0.0)

        self.declare_parameter('R', 10.0)
        self.declare_parameter('wheelbase', 2.5)
        self.declare_parameter('base_frame', 'robot_bs')
        self.declare_parameter('viz_lanes', [0.0])
        
        # Get Parameters
        self.id = self.get_parameter('id').value
        self.neighbor_ids = self.get_parameter('neighbor_ids').value
        self.k_s = self.get_parameter('k_s').value
        self.k_l = self.get_parameter('k_l').value
        self.v_f = self.get_parameter('v_f').value
        self.n_bar = self.get_parameter('n_bar').value
        self.k_n = self.get_parameter('k_n').value
        self.namespace = self.get_parameter('namespace').value

        self.R = self.get_parameter('R').value
        self.L = self.get_parameter('wheelbase').value
        self.base_frame = self.get_parameter('base_frame').value + f"_{self.id}"
        self.viz_lanes = self.get_parameter('viz_lanes').value
        
        self.l_i_des = self.get_parameter('l0').value
        self.deg_i = len(self.neighbor_ids)

        self.xc, self.yc = 0.0, 0.0
        self.prev_x, self.prev_y = None, None
        self.last_pose_stamp = None
        self.last_control_time = None
        self.neighbor_prev_positions = {}
        self.desired_offsets = {
            1: [0.0, 0.0],
            2: [1.2, 0.0],
            # 3: [-10.0, 2.0],
        }
        # self.desired_offsets = {
        #     1: [0.0, 0.0],
        #     2: [20.0, 0.0],
        #     3: [40.0, 0.0],
        #     4: [10.0, -3.4],
        #     5: [30.0, -4.0]
        # }
        self.marker_color_r = random.random()
        self.marker_color_g = random.random()
        self.marker_color_b = random.random()

        self.dt = 1.0 / self.get_parameter('frequency').value # nominal period, used as fallback only
        self.state = [0.0, 0.0, 0.0, 0.0]  # [s, l, psi, v]
        self.neighbor_states = {nid: [0.0, 0.0, 0.0, 0.0] for nid in self.neighbor_ids}

        qos_profile = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST)
        
        # Subscriptions
        self.state_sub = self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.neighbor_subs = []
        neighbor_topics = []
        for nid in self.neighbor_ids:
            topic_name = f'/{self.namespace}_{nid}/pose'
            neighbor_topics.append(topic_name)
            sub = self.create_subscription(
                PoseStamped,
                topic_name,
                lambda msg, nid=nid: self.neighbor_pose_callback(msg, nid),
                10
            )
            self.neighbor_subs.append(sub)

        # Startup readout of the effective (post-parameter) config. If any of
        # these are wrong, params.yaml is not being applied to this node.
        self.get_logger().info(
            f"[EFFECTIVE PARAMS] id={self.id} namespace='{self.namespace}' "
            f"neighbor_ids={self.neighbor_ids} neighbor_topics={neighbor_topics} "
            f"v_f={self.v_f} k_s={self.k_s} k_l={self.k_l} n_bar={self.n_bar} R={self.R}"
        )

        # self.state_sub = self.create_subscription(Float64MultiArray, 'vehicle_state', self.state_callback, 10) 
        # self.neighbor_subs = []
        # for nid in self.neighbor_ids:
        #     topic_name = f'/{self.namespace}_{nid}/vehicle_state'
        #     sub = self.create_subscription(
        #         Float64MultiArray, 
        #         topic_name, 
        #         lambda msg, nid=nid: self.neighbor_state_callback(msg, nid), 
        #         10
        #     )
        #     self.neighbor_subs.append(sub)
        
        # Publisher & Timer
        if self.id == 1:
            self.lane_pub = self.create_publisher(Marker, 'lane_marker', 10)
        self.kinematic_pub = self.create_publisher(Float64MultiArray, 'kinematic_input', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(self.dt, self.control_loop_callback)

    # def state_callback(self, msg):
    #     self.state = msg.data

    def pose_callback(self, msg):
        """Receive the current pose of the vehicle and convert pose to Frenet coordinates (state)."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation

        # 1. Heading Error (psi) - FIXED ORDER
        theta = quaternion_to_yaw(q)
        theta_center = math.atan2(y - self.yc, x - self.xc)  # MUST BE FIRST
        theta_r = theta_center + (math.pi / 2.0)
        psi = normalize_angle(theta - theta_r)

        # 2. Arc Length (s)
        theta_pos = theta_center % (2.0 * math.pi)
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

    def neighbor_pose_callback(self, msg, neighbor_id):
        """Processes neighbor's PoseStamped message and converts it to Frenet state [s, l, psi, v]."""
        x_j = msg.pose.position.x
        y_j = msg.pose.position.y
        q_j = msg.pose.orientation

        # 1. Heading Error (psi_j)
        theta_j = quaternion_to_yaw(q_j)
        theta_center_j = math.atan2(y_j - self.yc, x_j - self.xc)
        theta_r_j = theta_center_j + (math.pi / 2.0)
        psi_j = normalize_angle(theta_j - theta_r_j)

        # 2. Arc Length (s_j)
        theta_pos_j = theta_center_j % (2.0 * math.pi)
        s_j = self.R * theta_pos_j

        # 3. Lateral error (l_j)
        dist_from_center_j = math.hypot(x_j - self.xc, y_j - self.yc)
        l_j = dist_from_center_j - self.R

        # 4. Linear velocity estimation (v_j) -- uses actual elapsed time
        # between this neighbor's pose messages, not the nominal dt.
        stamp_j = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v_j = 0.0
        if neighbor_id not in self.neighbor_prev_positions:
            self.neighbor_prev_positions[neighbor_id] = (None, None, None)

        prev_xj, prev_yj, prev_stamp_j = self.neighbor_prev_positions[neighbor_id]
        if prev_xj is not None and prev_yj is not None and prev_stamp_j is not None:
            dt_j = stamp_j - prev_stamp_j
            if dt_j > 0.0:
                dist_moved = math.hypot(x_j - prev_xj, y_j - prev_yj)
                v_j = dist_moved / dt_j

        # Store updated position for next velocity calculation
        self.neighbor_prev_positions[neighbor_id] = (x_j, y_j, stamp_j)

        # Store neighbor's Frenet state vector
        self.neighbor_states[neighbor_id] = [s_j, l_j, psi_j, v_j]

    # def neighbor_state_callback(self, msg, neighbor_id):
    #     self.neighbor_states[neighbor_id] = msg.data

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

        if self.deg_i == 0:
            return

        s_i = self.state[0]
        l_i = self.state[1]

        # Circumference of the reference circle. Arc length s wraps from 2*pi*R
        # back to 0 each lap, so a raw (s_j - s_i) jumps by +/-circumference when
        # two robots straddle the theta=0 seam. Wrapping the difference to the
        # shortest signed arc keeps the relative gap continuous across the seam.
        circumference = 2.0 * math.pi * self.R

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

            # Relative distance states (Eq. 4 variant from paper framework).
            # d_ji_s is wrapped to [-circumference/2, circumference/2] (shortest
            # signed arc) so it does not jump when robots cross the theta=0 seam.
            d_ji_s = s_j - s_i + n_ji_s
            d_ji_s = (d_ji_s + circumference / 2.0) % circumference - circumference / 2.0
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

        ego_offset_s, ego_offset_l = self.desired_offsets.get(self.id, [0.0, 0.0])
        
        for nid in self.neighbor_ids:
            # Fetch target formation offset from configuration matrix
            # D_ji_s, D_ji_l = self.desired_offsets.get(nid, [0.0, 0.0])

            neighbor_offset_s, neighbor_offset_l = self.desired_offsets.get(nid, [0.0, 0.0])
            D_ji_s = neighbor_offset_s - ego_offset_s
            D_ji_l = neighbor_offset_l - ego_offset_l

            total_error_s += w_s[nid] * (d_ji_s_dict[nid] - D_ji_s)  #(D_ji_s - d_ji_s_dict[nid]) #
            total_error_l += w_l[nid] * (d_ji_l_dict[nid] - D_ji_l)  #(D_ji_l - d_ji_l_dict[nid]) # 

        # --- Step 4: Apply Noise Filter & Generate Kinematic Velocity Outputs ---
        filtered_error_s = self.smooth_threshold_function(total_error_s)
        filtered_error_l = self.smooth_threshold_function(total_error_l)
        
        u_is = self.k_s * filtered_error_s + self.v_f
        u_il = self.k_l * filtered_error_l

        U_MIN = 0.1
        U_MAX = 0.6
        u_is = max(min(u_is, U_MAX), U_MIN)
        # u_il = max(min(u_il, 5.0), -5.0)

        # --- Step 4.1: Road Adaptation ---
        v_i_des = u_is
        self.l_i_des += u_il * dt

        # Anti-windup on the road-adaptation integrator. Without this the desired
        # lateral position drifts unbounded (l_des was reaching 1.7+ on a 1m
        # track), pushing the robot off the circle until the controller diverges.
        # Keep it within the reachable lane band around the centre.
        L_DES_MIN = -0.5
        L_DES_MAX = 0.5
        self.l_i_des = max(min(self.l_i_des, L_DES_MAX), L_DES_MIN)
        
        # --- Step 5: Publish Control Vector ---
        input_msg = Float64MultiArray()
        input_msg.data = [v_i_des, self.l_i_des]
        self.kinematic_pub.publish(input_msg)

        # Throttled runtime readout. If neighbor s/l stay at 0.0 forever, the
        # neighbor pose subscription is not receiving data (wrong topic).
        neighbor_dump = {nid: [round(self.neighbor_states[nid][0], 3),
                               round(self.neighbor_states[nid][1], 3)]
                         for nid in self.neighbor_ids}
        self.get_logger().info(
            f"id={self.id} self[s,l]=[{s_i:.3f},{l_i:.3f}] "
            f"neighbor[s,l]={neighbor_dump} "
            f"err_s={total_error_s:.3f} err_l={total_error_l:.3f} "
            f"-> v_des={v_i_des:.3f} l_des={self.l_i_des:.3f}",
            throttle_duration_sec=1.0,
        )

        if self.id == 1:
            self.publish_lane_centerline()
        # self.publish_to_sim(self.state)

    def frenet_to_cartesian(self, state):
        """Converts internal Frenet vector safely to Cartesian coordinates"""
        s, l, psi, _ = state # Correctly unpacks from the 4-element state array
        
        theta_r = s / self.R
        x_r = self.R * math.sin(theta_r)
        y_r = self.R - self.R * math.cos(theta_r)

        # Position Projections
        x = x_r + l * math.sin(theta_r)
        y = y_r - l * math.cos(theta_r)
        global_theta = theta_r + psi
        
        return x, y, global_theta


    def publish_lane_centerline(self):
        """Publishes all configured visualization lanes to RViz dynamically."""
        now = self.get_clock().now().to_msg()
        num_points = 200
        
        # Loop through whatever lanes are requested in 'viz_lanes'
        for idx, lane_offset in enumerate(self.viz_lanes):
            marker = Marker()
            marker.header.frame_id = 'world' # "map"
            marker.header.stamp = now
            marker.ns = "environment"
            marker.id = idx + 1  # Dynamic ID ensures they don't overwrite each other
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Style the main centerline (0.0) differently than adjacent lanes
            if math.isclose(lane_offset, 0.0, abs_tol=1e-3):
                marker.scale.x = 0.05
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 0.6  # Green
            else:
                marker.scale.x = 0.05
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.8, 0.8, 0.8, 0.5  # Semitransparent Gray

            # Calculate circle coordinates with the given lane offset.
            # Centred at the origin (grid centre) to match the control path used
            # in pose_callback (theta_center = atan2(y, x), l = dist-from-origin).
            for i in range(num_points + 1):
                theta_r = (2.0 * math.pi / num_points) * i
                p = Point()
                p.x = (self.R + lane_offset) * math.cos(theta_r)
                p.y = (self.R + lane_offset) * math.sin(theta_r)
                p.z = 0.0
                marker.points.append(p)

            self.lane_pub.publish(marker)

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