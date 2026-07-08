"""
Road Adaptation Layer
Based on paper: MeereFidanHeemels2023_IFAC

Step 1: Get the kinematic input u_i

Step 2: Calculate desired v and l
    v_des = u_is * (1 - k_r * l)
    l_des = u_il * dt + l_des(t - dt)

"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

class RoadAdaptationNode(Node):
    def __init__(self):
        super().__init__('road_adaptation_node')

        # Declare Parameters 
        self.declare_parameter('R', 20.0)
        self.declare_parameter('l0', 0.0)
        self.declare_parameter('frequency', 20.0)

        # Get Parameters
        self.R = self.get_parameter('R').value
        self.dt = 1.0 / self.get_parameter('frequency').value # period
        self.l_i_des = self.get_parameter('l0').value
        
        self.state = [0.0, 0.0, 0.0, 0.0]
        self.kinematic = [0.0, 0.0]

        # Subscriber, Publishers & Timer
        self.kinematic_sub = self.create_subscription(Float64MultiArray, 'kinematic_input', self.kinematic_callback, 10)
        self.state_sub = self.create_subscription(Float64MultiArray, 'vehicle_state', self.state_callback, 10)
        self.v_des_pub = self.create_publisher(Float64, 'v_des', 10)
        self.l_des_pub = self.create_publisher(Float64, 'l_des', 10)

        self.timer = self.create_timer(self.dt, self.control_loop_callback)
        # self.get_logger().info(f"Road Adaptation Layer Initialized.")

    def kinematic_callback(self, msg):
        self.kinematic = msg.data

    def state_callback(self, msg):
        self.state = msg.data

    def control_loop_callback(self):
        kappa_r = 1.0 / self.R
        current_l = self.state[1]
        
        v_i_des = self.kinematic[0] * (1.0 + kappa_r * current_l)
        self.l_i_des += self.kinematic[1] * self.dt
        
        v_msg = Float64()
        l_msg = Float64()
        v_msg.data = v_i_des
        l_msg.data = self.l_i_des
        self.v_des_pub.publish(v_msg)    
        self.l_des_pub.publish(l_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoadAdaptationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()