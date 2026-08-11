#!/usr/bin/env python3
"""
Measure the robot's velocity-loop bandwidth 'a' and friction offset 'delta'.

The model being identified is the one the longitudinal controller assumes:

    v_dot = a * (u - v) + d          u = commanded linear.x

  a  = 1 / (time constant of the speed response)   ->  alpha_bar_hat = 1/a
                                                       beta_hat      = -a
  d  = the offset that makes the robot settle slightly off the command
                                                    ->  delta_hat    = d

Runs on the BASE STATION. Nothing is installed or changed on the robot.

    python3 measure_a.py --robot robot_1 --speed 0.5 --duration 4

The robot drives in a STRAIGHT LINE for --duration seconds. Make sure it has
about (speed * duration + 1) metres of clear space ahead of it, and keep a hand
near Ctrl-C. Publishing goes to raw_cmd_vel by default so the polytope safety
net stays in the loop; use --topic cmd_vel to bypass it.

IMPORTANT: stop the platoon launch first. If controller_node is running it will
fight this script for the same topic.

Run it 2-3 times at different speeds (0.3 / 0.5 / 0.7). If 'a' comes out the
same each time, the first-order model is good and you can trust the number.
"""

import math
import time
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import qos_profile_sensor_data


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StepTest(Node):
    def __init__(self, args):
        super().__init__('measure_a')
        self.args = args
        self.samples = []          # (t_since_start, x, y, yaw)
        self.t0 = None
        self.done = False

        self.pub = self.create_publisher(Twist, f'/{args.robot}/{args.topic}', 10)
        self.create_subscription(PoseStamped, f'/{args.robot}/pose',
                                 self.pose_cb, qos_profile_sensor_data)
        self.create_timer(1.0 / 20.0, self.tick)
        self.get_logger().info(
            f"stepping /{args.robot}/{args.topic} to {args.speed} m/s "
            f"for {args.duration}s -- clear space needed: "
            f"~{args.speed * args.duration + 1.0:.1f} m")

    def pose_cb(self, msg):
        if self.t0 is None:
            return
        self.samples.append((
            time.time() - self.t0,
            msg.pose.position.x,
            msg.pose.position.y,
            quaternion_to_yaw(msg.pose.orientation),
        ))

    def tick(self):
        if self.done:
            return
        if self.t0 is None:
            self.t0 = time.time()
        t = time.time() - self.t0

        msg = Twist()
        if t < self.args.duration:
            msg.linear.x = float(self.args.speed)
        else:
            self.done = True                    # publish zero and finish
        self.pub.publish(msg)

    def stop(self):
        for _ in range(10):
            self.pub.publish(Twist())
            time.sleep(0.02)


def analyse(samples, u, smooth=0.3):
    """Returns (a, d, v_inf, t63) or None."""
    if len(samples) < 20:
        return None

    # signed speed: project displacement onto the heading, so reverse and
    # sideways motion do not read as forward speed
    vs = []
    for i in range(1, len(samples)):
        t0, x0, y0, _ = samples[i - 1]
        t1, x1, y1, yaw = samples[i]
        dt = t1 - t0
        if dt < 1e-4:
            continue
        v = ((x1 - x0) * math.cos(yaw) + (y1 - y0) * math.sin(yaw)) / dt
        vs.append((t1, v))

    # light low-pass so the 63% crossing is not picked off a noise spike
    filt, acc = [], vs[0][1]
    for t, v in vs:
        acc = (1 - smooth) * acc + smooth * v
        filt.append((t, acc))

    tail = [v for t, v in filt if t > filt[-1][0] * 0.7]
    v_inf = sum(tail) / len(tail)
    if v_inf <= 1e-3:
        return None

    target = 0.632 * v_inf
    t63 = next((t for t, v in filt if v >= target), None)
    if t63 is None or t63 <= 0:
        return None

    a = 1.0 / t63
    d = a * (v_inf - u)          # from v_inf = u + d/a
    return a, d, v_inf, t63, filt


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--robot', default='robot_1')
    p.add_argument('--topic', default='raw_cmd_vel',
                   help='raw_cmd_vel keeps the safety net (default); cmd_vel bypasses it')
    p.add_argument('--speed', type=float, default=0.5)
    p.add_argument('--duration', type=float, default=4.0)
    args = p.parse_args()

    rclpy.init()
    node = StepTest(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
        for _ in range(20):                     # a little tail after the step
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()

    print()
    res = analyse(node.samples, args.speed)
    if res is None:
        print("not enough usable pose data -- did the robot move? is /pose publishing?")
    else:
        a, d, v_inf, t63, filt = res
        print("  t[s]   v[m/s]")
        for t, v in filt[::max(1, len(filt) // 25)]:
            print(f"  {t:5.2f}  {v:6.3f}")
        print(f"\n  commanded u      = {args.speed:.3f} m/s")
        print(f"  settled  v_inf   = {v_inf:.3f} m/s   (gap {v_inf - args.speed:+.3f})")
        print(f"  63% rise time    = {t63:.3f} s")
        print( "  ------------------------------------------")
        print(f"  a     = 1/t63    = {a:.2f}   ->  alpha_bar_hat = {1.0/a:.3f}")
        print(f"                            ->  beta_hat      = {-a:.2f}")
        print(f"  delta = a*(v_inf-u) = {d:+.3f}  ->  delta_hat  = {d:+.3f}")
        print( "  ------------------------------------------")
        print(f"  k_1 must satisfy 0 < k_1 < 2a = {2*a:.1f};  k_1 = {a:.1f} is the fastest stable")
        print(f"  k_2 = a / (integral settling time):  {a/5:.2f} for 5 s, {a/2:.2f} for 2 s")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
