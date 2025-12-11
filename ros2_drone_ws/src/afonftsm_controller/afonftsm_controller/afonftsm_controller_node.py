#!/usr/bin/env python3
# AFONFTSM-inspired position controller (simplified & practical)
# Publishes velocity setpoints to /mavros/setpoint_velocity/cmd_vel
# Uses GL finite-memory approximation for fractional derivative/integral
#
# Based on: "Adaptive Fractional-Order Nonsingular Fast Terminal Sliding Mode-Based Robust Tracking Control..."
# (Labbadi & Cherkaoui). See provided PDF for formulas/derivation. :contentReference[oaicite:2]{index=2}

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
import time

def frac_binomial_coeffs(gamma, N):
    # returns array c[0..N] for GL approx: D^gamma f(t) ~ sum_{k=0..N} c_k f(t-k*dt)
    # c_k = (-1)^k * C(gamma, k) with C(gamma,k) = gamma(gamma-1)...(gamma-k+1)/k!
    c = np.zeros(N+1, dtype=np.float64)
    c[0] = 1.0
    for k in range(1, N+1):
        c[k] = c[k-1] * ((gamma - (k-1)) / k) * -1.0
    return c

class AFONFTSMController(Node):
    def __init__(self):
        super().__init__('afonftsm_controller')

        # params (tune these)
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('memory_len', 60)   # fractional GL memory (samples)
        self.declare_parameter('gamma_d', 0.9)     # fractional derivative order ~ gamma1
        self.declare_parameter('gamma_i', 0.1)     # fractional integral order ~ gamma2
        self.declare_parameter('c_d', 0.18)        # coefficient for frac derivative term
        self.declare_parameter('c_i', 0.18)        # coefficient for frac integral term

        self.hz = float(self.get_parameter('rate').value)
        self.dt = 1.0 / self.hz
        self.N = int(self.get_parameter('memory_len').value)
        self.gamma_d = float(self.get_parameter('gamma_d').value)
        self.gamma_i = float(self.get_parameter('gamma_i').value)
        self.c_d = float(self.get_parameter('c_d').value)
        self.c_i = float(self.get_parameter('c_i').value)

        # sliding gains (paper uses many; we use practical ones)
        self.k1 = 2.6      # term ~ }x1 in paper
        self.k2 = 0.22     # term ~ }x2 in paper
        self.k_alpha = 1.2  # nonlinear term weight for |S|^{0.5} sgn(S)

        # velocity output saturation
        self.v_max = 2.5   # m/s

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        # publishers/subscribers
        self.vel_pub = self.create_publisher(TwistStamped,
                                             '/mavros/setpoint_velocity/cmd_vel',
                                             qos)
        self.traj_sub = self.create_subscription(PoseStamped,
                                                '/trajectory/desired',
                                                self.traj_cb,
                                                qos)
        # prefer odom because provides velocity; fallback to /mavros/local_position/pose if needed
        self.odom_sub = self.create_subscription(Odometry,
                                                 '/mavros/local_position/odom',
                                                 self.odom_cb,
                                                 qos)

        # buffers for fractional ops (store e1 history)
        self.e1_hist_x = np.zeros(self.N+1)
        self.e1_hist_y = np.zeros(self.N+1)
        self.e1_hist_z = np.zeros(self.N+1)
        self.time_hist = np.zeros(self.N+1)

        # GL coeffs (precompute)
        self.c_d_coeffs = frac_binomial_coeffs(self.gamma_d, self.N)
        self.c_i_coeffs = frac_binomial_coeffs(self.gamma_i, self.N)

        # current states
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.ref_pos = np.zeros(3)
        self.ref_vel = np.zeros(3)  # if trajectory generator publishes velocities, use them; else zeros

        self.last_time = self.get_clock().now().nanoseconds * 1e-9
        self.start_time = time.time()

        # timer loop
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('AFONFTSM controller started (publishing to /mavros/setpoint_velocity/cmd_vel).')

    def traj_cb(self, msg: PoseStamped):
        self.ref_pos[0] = msg.pose.position.x
        self.ref_pos[1] = msg.pose.position.y
        self.ref_pos[2] = msg.pose.position.z
        # ref_vel left zero unless you publish velocities on another topic

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.pos[:] = [p.x, p.y, p.z]
        self.vel[:] = [v.x, v.y, v.z]

    def frac_derivative(self, hist, coeffs):
        # GL fractional derivative approximation: sum_{k=0..N} c_k * f(t - k*dt)
        # hist[0] = most recent sample, hist[1] = previous, ...
        return np.dot(coeffs, hist)

    def frac_integral(self, hist, coeffs):
        # GL fractional integral can be approximated similarly with positive binomial pattern (we reuse coeffs)
        # Simpler: cumulative weighted sum (this is an approximation)
        return np.dot(coeffs, hist)

    def shift_hist(self, hist, new_val):
        hist[1:] = hist[:-1]
        hist[0] = new_val

    def control_loop(self):
        # read states
        tnow = time.time()
        # errors
        e1 = self.pos - self.ref_pos        # shape (3,)
        e2 = self.vel - self.ref_vel

        # update histories (store e1 values for fractional ops)
        self.shift_hist(self.e1_hist_x, e1[0])
        self.shift_hist(self.e1_hist_y, e1[1])
        self.shift_hist(self.e1_hist_z, e1[2])

        # fractional terms
        D_fx = self.frac_derivative(self.e1_hist_x, self.c_d_coeffs) * (1.0 / (self.dt ** self.gamma_d + 1e-12))
        D_fy = self.frac_derivative(self.e1_hist_y, self.c_d_coeffs) * (1.0 / (self.dt ** self.gamma_d + 1e-12))
        D_fz = self.frac_derivative(self.e1_hist_z, self.c_d_coeffs) * (1.0 / (self.dt ** self.gamma_d + 1e-12))

        I_fx = self.frac_integral(self.e1_hist_x, self.c_i_coeffs) * (self.dt ** self.gamma_i)
        I_fy = self.frac_integral(self.e1_hist_y, self.c_i_coeffs) * (self.dt ** self.gamma_i)
        I_fz = self.frac_integral(self.e1_hist_z, self.c_i_coeffs) * (self.dt ** self.gamma_i)

        # sliding surfaces S = e2 + c_d * D^{...} e1 + c_i * I^{...} e1
        Sx = e2[0] + self.c_d * D_fx + self.c_i * I_fx
        Sy = e2[1] + self.c_d * D_fy + self.c_i * I_fy
        Sz = e2[2] + self.c_d * D_fz + self.c_i * I_fz

        # virtual control (practical simplified form of equation (23) in paper)
        # vx0 = -k_eq * e2_term + desired accel (we approximate desired accel as zero for moving reference)
        # then robust correction terms: -k_alpha*|S|^{0.5} sign(S) - k2*S
        # Note: you can augment with feedforward accelerations if you have them.
        vx0 = - (self.k1 * np.sign(Sx) * np.sqrt(abs(Sx) + 1e-12) + self.k2 * Sx)
        vy0 = - (self.k1 * np.sign(Sy) * np.sqrt(abs(Sy) + 1e-12) + self.k2 * Sy)
        vz0 = - (self.k1 * np.sign(Sz) * np.sqrt(abs(Sz) + 1e-12) + self.k2 * Sz)

        # add a proportional correction to reduce position error (outer-most correction)
        # This term helps move toward the reference position in presence of model mismatch.
        p_corr = 0.8
        vx_cmd = vx0 - p_corr * e1[0]
        vy_cmd = vy0 - p_corr * e1[1]
        vz_cmd = vz0 - p_corr * e1[2]

        # saturation
        vx_cmd = np.clip(vx_cmd, -self.v_max, self.v_max)
        vy_cmd = np.clip(vy_cmd, -self.v_max, self.v_max)
        vz_cmd = np.clip(vz_cmd, -self.v_max, self.v_max)

        # publish TwistStamped
        tmsg = TwistStamped()
        tmsg.header.stamp = self.get_clock().now().to_msg()
        tmsg.twist.linear.x = float(vx_cmd)
        tmsg.twist.linear.y = float(vy_cmd)
        tmsg.twist.linear.z = float(vz_cmd)
        # no angular velocity commands here
        self.vel_pub.publish(tmsg)

        # debug logging occasionally
        if int(time.time() - self.start_time) % 5 == 0:
            self.get_logger().debug(f"ref_pos={self.ref_pos}, pos={self.pos}, vel_cmd={[vx_cmd,vy_cmd,vz_cmd]}")
def main(args=None):
    rclpy.init(args=args)
    node = AFONFTSMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
