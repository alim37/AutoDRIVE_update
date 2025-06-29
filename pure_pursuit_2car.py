#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import numpy as np
import math

def catmull_rom_chain(pts, num_points=100):
    """
    Build a Catmull–Rom spline through the list of 2D points pts,
    then sample `num_points` uniformly along it.
    Returns an array shape (num_points, 2).
    """
    # Pad endpoints so first/last segments interpolate correctly
    p = np.vstack([pts[0], pts, pts[-1]])
    # Parameter t for each original point
    t = np.linspace(0, 1, len(p))
    # Interpolate x and y separately
    from scipy.interpolate import CubicSpline
    cs_x = CubicSpline(t, p[:,0], bc_type='clamped')
    cs_y = CubicSpline(t, p[:,1], bc_type='clamped')
    ts = np.linspace(0, 1, num_points)
    return np.vstack([cs_x(ts), cs_y(ts)]).T

class DualPurePursuitIPSOnly(Node):
    def __init__(self):
        super().__init__('dual_pure_pursuit_ips')

        # === Original waypoints ===
        raw1 = np.array([
            [0.25, -5.25], [-0.61, -6.57], [-1.92, -6.81], [-2.79, -5.67],
            [-1.48, -2.55], [-2.25, 3.66], [-1.59, 5.02], [-0.15,  5.08], [0.55,3.94]
        ])
        raw2 = np.array([
            [-1.48, -2.55], [-2.25, 3.66], [-1.59, 5.02], [-0.15,  5.08], [0.55,3.94],
            [0.25, -5.25], [-0.61, -6.57], [-1.92, -6.81], [-2.79, -5.67]
        ])

        # === Build and sample splines ===
        self.spline_car1 = catmull_rom_chain(raw1, num_points=200)
        self.spline_car2 = catmull_rom_chain(raw2, num_points=200)
        self.idx1 = 0
        self.idx2 = 0

        # Pure Pursuit params
        self.lookahead_distance   = 1.0
        self.wheelbase            = 0.3
        self.max_steering_angle   = math.radians(90)
        self.target_speed         = 0.075

        # Publishers
        self.steer_pub1    = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.throttle_pub1 = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.steer_pub2    = self.create_publisher(Float32, '/autodrive/f1tenth_2/steering_command', 10)
        self.throttle_pub2 = self.create_publisher(Float32, '/autodrive/f1tenth_2/throttle_command', 10)

        # IPS state
        self.prev1 = None; self.pos1 = None
        self.prev2 = None; self.pos2 = None

        self.create_subscription(Point, '/autodrive/f1tenth_1/ips', self.ips_cb1, 10)
        self.create_subscription(Point, '/autodrive/f1tenth_2/ips', self.ips_cb2, 10)

        # Control loop
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('DualPurePursuitIPSOnly with spline initialized')

    def ips_cb1(self, msg):
        self.prev1 = self.pos1
        self.pos1  = (msg.x, msg.y)

    def ips_cb2(self, msg):
        self.prev2 = self.pos2
        self.pos2  = (msg.x, msg.y)

    def control_loop(self):
        for car in (1, 2):
            pos   = getattr(self, f'pos{car}')
            prev  = getattr(self, f'prev{car}')
            spline= getattr(self, f'spline_car{car}')
            idx   = getattr(self, f'idx{car}')

            if not pos or not prev:
                self.get_logger().warn(f"Car{car} waiting for IPS")
                continue

            # Pure pursuit on spline
            delta, new_idx = self.pursue(pos, prev, spline, idx)
            setattr(self, f'idx{car}', new_idx)

            # publish
            getattr(self, f'steer_pub{car}').publish(Float32(data=delta))
            getattr(self, f'throttle_pub{car}').publish(Float32(data=self.target_speed))
            self.get_logger().info(f"[Car{car}] steer={delta:.3f}, idx={new_idx}")

    def pursue(self, pos, prev, path, idx):
        x,y = pos; xp,yp = prev
        # yaw
        yaw = math.atan2(y-yp, x-xp)
        # advance idx until point is beyond lookahead distance
        N = len(path)
        while True:
            tx,ty = path[idx % N]
            if math.hypot(tx-x, ty-y) > self.lookahead_distance:
                break
            idx += 1
            if idx>=N: idx=0; break

        # transform to vehicle frame
        dx,dy = tx-x, ty-y
        xv =  dx*math.cos(-yaw) - dy*math.sin(-yaw)
        yv =  dx*math.sin(-yaw) + dy*math.cos(-yaw)

        # standard pure-pursuit
        alpha = math.atan2(yv, xv)
        delta = math.atan2(2*self.wheelbase * math.sin(alpha),
                           self.lookahead_distance)
        # clamp
        return ( max(-self.max_steering_angle, min(self.max_steering_angle, delta)),
                 idx )

def main(args=None):
    rclpy.init(args=args)
    node = DualPurePursuitIPSOnly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
