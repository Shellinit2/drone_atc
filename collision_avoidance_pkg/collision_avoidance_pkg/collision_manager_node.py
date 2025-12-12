import time

import numpy as np
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node

from px4_msgs.msg import TrajectorySetpoint

TAKEOFF_ALTITUDE = 5.0
CRUISE_SPEED = 1.0
TAKEOFF_DURATION = TAKEOFF_ALTITUDE / CRUISE_SPEED
MIN_SAFE_DISTANCE = 1.5
DELAY_STEP = 0.1
MAX_DELAY = 20.0


def calculate_3d_path(p_2d, q_2d):
    p_start = np.array([p_2d[0], p_2d[1], 0.0])
    p_lift = np.array([p_2d[0], p_2d[1], TAKEOFF_ALTITUDE])
    q_end = np.array([q_2d[0], q_2d[1], TAKEOFF_ALTITUDE])

    dist = np.linalg.norm(q_2d - p_2d)
    cruise_time = dist / CRUISE_SPEED
    total_time = TAKEOFF_DURATION + cruise_time

    return p_start, p_lift, q_end, total_time, cruise_time


def position_3d(p_start, p_lift, q_end, time_now, start_t, cruise_duration):
    if time_now < start_t:
        return p_start

    t = time_now - start_t

    # Takeoff
    if t <= TAKEOFF_DURATION:
        u = t / TAKEOFF_DURATION
        return p_start * (1 - u) + p_lift * u

    # Cruise
    t -= TAKEOFF_DURATION
    if t >= cruise_duration:
        return q_end

    u = t / cruise_duration
    xy = p_lift[:2] * (1 - u) + q_end[:2] * u
    return np.array([xy[0], xy[1], TAKEOFF_ALTITUDE])


# ==========================
# COLLISION AVOIDANCE LOGIC
# ==========================
def min_delay_to_avoid(dr1, dr2, min_dist):
    p1_s, p1_l, q1_e, d1, t1_cruise = calculate_3d_path(dr1["p"], dr1["q"])
    p2_s, p2_l, q2_e, d2, t2_cruise = calculate_3d_path(dr2["p"], dr2["q"])

    t1 = dr1["start"] + dr1["delay"]
    t2 = dr2["start"] + dr2["delay"]

    delay = 0.0
    dt = 0.05

    while delay <= MAX_DELAY:
        safe = True
        T = max(t1 + d1, t2 + delay + d2) + 1.0
        for t in np.arange(0, T, dt):
            A = position_3d(p1_s, p1_l, q1_e, t, t1, t1_cruise)
            B = position_3d(p2_s, p2_l, q2_e, t, t2 + delay, t2_cruise)
            if np.linalg.norm(A - B) < min_dist:
                safe = False
                break
        if safe:
            return delay
        delay += DELAY_STEP
    return MAX_DELAY


def resolve_all_conflicts(drones):
    changed = True
    N = len(drones)
    while changed:
        changed = False
        for i in range(N):
            for j in range(i + 1, N):
                d1 = drones[i]
                d2 = drones[j]
                delay = min_delay_to_avoid(d1, d2, MIN_SAFE_DISTANCE)
                if delay > 0:
                    if d1["start"] <= d2["start"]:
                        d2["delay"] += delay
                    else:
                        d1["delay"] += delay

                    changed = True

    return drones


class CollisionManager(Node):
    def __init__(self):
        super().__init__("collision_manager")

        self.drones = [
            {
                "id": 0,
                "p": np.array([0, 0]),
                "q": np.array([5, 0]),
                "start": 0.0,
                "delay": 0.0,
            },
            {
                "id": 1,
                "p": np.array([0, 5]),
                "q": np.array([5, 5]),
                "start": 0.0,
                "delay": 0.0,
            },
            # Add more drones if needed
        ]

        # Create publishers dynamically for each drone
        self.publishers = {}
        for d in self.drones:
            topic = f"drone_{d['id']}_setpoint"
            self.publishers[d["id"]] = self.create_publisher(
                TrajectorySetpoint, topic, 10
            )
            self.get_logger().info(f"Created publisher: {topic}")

        start = time.time()
        resolve_all_conflicts(self.drones)
        end = time.time()

        self.get_logger().info(f"Collision Resolution Time: {end - start:.3f}s")
        for d in self.drones:
            self.get_logger().info(
                f"Drone {d['id']} -> Start={d['start']:.1f}s Delay={d['delay']:.2f}s"
            )

        for d in self.drones:
            p_s, p_l, q_e, d_total, d_cruise = calculate_3d_path(d["p"], d["q"])
            d["p_start"] = p_s
            d["p_lift"] = p_l
            d["q_end"] = q_e
            d["dur_total"] = d_total
            d["dur_cruise"] = d_cruise

        self.sim_time = 0.0
        self.dt = 0.05
        self.max_sim_duration = max(
            d["start"] + d["delay"] + d["dur_total"] for d in self.drones
        )

        self.timer = self.create_timer(self.dt, self.timer_callback)

    # ---------------------------------------------
    # Helper to publish 3D setpoints in PX4 ENU->NED
    # ---------------------------------------------
    def publish_setpoint(self, pub, position_enu):
        msg = TrajectorySetpoint()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)

        msg.position[0] = position_enu[0]
        msg.position[1] = position_enu[1]
        msg.position[2] = -position_enu[2]  # ENU z -> NED z (negative)

        pub.publish(msg)

    def timer_callback(self):
        if self.sim_time > self.max_sim_duration + 3.0:
            return

        for d in self.drones:
            pos = position_3d(
                d["p_start"],
                d["p_lift"],
                d["q_end"],
                self.sim_time,
                d["start"] + d["delay"],
                d["dur_cruise"],
            )

            self.publish_setpoint(self.publishers[d["id"]], pos)

        self.sim_time += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = CollisionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
