import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

TAKEOFF_ALTITUDE = 5.0
CRUISE_SPEED = 1.0
TAKEOFF_DURATION = TAKEOFF_ALTITUDE / CRUISE_SPEED
MIN_DIST = 1.0


def calculate_3d_path(p_2d, q_2d):
    p_start = np.array([p_2d[0], p_2d[1], 0.0])
    p_lift = np.array([p_2d[0], p_2d[1], TAKEOFF_ALTITUDE])
    q_end = np.array([q_2d[0], q_2d[1], TAKEOFF_ALTITUDE])
    travel_dist_2d = np.linalg.norm(q_2d - p_2d)
    travel_duration_2d = travel_dist_2d / CRUISE_SPEED
    total_duration = TAKEOFF_DURATION + travel_duration_2d
    return p_start, p_lift, q_end, total_duration, travel_duration_2d


def position_3d(p_start, p_lift, q_end, t, start_t, travel_duration_2d):
    if t < start_t:
        return p_start
    rel = t - start_t
    if rel <= TAKEOFF_DURATION:
        u = rel / TAKEOFF_DURATION
        return p_start * (1 - u) + p_lift * u
    rel -= TAKEOFF_DURATION
    if rel >= travel_duration_2d:
        return q_end
    u = rel / travel_duration_2d
    xy = p_lift[:2] * (1 - u) + q_end[:2] * u
    return np.array([xy[0], xy[1], TAKEOFF_ALTITUDE])


def min_delay_to_avoid(p1, q1, s1, p2, q2, s2, min_dist):
    p1s, p1l, q1e, d1, t1 = calculate_3d_path(p1, q1)
    p2s, p2l, q2e, d2, t2 = calculate_3d_path(p2, q2)
    delay = 0
    dt = 0.1
    step_delay = 0.1
    max_delay = 20
    while delay <= max_delay:
        safe = True
        T = max(s1 + d1, s2 + delay + d2)
        for t in np.arange(0, T, dt):
            A = position_3d(p1s, p1l, q1e, t, s1, t1)
            B = position_3d(p2s, p2l, q2e, t, s2 + delay, t2)
            if np.linalg.norm(A - B) < min_dist:
                safe = False
                break
        if safe:
            return delay
        delay += step_delay
    return delay


def resolve_all(drones):
    changed = True
    n = len(drones)
    while changed:
        changed = False
        for i in range(n):
            for j in range(i + 1, n):
                d1 = drones[i]
                d2 = drones[j]
                delay = min_delay_to_avoid(
                    d1["p"],
                    d1["q"],
                    d1["start"] + d1["delay"],
                    d2["p"],
                    d2["q"],
                    d2["start"] + d2["delay"],
                    MIN_DIST,
                )
                if delay > 0:
                    if d1["start"] <= d2["start"]:
                        d2["delay"] += delay
                    else:
                        d1["delay"] += delay
                    changed = True
    return drones


def print_collisions_no_delay(drones, dt=0.1):
    print("\n===== POTENTIAL COLLISIONS WITHOUT DELAYS =====")
    for i in range(len(drones)):
        for j in range(i + 1, len(drones)):
            d1 = drones[i]
            d2 = drones[j]
            p1s, p1l, q1e, d1T, d1travel = calculate_3d_path(d1["p"], d1["q"])
            p2s, p2l, q2e, d2T, d2travel = calculate_3d_path(d2["p"], d2["q"])
            Tmax = max(d1["start"] + d1T, d2["start"] + d2T)
            min_dist_found = 999
            min_t = None
            for t in np.arange(0, Tmax, dt):
                A = position_3d(p1s, p1l, q1e, t, d1["start"], d1travel)
                B = position_3d(p2s, p2l, q2e, t, d2["start"], d2travel)
                dist = np.linalg.norm(A - B)
                if dist < min_dist_found:
                    min_dist_found = dist
                    min_t = t
            print(
                f"Drone {i + 1} & Drone {j + 1}: closest={min_dist_found:.2f}m at t={min_t:.2f}s"
                + (" COLLISION" if min_dist_found < MIN_DIST else "")
            )


DRONES = [
    {"p": np.array([0, 0]), "q": np.array([5, 0]), "start": 0.0, "delay": 0.0},
    {"p": np.array([0, 5]), "q": np.array([5, 5]), "start": 0.0, "delay": 0.0},
    {"p": np.array([2, -3]), "q": np.array([2, 8]), "start": 1.0, "delay": 0.0},
    {"p": np.array([-3, 2]), "q": np.array([8, 2]), "start": 0.5, "delay": 0.0},
]

print_collisions_no_delay(DRONES)
DRONES = resolve_all(DRONES)

print("\n===== FINAL DELAYS AFTER COLLISION AVOIDANCE =====")
for i, d in enumerate(DRONES):
    print(f"Drone {i + 1}: delay={d['delay']:.2f}s")

# --- Animation ---
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-5, 10)
ax.set_ylim(-5, 10)
ax.set_zlim(0, TAKEOFF_ALTITUDE + 2)
colors = ["r", "g", "b", "c", "m", "y"]
artists = []
maxT = 0
for i, d in enumerate(DRONES):
    ps, pl, qe, dur, travel = calculate_3d_path(d["p"], d["q"])
    col = colors[i % len(colors)]
    ax.plot(
        [ps[0], pl[0], qe[0]],
        [ps[1], pl[1], qe[1]],
        [ps[2], pl[2], qe[2]],
        col + "--",
        alpha=0.4,
    )
    (point,) = ax.plot(
        [], [], [], col + "o", label=f"Drone {i + 1} (delay={d['delay']:.1f})"
    )
    artists.append((point, d, ps, pl, qe, dur, travel))
    maxT = max(maxT, d["start"] + d["delay"] + dur)
ax.legend()
time_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)


def update(frame):
    t = frame / 30.0
    for point, d, ps, pl, qe, dur, tr in artists:
        pos = position_3d(ps, pl, qe, t, d["start"] + d["delay"], tr)
        point._verts3d = [[pos[0]], [pos[1]], [pos[2]]]
    time_text.set_text(f"t={t:.2f}s")
    return []


ani = FuncAnimation(fig, update, frames=int(maxT * 30), interval=33)
plt.show()
