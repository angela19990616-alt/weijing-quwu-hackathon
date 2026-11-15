#!/usr/bin/env python3
"""Offline cruise simulation (no robot, no grasping): the real goto controller driving a simulated TA2 chassis
along the mission route. Checks the route logic, timing and stopping accuracy before anything moves on the floor.

  python sim_cruise.py <out.json> [--slip 0.03] [--yaw-drift-deg-per-m 1.0] [--latency 0.1]

Simulated: velocity commands at 20 Hz, the robot's ramp limits (0.3 m/s^2 up, 0.6 m/s^2 down, from the vendor notes),
command latency, wheel slip (odometry over-reads distance), a slow yaw drift, encoder wrap at 8*pi, encoder noise.
Controller: identical constants and route planner to goto.py (imported from it)."""
import argparse, json, sys, pathlib
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import goto as G

ROUTE = [("P1", 0), ("P2", 2), ("P3", 3), ("P4", 0), ("P5", 12), ("P1", 0), ("P2", 3)]   # (waypoint, seconds to stay there)
DT = 1 / G.RATE


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("--start", default="P0")
    ap.add_argument("--slip", type=float, default=0.03); ap.add_argument("--yaw-drift-deg-per-m", type=float, default=1.0)
    ap.add_argument("--latency", type=float, default=0.10); ap.add_argument("--seed", type=int, default=1); ap.add_argument("--rezero", action="store_true", help="on every arrival at P5 the robot learns its true pose (vision / parking mark)")
    a = ap.parse_args(); rng = np.random.default_rng(a.seed)
    true = np.array([*G.WP[a.start], 0.0]); est = true.copy()          # true pose vs what odometry believes
    vel = np.zeros(3); enc = np.zeros(3); last_enc = enc.copy(); t = 0.0
    queue = []; track = []; events = []; legs_log = []
    drift = np.radians(a.yaw_drift_deg_per_m)

    def step(cmd):
        nonlocal true, est, vel, enc, last_enc, t
        queue.append((t + a.latency, np.array(cmd, float)))
        target = vel.copy()
        while queue and queue[0][0] <= t: target = queue.pop(0)[1]
        if queue and not np.any([q[0] <= t for q in queue]): pass
        step.target = target if (queue == [] or True) else target
        dv = step.target - vel; lim = np.where(np.abs(step.target) > np.abs(vel), 0.3, 0.6) * DT
        vel = vel + np.clip(dv, -lim, lim)
        c, s = np.cos(true[2]), np.sin(true[2]); moved = float(np.hypot(vel[0], vel[1]) * DT)
        true = true + np.array([c * vel[0] - s * vel[1], s * vel[0] + c * vel[1], vel[2]]) * DT + np.array([0, 0, drift * moved])
        wheel = G.J @ (vel * DT * np.array([1 + a.slip, 1 + a.slip, 1.0])) + rng.normal(0, 2e-4, 3)   # encoders over-read when slipping
        enc = (enc + wheel + G.WRAP / 2) % G.WRAP - G.WRAP / 2
        d = enc - last_enc; d -= G.WRAP * np.round(d / G.WRAP); b = G.JINV @ d; last_enc = enc.copy()
        th = est[2] + b[2] / 2; c, s = np.cos(th), np.sin(th)
        est = est + np.array([c * b[0] - s * b[1], s * b[0] + c * b[1], b[2]])
        t += DT
        if int(round(t / DT)) % 4 == 0: track.append([round(t, 2), round(true[0], 4), round(true[1], 4), round(float(np.degrees(true[2])), 2), round(est[0], 4), round(est[1], 4)])
    step.target = np.zeros(3)

    for name, stay in ROUTE:
        legs = G.plan(est[:2], G.WP[name]); t_start = t
        for q in legs:
            held = 0.0; t0 = t; limit = np.linalg.norm(q - est[:2]) / G.VMIN + 10
            while True:
                e = q - est[:2]; dist = float(np.linalg.norm(e))
                if t - t0 > limit: events.append(f"{t:.1f}s TIMEOUT towards {name}"); break
                if dist < G.TOL:
                    step([0, 0, 0]); held += DT
                    if held > G.HOLD_S: break
                    continue
                held = 0.0; sp = float(np.clip(G.KP * dist, G.VMIN, G.VMAX)); w = e / dist * sp
                c, s = np.cos(est[2]), np.sin(est[2])
                step([c * w[0] + s * w[1], -s * w[0] + c * w[1], float(np.clip(-G.K_YAW * est[2], -G.WZ_MAX, G.WZ_MAX))])
        err = float(np.linalg.norm(true[:2] - G.WP[name]) * 100)
        legs_log.append(dict(to=name, name=G.CFG["waypoints"][name]["name"], legs=len(legs), drive_s=round(t - t_start, 1), arrive_t=round(t, 1),
                             true_error_cm=round(err, 1), believed_error_cm=round(float(np.linalg.norm(est[:2] - G.WP[name]) * 100), 1),
                             heading_deg=round(float(np.degrees(true[2])), 1), stay_s=stay))
        for _ in range(int(stay / DT)): step([0, 0, 0])
        if name == "P5" and a.rezero:         # the yellow stop doubles as a re-localisation: belief := truth
            est = true.copy(); events.append(f"{t:.1f}s re-zeroed at P5")
    path = float(sum(np.hypot(track[i][1] - track[i - 1][1], track[i][2] - track[i - 1][2]) for i in range(1, len(track))))
    out = dict(params=vars(a), route=[r[0] for r in ROUTE], total_s=round(t, 1), path_m=round(path, 2), vmax_cm_s=G.VMAX * 100,
               max_true_error_cm=max(l["true_error_cm"] for l in legs_log), legs=legs_log, events=events, track=track)
    json.dump(out, open(a.out, "w")); out.pop("track"); print(json.dumps(out, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
