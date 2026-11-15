#!/usr/bin/env python3
"""Drive the TA2 chassis to a named waypoint using wheel odometry. Runs in the official B70 client container.

  goto.py --to P3            dry run: print the plan, publish nothing
  goto.py --to P3 --act      really drive (a person with the e-stop must stand by)
  goto.py --reset P5         tell the program "the robot is parked on this waypoint now" (no motion)
  goto.py --where            print the pose the program believes

Frame: origin = P5 (yellow parking mark); right = along the wall towards the red zone; forward = towards the wall.
The robot always faces the wall: this program only translates and holds heading.
Route between zones: back off to the safe line first, slide sideways, then go forward. Never sideways close to a table.
Safety: speed cap, 20 Hz closed loop on the wheel encoders, axis/sign probe before the first real move,
stale-encoder stop, time limit, zero velocity on exit. The robot itself drops the target 1 s after the last command.
"""
import argparse, json, os, pathlib, sys, time
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
CFG = json.load(open(HERE / "waypoints.json"))
J = np.array(CFG["J_wheel_from_cmd"]); JINV = np.linalg.inv(J); WRAP = float(CFG["encoder_wrap_rad"])
WP = {k: np.array([v["right_cm"], v["forward_cm"]], float) / 100 for k, v in CFG["waypoints"].items()}
STATE = pathlib.Path(os.environ.get("NAV_STATE", "/outputs/nav_pose.json"))
SAFE_FWD = 0.20        # m: sideways travel only at or behind this line
VMAX, VMIN, KP = 0.08, 0.015, 0.8
TOL, HOLD_S, RATE = 0.015, 0.5, 20.0
WZ_MAX, K_YAW = 0.15, 1.0


def plan(cur, tgt):
    """Waypoints of the route: retreat -> slide -> advance. Skips legs shorter than 2 cm."""
    legs, p = [], cur.copy()
    if abs(tgt[0] - cur[0]) > 0.02:
        back = min(cur[1], tgt[1], SAFE_FWD)
        for q in ([p[0], back], [tgt[0], back]):
            q = np.array(q, float)
            if np.linalg.norm(q - p) > 0.02: legs.append(q); p = q
    if np.linalg.norm(tgt - p) > 0.02: legs.append(tgt.copy())
    return legs


def load_pose():
    if STATE.exists():
        d = json.load(open(STATE)); return np.array([d["right_m"], d["forward_m"], d["heading_rad"]]), d.get("at", ""), bool(d.get("known", False))
    return None, "", False


def daemon_pose():
    """Pose from odom_daemon.py if it wrote within the last second, else None."""
    try:
        d = json.load(open(STATE))
        if d.get("src") == "odom" and time.time() - d.get("epoch", 0) < 1.0 and (9 if d.get("enc_age") is None else d["enc_age"]) < 0.5:
            return np.array([d["right_m"], d["forward_m"], d["heading_rad"]])
    except Exception:
        pass
    return None


def save_pose(p, note="", force=False):
    if not force and daemon_pose() is not None:
        return                      # the daemon owns the file while it is alive
    STATE.parent.mkdir(parents=True, exist_ok=True)
    json.dump({"src": "goto", "known": True, "epoch": time.time(), "right_m": float(p[0]), "forward_m": float(p[1]), "heading_rad": float(p[2]), "at": time.strftime("%H:%M:%S"), "note": note}, open(STATE, "w"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--to"); ap.add_argument("--reset"); ap.add_argument("--near"); ap.add_argument("--tol-cm", type=float, default=25.0); ap.add_argument("--where", action="store_true"); ap.add_argument("--act", action="store_true")
    a = ap.parse_args()
    pose, at, pose_known = load_pose()
    if a.near:
        if a.near not in WP: print("no such waypoint"); return 2
        dp = daemon_pose()
        if dp is None: print("NOT SURE: the position program is not running or has no fresh wheel data"); return 3
        if not json.load(open(STATE)).get("known", True): print("NOT SURE: position not calibrated - park on the yellow mark and calibrate"); return 3
        d = float(np.linalg.norm(dp[:2] - WP[a.near]) * 100); hd = abs(float(np.degrees(dp[2])))
        ok = d <= a.tol_cm and hd <= 8
        print(f"{'NEAR' if ok else 'FAR FROM'} {a.near}: {d:.0f} cm away, heading off by {hd:.1f} deg (limit {a.tol_cm:.0f} cm / 8 deg)"); return 0 if ok else 1
    if a.where:
        print("unknown: run --reset <waypoint> with the robot parked on it" if pose is None or not pose_known else
              f"right {pose[0]*100:.0f} cm, forward {pose[1]*100:.0f} cm, heading {np.degrees(pose[2]):.1f} deg (saved {at})"); return 0
    if a.reset:
        if a.reset not in WP: print("no such waypoint; have:", ", ".join(WP)); return 2
        save_pose(np.array([*WP[a.reset], 0.0]), f"reset to {a.reset}", force=True); print(f"pose set to {a.reset} {CFG['waypoints'][a.reset]['name']}"); return 0
    if a.to not in WP: print("no such waypoint; have:", ", ".join(f"{k}({v['name']})" for k, v in CFG["waypoints"].items())); return 2
    if pose is None or not pose_known: print("pose unknown: park the robot on a waypoint mark and run --reset <waypoint> first"); return 2
    if abs(float(np.degrees(pose[2]))) > 15:
        print("heading estimate is implausible for the fixed-heading map; recalibrate on a waypoint before driving"); return 2
    legs = plan(pose[:2], WP[a.to])
    print(f"from right {pose[0]*100:.0f} forward {pose[1]*100:.0f} cm  ->  {a.to} {CFG['waypoints'][a.to]['name']} (right {WP[a.to][0]*100:.0f}, forward {WP[a.to][1]*100:.0f})")
    for i, q in enumerate(legs, 1): print(f"  leg {i}: to right {q[0]*100:.0f}, forward {q[1]*100:.0f} cm")
    if not legs: print("already there"); return 0
    if not a.act: print("dry run: nothing published. Add --act to drive."); return 0

    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32MultiArray
    rclpy.init(); node = rclpy.create_node("adam_nav_goto")
    enc = {"p": None, "t": 0.0}
    node.create_subscription(JointState, "/chassis/joint_states", lambda m: enc.update(p=np.array(m.position[:3], float), t=time.monotonic()) if len(m.position) >= 3 else None, 10)
    pub = node.create_publisher(Float32MultiArray, "/api/chassis/velocity", 10)
    send = lambda lat, fwd, wz=0.0: pub.publish(Float32MultiArray(data=[float(lat), float(fwd), float(wz)]))

    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=0.01)

    def stop():
        for _ in range(6): send(0, 0, 0); spin(0.04)

    rc, last = 1, None
    def step_odometry():
        nonlocal last, pose
        if enc["p"] is None: return False
        dp = daemon_pose()
        if dp is not None: pose = dp; last = enc["p"].copy(); return True
        if last is not None:
            d = enc["p"] - last; d -= WRAP * np.round(d / WRAP); b = JINV @ d           # body [right, forward, yaw]
            th = pose[2] + b[2] / 2; c, s = np.cos(th), np.sin(th)
            pose = pose + np.array([c * b[0] - s * b[1], s * b[0] + c * b[1], b[2]])
        last = enc["p"].copy(); return True
    try:
        spin(1.0)
        if not step_odometry(): print("no wheel encoder data (/chassis/joint_states): is the client connected?"); return 3
        # probe: a short forward nudge must show up as forward motion in the odometry, else the axis mapping is wrong
        p0 = pose.copy(); t0 = time.monotonic()
        while time.monotonic() - t0 < 0.6: send(0, 0.03); spin(1 / RATE); step_odometry()
        stop(); spin(0.4); step_odometry(); dw = pose - p0
        c0, s0 = np.cos(p0[2]), np.sin(p0[2]); d = np.array([c0 * dw[0] + s0 * dw[1], -s0 * dw[0] + c0 * dw[1]])   # world -> the robot's own frame
        print(f"probe: commanded forward, robot moved (in its own frame) right {d[0]*100:+.1f} cm forward {d[1]*100:+.1f} cm")
        if d[1] < 0.004 or abs(d[0]) > abs(d[1]): print("PROBE FAILED: axis order or sign differs from the calibration. Stopping."); return 4
        if abs(np.degrees(pose[2])) > 15: print(f"HEADING {np.degrees(pose[2]):.0f} deg: the program believes the robot is not facing the wall. Park on a mark facing the wall and calibrate first. Stopping."); return 7
        for i, q in enumerate(legs, 1):
            t0 = time.monotonic(); held = None; limit = np.linalg.norm(q - pose[:2]) / VMIN + 10
            while True:
                spin(1 / RATE); step_odometry()
                if time.monotonic() - enc["t"] > 0.3: stop(); print("encoder data stale: stopped"); return 5
                if time.monotonic() - t0 > limit: stop(); print("leg time limit: stopped"); return 6
                e = q - pose[:2]; dist = float(np.linalg.norm(e))
                if dist < TOL:
                    send(0, 0, 0); held = held or time.monotonic()
                    if time.monotonic() - held > HOLD_S: break
                    continue
                held = None; sp = float(np.clip(KP * dist, VMIN, VMAX)); w = e / dist * sp
                c, s = np.cos(pose[2]), np.sin(pose[2])                                   # world -> body
                send(c * w[0] + s * w[1], -s * w[0] + c * w[1], float(np.clip(-K_YAW * pose[2], -WZ_MAX, WZ_MAX)))
            print(f"  leg {i} done: right {pose[0]*100:.0f} forward {pose[1]*100:.0f} cm, heading {np.degrees(pose[2]):.1f} deg")
            save_pose(pose, f"after leg {i} towards {a.to}")
        rc = 0; print(f"RESULT: ARRIVED {a.to}")
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        stop(); spin(0.3); step_odometry(); save_pose(pose, f"last goto {a.to} rc={rc}"); node.destroy_node(); rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
