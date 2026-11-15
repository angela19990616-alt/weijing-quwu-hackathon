#!/usr/bin/env python3
"""Live wheel odometry for the map. Runs inside the B70 client container all the time (no commands are published).
Integrates /chassis/joint_states through the calibrated wheel matrix and writes the pose 5 times a second to
NAV_STATE (default /outputs/nav_pose.json). Works in VR mode and in API mode alike.
Someone else rewriting the file (goto.py --reset) is adopted as the new truth."""
import json, os, pathlib, time
import numpy as np
import rclpy
from sensor_msgs.msg import JointState, Joy
from nav_filter import PoseEKF, encoder_discontinuity

HERE = pathlib.Path(__file__).resolve().parent
CFG = json.load(open(HERE / "waypoints.json"))
JINV = np.linalg.inv(np.array(CFG["J_wheel_from_cmd"])); WRAP = float(CFG["encoder_wrap_rad"])
STATE = pathlib.Path(os.environ.get("NAV_STATE", "/outputs/nav_pose.json"))
MARKS = STATE.with_name("nav_marks.json")     # VR-controller calibration session: written by the control panel, marks appended here


def read():
    try:
        return json.load(open(STATE))
    except Exception:
        return None


def main():
    rclpy.init(); node = rclpy.create_node("adam_nav_odom")
    s = {"last": None, "last_rx": None, "pose": None, "t": 0.0, "known": False, "moved": 0.0,
         "encoder_resets": 0}
    d = read()
    if d: s["pose"] = np.array([d["right_m"], d["forward_m"], d["heading_rad"]]); s["known"] = bool(d.get("known", True))
    else: s["pose"] = np.zeros(3)
    # This competition map is fixed-heading. A large persisted yaw means the
    # previous encoder stream jumped or the pose was never recalibrated.
    if abs(float(np.degrees(s["pose"][2]))) > 15.0:
        s["known"] = False
    ekf = PoseEKF(s["pose"], d.get("covariance") if d else None)

    def cb(m):
        if len(m.position) < 3: return
        p = np.array(m.position[:3], float); now_mono = time.monotonic(); s["t"] = time.time()
        if s["last"] is not None:
            dw = p - s["last"]; dw -= WRAP * np.round(dw / WRAP); b = JINV @ dw
            dt = now_mono - s["last_rx"]
            if encoder_discontinuity(b, dt):
                # A robot/bridge restart resets encoder angles. Never integrate that jump as motion.
                s["known"] = False; s["encoder_resets"] += 1
            else:
                s["pose"] = ekf.predict(b); s["moved"] += float(np.hypot(b[0], b[1]))
        s["last"] = p; s["last_rx"] = now_mono
    node.create_subscription(JointState, "/chassis/joint_states", cb, 50)
    joy = {"prev": None, "x": 0, "y": 0}

    def joy_cb(m):                      # left controller: buttons[2] = X, buttons[3] = Y (same as the recording platform)
        b = list(m.buttons)
        if len(b) < 4: return
        if joy["prev"] is not None:
            if joy["prev"][2] == 0 and b[2] == 1: joy["x"] += 1
            if joy["prev"][3] == 0 and b[3] == 1: joy["y"] += 1
        joy["prev"] = b
    node.create_subscription(Joy, "/xr/left_hand_inputs", joy_cb, 20)

    def handle_marks(speed_cm_s):
        """X = 'the robot is standing on the next mark of the list'; first X of a session = origin. Y = undo the last mark."""
        nx, ny = joy["x"], joy["y"]; joy["x"] = joy["y"] = 0
        if not (nx or ny): return
        try:
            d = json.load(open(MARKS))
        except Exception:
            return
        if not d.get("active"): return
        marks = d.setdefault("marks", []); seq = d.get("seq", [])
        for _ in range(ny):
            if marks: marks.pop()
        for _ in range(nx):
            if speed_cm_s > 1.5:
                d["last_msg"] = time.strftime("%H:%M:%S") + " 车还在动，这次 X 没有记录"; continue
            if len(marks) >= len(seq):
                d["last_msg"] = time.strftime("%H:%M:%S") + " 顺序表已经走完，多按的 X 没有记录"; continue
            label = seq[len(marks)]
            if not marks:                               # first mark of the session defines the origin
                ekf.reset(np.zeros(3)); s["pose"] = ekf.x.copy(); s["known"] = True
            else:
                wp = CFG.get("waypoints", {}).get(label)
                if wp:
                    sigma = max(float(wp.get("spread_cm", 5.0)) / 100.0, 0.02)
                    s["pose"] = ekf.update_waypoint(float(wp["right_cm"]) / 100.0,
                                                    float(wp["forward_cm"]) / 100.0,
                                                    position_std_m=sigma)
                    s["known"] = True
            marks.append({"wp": label, "right_cm": round(float(s["pose"][0]) * 100, 1), "forward_cm": round(float(s["pose"][1]) * 100, 1),
                          "heading_deg": round(float(np.degrees(s["pose"][2])), 1), "t": time.strftime("%H:%M:%S"), "who": "手柄X"})
            d["last_msg"] = time.strftime("%H:%M:%S") + f" 已记录 {label}"
        tmp = MARKS.with_suffix(".tmp"); json.dump(d, open(tmp, "w"), ensure_ascii=False); os.replace(tmp, MARKS)
    wrote = 0.0; my_mtime = 0.0; speed_ref = (time.time(), 0.0); speed = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now - wrote < 0.2: continue
        try:  # adopt an external reset
            mt = STATE.stat().st_mtime
            if my_mtime and abs(mt - my_mtime) > 1e-6:
                d = read()
                if d and d.get("src") != "odom":
                    ekf.reset(np.array([d["right_m"], d["forward_m"], d["heading_rad"]]))
                    s["pose"] = ekf.x.copy(); s["known"] = bool(d.get("known", True))
        except FileNotFoundError:
            pass
        if now - speed_ref[0] >= 1.0: speed = (s["moved"] - speed_ref[1]) / (now - speed_ref[0]); speed_ref = (now, s["moved"])
        handle_marks(speed * 100)
        tmp = STATE.with_suffix(".tmp")
        json.dump({"src": "odom", "right_m": float(s["pose"][0]), "forward_m": float(s["pose"][1]), "heading_rad": float(s["pose"][2]),
                   "known": s["known"], "epoch": now, "enc_age": round(now - s["t"], 2) if s["t"] else None,
                   "speed_cm_s": round(speed * 100, 1), "at": time.strftime("%H:%M:%S"),
                   "covariance": ekf.covariance_flat(), "encoder_resets": s["encoder_resets"]}, open(tmp, "w"))
        os.replace(tmp, STATE); my_mtime = STATE.stat().st_mtime; wrote = now


if __name__ == "__main__":
    main()
