# -*- coding: utf-8 -*-
import csv
import os
import re
import time
from datetime import datetime

from dobot_api import DobotApiDashboard, DobotApiMove


# Leader: CR3-A, dragged by hand.
# Follower: CR3-B, follows leader joint deltas.
LEADER_IP = "192.168.5.1"
FOLLOWER_IP = "192.168.6.1"

DASHBOARD_PORT = 29999
MOVE_PORT = 30003

FOLLOW_INTERVAL = 0.05
SERVO_TIME = 0.05
LOOKAHEAD_TIME = 30
GAIN = 300

LEADER_SPEED = 10
FOLLOWER_SPEED = 10

# Per cycle max joint delta, degree. Keep small for teaching safety.
MAX_DELTA_DEG = 0.5

# Target smoothing. Higher means more responsive, lower means smoother.
SMOOTH_ALPHA = 0.5

# Ignore tiny leader movement noise.
DELTA_DEADBAND = 0.02

# Optional conservative joint limits for follower target.
# Set USE_JOINT_LIMITS = False if you want pure no-limit copying.
USE_JOINT_LIMITS = True
JOINT_LIMITS = [
    (-350.0, 350.0),
    (-107.0, 92.0),
    (-140.0, 140.0),
    (-178.0, 127.0),
    (-178.0, 178.0),
    (-350.0, 350.0),
]

JOINT_SCALE = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

RECORD_DIR = "records"
RECORD_EVERY_N_STEPS = 1


def parse_robot_values(reply):
    match = re.search(r"\{([^}]*)\}", reply)
    if not match:
        raise ValueError(f"Can not parse robot reply: {reply!r}")

    values = [float(item) for item in re.findall(r"[-+]?\d+(?:\.\d+)?", match.group(1))]
    if len(values) < 6:
        raise ValueError(f"Reply has fewer than 6 joint values: {reply!r}")
    return values[:6]


def normalize_angle(angle):
    return ((angle + 180.0) % 360.0) - 180.0


def normalize_joints(joints):
    return [normalize_angle(value) for value in joints]


def shortest_angle_delta(current, previous):
    return normalize_angle(current - previous)


def get_angle(dashboard):
    return normalize_joints(parse_robot_values(dashboard.GetAngle()))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def limit_delta(delta):
    limited = []
    for index, value in enumerate(delta):
        if abs(value) < DELTA_DEADBAND:
            value = 0.0
        value = clamp(value, -MAX_DELTA_DEG, MAX_DELTA_DEG)
        limited.append(value * JOINT_SCALE[index])
    return limited


def clamp_joint_limits(joints):
    if not USE_JOINT_LIMITS:
        return joints
    return [
        clamp(value, lower, upper)
        for value, (lower, upper) in zip(joints, JOINT_LIMITS)
    ]


def smooth_target(previous_target, raw_target):
    return [
        previous + (current - previous) * SMOOTH_ALPHA
        for previous, current in zip(previous_target, raw_target)
    ]


def vector_add(a, b):
    return [x + y for x, y in zip(a, b)]


def vector_sub_delta(current, previous):
    return [
        shortest_angle_delta(current_value, previous_value)
        for current_value, previous_value in zip(current, previous)
    ]


def max_abs(values):
    return max(abs(value) for value in values)


def connect_robot(ip):
    dashboard = DobotApiDashboard(ip, DASHBOARD_PORT)
    move = DobotApiMove(ip, MOVE_PORT)
    dashboard.log = lambda _text: None
    move.log = lambda _text: None
    return dashboard, move


def make_record_writer():
    os.makedirs(RECORD_DIR, exist_ok=True)
    filename = datetime.now().strftime("copy_act_%Y%m%d_%H%M%S.csv")
    path = os.path.join(RECORD_DIR, filename)
    file_obj = open(path, "w", newline="", encoding="utf-8")
    writer = csv.writer(file_obj)

    writer.writerow(
        ["timestamp", "step"]
        + [f"leader_j{i}" for i in range(1, 7)]
        + [f"leader_delta_j{i}" for i in range(1, 7)]
        + [f"action_target_j{i}" for i in range(1, 7)]
        + [f"follower_actual_j{i}" for i in range(1, 7)]
    )
    return path, file_obj, writer


def record_step(writer, step, leader_joints, leader_delta, action_target, follower_actual):
    writer.writerow(
        [time.time(), step]
        + leader_joints
        + leader_delta
        + action_target
        + follower_actual
    )


def main():
    print("Connecting leader and follower arms...")
    leader_dashboard, leader_move = connect_robot(LEADER_IP)
    follower_dashboard, follower_move = connect_robot(FOLLOWER_IP)
    leader_drag_started = False
    record_path, record_file, writer = make_record_writer()
    print(f"Recording to: {record_path}")

    try:
        print("Clearing alarms and enabling arms...")
        leader_dashboard.ClearError()
        follower_dashboard.ClearError()
        leader_dashboard.EnableRobot()
        follower_dashboard.EnableRobot()
        leader_dashboard.SpeedFactor(LEADER_SPEED)
        follower_dashboard.SpeedFactor(FOLLOWER_SPEED)

        leader_last = get_angle(leader_dashboard)
        follower_target = get_angle(follower_dashboard)
        follower_target = clamp_joint_limits(follower_target)
        follower_smoothed = follower_target[:]

        print(f"Leader start joints: {leader_last}")
        print(f"Follower start target: {follower_target}")

        print("Starting leader drag mode...")
        start_drag_reply = leader_dashboard.StartDrag()
        leader_drag_started = start_drag_reply.strip().startswith("0,")
        if not leader_drag_started:
            print(f"StartDrag failed: {start_drag_reply.strip()}")
            print("Please enable drag mode manually, then set ENABLE_MASTER_DRAG style workflow if needed.")
            return

        print("Incremental copy started. Press Ctrl+C to stop.")
        step = 0

        while True:
            leader_current = get_angle(leader_dashboard)
            raw_delta = vector_sub_delta(leader_current, leader_last)
            leader_delta = limit_delta(raw_delta)

            raw_target = vector_add(follower_target, leader_delta)
            raw_target = clamp_joint_limits(raw_target)
            follower_smoothed = smooth_target(follower_smoothed, raw_target)

            if max_abs(vector_sub_delta(follower_smoothed, follower_target)) >= DELTA_DEADBAND:
                servo_reply = follower_move.ServoJ(
                    follower_smoothed[0],
                    follower_smoothed[1],
                    follower_smoothed[2],
                    follower_smoothed[3],
                    follower_smoothed[4],
                    follower_smoothed[5],
                    t=SERVO_TIME,
                    lookahead_time=LOOKAHEAD_TIME,
                    gain=GAIN,
                )
                if not servo_reply.strip().startswith("0,"):
                    print(f"ServoJ failed: {servo_reply.strip()}")
                    break
                follower_target = follower_smoothed[:]

            if step % RECORD_EVERY_N_STEPS == 0:
                follower_actual = get_angle(follower_dashboard)
                record_step(writer, step, leader_current, leader_delta, follower_target, follower_actual)

            leader_last = leader_current
            step += 1
            time.sleep(FOLLOW_INTERVAL)

    except KeyboardInterrupt:
        print("\nStop requested, exiting...")
    finally:
        if leader_drag_started:
            try:
                leader_dashboard.StopDrag()
            except Exception as exc:
                print(f"Failed to stop leader drag mode: {exc}")

        try:
            follower_move.Sync()
        except Exception as exc:
            print(f"Failed to sync follower queue: {exc}")

        record_file.close()
        leader_dashboard.close()
        leader_move.close()
        follower_dashboard.close()
        follower_move.close()
        print("Disconnected")


if __name__ == "__main__":
    main()
