# -*- coding: utf-8 -*-
import re
import time

from dobot_api import DobotApiDashboard, DobotApiFeedBack, DobotApiMove


# 5.1 is the master arm, 6.1 is the slave arm.
MASTER_IP = "192.168.5.1"
SLAVE_IP = "192.168.6.1"

DASHBOARD_PORT = 29999
MOVE_PORT = 30003
FEEDBACK_PORT = 30004

# Follow parameters.
FOLLOW_INTERVAL = 0.02
SERVO_TIME = 0.02
LOOKAHEAD_TIME = 30
GAIN = 500
FOLLOW_SPEED = 10
TARGET_FILTER_ALPHA = 0.75
SKIP_LOG_INTERVAL = 0.5
FEEDBACK_RETRIES = 5

# Ignore tiny joint changes to reduce command jitter. Unit: degree.
JOINT_DEADBAND = 0.02

# Align the slave to the master before follow control starts.
ALIGN_SLAVE_TO_MASTER_ON_START = True
ALIGN_THRESHOLD = 1.0
ALIGN_SPEED = 20

# Per-joint scale/direction. Change a value to -1.0 if a joint moves opposite.
JOINT_SCALE = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

# True: put the master into drag mode, so the slave follows manual movement.
# Set False if the master is already controlled elsewhere.
ENABLE_MASTER_DRAG = True
USE_MASTER_FEEDBACK = True
NORMALIZE_JOINT_ANGLES = True


def parse_robot_values(reply):
    """Parse Dobot replies like: 0,{1,2,3,4,5,6},GetAngle();"""
    match = re.search(r"\{([^}]*)\}", reply)
    if not match:
        raise ValueError(f"Can not parse robot reply: {reply!r}")

    values = [float(item) for item in re.findall(r"[-+]?\d+(?:\.\d+)?", match.group(1))]
    if len(values) < 6:
        raise ValueError(f"Reply has fewer than 6 joint values: {reply!r}")
    return values[:6]


def normalize_angle(angle):
    if not NORMALIZE_JOINT_ANGLES:
        return angle
    return ((angle + 180.0) % 360.0) - 180.0


def normalize_joints(joints):
    return [normalize_angle(value) for value in joints]


def shortest_angle_delta(current, start):
    return normalize_angle(current - start)


def get_angle(dashboard):
    return normalize_joints(parse_robot_values(dashboard.GetAngle()))


def get_feedback_angle(feedback):
    for _ in range(FEEDBACK_RETRIES):
        feed_info = feedback.feedBackData()
        if feed_info is not None and len(feed_info) > 0:
            if hex(feed_info["test_value"][0]) == "0x123456789abcdef":
                return normalize_joints([float(value) for value in feed_info["q_actual"][0]])
        time.sleep(0.005)
    raise ValueError("Invalid feedback packet")


def build_slave_target(master_start, slave_start, master_current):
    target = []
    for index, current in enumerate(master_current):
        delta = shortest_angle_delta(current, master_start[index]) * JOINT_SCALE[index]
        target.append(slave_start[index] + delta)
    return target


def max_joint_delta(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def smooth_target(previous_target, target):
    alpha = TARGET_FILTER_ALPHA
    return [
        previous + (current - previous) * alpha
        for previous, current in zip(previous_target, target)
    ]


def parse_error_ids(error_reply):
    match = re.search(r"\{(.*)\}", error_reply, flags=re.S)
    if not match:
        return []
    return [int(item) for item in re.findall(r"-?\d+", match.group(1)) if int(item) != 0]


def read_error_ids(dashboard):
    return parse_error_ids(dashboard.GetErrorID())


def has_nonzero_error(error_ids):
    return any(error_id != 0 for error_id in error_ids)


def describe_errors(name, error_ids):
    if not error_ids:
        return
    print(f"{name} errors detected: {error_ids}")
    if -2 in error_ids:
        print(f"{name}: error -2 is likely collision/protective stop. Stop follow and handle manually.")


def align_slave_to_master(slave_dashboard, slave_move, master_joints, slave_joints):
    delta = max_joint_delta(master_joints, slave_joints)
    if delta <= ALIGN_THRESHOLD:
        print(f"Arms are already aligned. Max joint delta: {delta:.3f} deg")
        return slave_joints

    print(f"Aligning slave to master. Max joint delta: {delta:.3f} deg")
    slave_dashboard.SpeedFactor(ALIGN_SPEED)
    slave_move.JointMovJ(
        master_joints[0],
        master_joints[1],
        master_joints[2],
        master_joints[3],
        master_joints[4],
        master_joints[5],
    )
    slave_move.Sync()

    aligned_joints = get_angle(slave_dashboard)
    aligned_delta = max_joint_delta(master_joints, aligned_joints)
    print(f"Alignment finished. Max joint delta: {aligned_delta:.3f} deg")
    return aligned_joints


def connect_robot(ip):
    dashboard = DobotApiDashboard(ip, DASHBOARD_PORT)
    move = DobotApiMove(ip, MOVE_PORT)
    return dashboard, move


def main():
    print("Connecting master and slave arms...")
    master_dashboard, master_move = connect_robot(MASTER_IP)
    master_feedback = DobotApiFeedBack(MASTER_IP, FEEDBACK_PORT)
    slave_dashboard, slave_move = connect_robot(SLAVE_IP)
    master_drag_started = False
    print("Connected")

    try:
        print("Clearing alarms and enabling arms...")
        master_dashboard.ClearError()
        master_dashboard.EnableRobot()
        slave_dashboard.ClearError()
        slave_dashboard.EnableRobot()
        slave_dashboard.SpeedFactor(FOLLOW_SPEED)

        master_start = get_angle(master_dashboard)
        slave_start = get_angle(slave_dashboard)

        print(f"Master initial joints: {master_start}")
        print(f"Slave initial joints: {slave_start}")

        if ALIGN_SLAVE_TO_MASTER_ON_START:
            slave_start = align_slave_to_master(
                slave_dashboard,
                slave_move,
                master_start,
                slave_start,
            )

        master_start = get_angle(master_dashboard)
        slave_start = get_angle(slave_dashboard)
        last_target = slave_start[:]

        slave_dashboard.SpeedFactor(FOLLOW_SPEED)

        print(f"Master follow-start joints: {master_start}")
        print(f"Slave follow-start joints: {slave_start}")
        print("Follow started without joint limits or singularity checks. Press Ctrl+C to stop.")

        if ENABLE_MASTER_DRAG:
            print("Starting master drag mode...")
            start_drag_reply = master_dashboard.StartDrag()
            master_drag_started = start_drag_reply.strip().startswith("0,")
            if not master_drag_started:
                print(f"StartDrag failed: {start_drag_reply.strip()}")
                return

        last_skip_log_time = 0.0

        while True:
            master_errors = read_error_ids(master_dashboard)
            slave_errors = read_error_ids(slave_dashboard)
            if has_nonzero_error(master_errors):
                describe_errors("Master", master_errors)
                return
            if has_nonzero_error(slave_errors):
                describe_errors("Slave", slave_errors)
                return

            if USE_MASTER_FEEDBACK:
                try:
                    master_current = get_feedback_angle(master_feedback)
                except ValueError as exc:
                    now = time.time()
                    if now - last_skip_log_time >= SKIP_LOG_INTERVAL:
                        print(f"Feedback invalid, fallback to dashboard GetAngle: {exc}")
                        last_skip_log_time = now
                    master_current = get_angle(master_dashboard)
            else:
                master_current = get_angle(master_dashboard)

            target = build_slave_target(master_start, slave_start, master_current)
            target = smooth_target(last_target, target)

            if max_joint_delta(target, last_target) >= JOINT_DEADBAND:
                servo_reply = slave_move.ServoJ(
                    target[0],
                    target[1],
                    target[2],
                    target[3],
                    target[4],
                    target[5],
                    t=SERVO_TIME,
                    lookahead_time=LOOKAHEAD_TIME,
                    gain=GAIN,
                )
                if not servo_reply.strip().startswith("0,"):
                    print(f"ServoJ failed: {servo_reply.strip()}")
                    describe_errors("Slave", read_error_ids(slave_dashboard))
                    return
                last_target = target

            time.sleep(FOLLOW_INTERVAL)

    except KeyboardInterrupt:
        print("\nStop requested, exiting...")
    finally:
        if master_drag_started:
            try:
                master_dashboard.StopDrag()
            except Exception as exc:
                print(f"Failed to stop master drag mode: {exc}")

        try:
            slave_move.Sync()
        except Exception as exc:
            print(f"Failed to sync slave queue: {exc}")

        master_dashboard.close()
        master_move.close()
        master_feedback.close()
        slave_dashboard.close()
        slave_move.close()
        print("Disconnected")


if __name__ == "__main__":
    main()
