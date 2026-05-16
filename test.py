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
MAX_STEP_DEG = 0.3
SINGULARITY_STOP_DEG = 1
SINGULARITY_SLOW_ZONE_DEG = 5.0
SKIP_LOG_INTERVAL = 0.5
FEEDBACK_RETRIES = 5
STOP_SLAVE_ON_SAFETY = True

# Ignore tiny joint changes to reduce command jitter. Unit: degree.
JOINT_DEADBAND = 0.02

# Align the slave to the master before follow control starts.
ALIGN_SLAVE_TO_MASTER_ON_START = True
ALIGN_THRESHOLD = 1.0
ALIGN_SPEED = 20

# Per-joint scale/direction. Change a value to -1.0 if a joint moves opposite.
JOINT_SCALE = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

# Joint soft limits. Adjust these to your real safe range before production use.
JOINT_LIMITS = [
    (-350.0, 350.0),
    (-107.0, 92.0),
    (-140.0, 140.0),
    (-178.0, 127.0),
    (-178.0, 178.0),
    (-350.0, 350.0),
]

# True: put the master into drag mode, so the slave follows manual movement.
# Set False if the master is already controlled elsewhere.
ENABLE_MASTER_DRAG = True
USE_MASTER_FEEDBACK = True
NORMALIZE_JOINT_ANGLES = True


class SafetyStop(Exception):
    pass


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


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def clamp_to_joint_limits(joints):
    return [
        clamp(value, lower, upper)
        for value, (lower, upper) in zip(joints, JOINT_LIMITS)
    ]


def joint_limit_violations(joints):
    violations = []
    for index, (value, (lower, upper)) in enumerate(zip(joints, JOINT_LIMITS), start=1):
        if value < lower or value > upper:
            violations.append(f"J{index}={value:.3f} outside [{lower:.3f}, {upper:.3f}]")
    return violations


def build_slave_target(master_start, slave_start, master_current):
    target = []
    for index, current in enumerate(master_current):
        delta = shortest_angle_delta(current, master_start[index]) * JOINT_SCALE[index]
        lower, upper = JOINT_LIMITS[index]
        target.append(clamp(slave_start[index] + delta, lower, upper))
    return target


def max_joint_delta(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def smooth_target(previous_target, target):
    alpha = TARGET_FILTER_ALPHA
    return [
        previous + (current - previous) * alpha
        for previous, current in zip(previous_target, target)
    ]


def limit_step(last_target, target):
    limited = []
    for last, current in zip(last_target, target):
        delta = clamp(current - last, -MAX_STEP_DEG, MAX_STEP_DEG)
        limited.append(last + delta)
    return limited


def is_near_singularity(joints):
    _, j2, j3, _, j5, _ = joints

    if abs(j3) < SINGULARITY_STOP_DEG:
        return True, "elbow singularity risk: J3 near 0 deg"

    if abs(j5) < SINGULARITY_STOP_DEG:
        return True, "wrist singularity risk: J5 near 0 deg"

    if abs(j2) < 10.0 and abs(j3) < SINGULARITY_SLOW_ZONE_DEG:
        return True, "shoulder-like singularity risk: J2/J3 near folded center"

    return False, ""


def moving_closer_to_singularity(last_target, target):
    last_j3 = last_target[2]
    target_j3 = target[2]
    if abs(target_j3) < abs(last_j3) and abs(target_j3) < SINGULARITY_SLOW_ZONE_DEG:
        return True, "J3 moving closer to elbow singularity"

    last_j5 = last_target[4]
    target_j5 = target[4]
    if abs(target_j5) < abs(last_j5) and abs(target_j5) < SINGULARITY_SLOW_ZONE_DEG:
        return True, "J5 moving closer to wrist singularity"

    return False, ""


def assert_pose_safe(name, joints):
    violations = joint_limit_violations(joints)
    if violations:
        raise SafetyStop(f"{name} pose is outside JOINT_LIMITS. " + "; ".join(violations))

    near_singularity, reason = is_near_singularity(joints)
    if near_singularity:
        raise SafetyStop(f"{name} pose is near singularity: {reason}. joints={joints}")


def safety_stop_robot(dashboard):
    if not STOP_SLAVE_ON_SAFETY:
        return
    try:
        dashboard.ResetRobot()
    except Exception as exc:
        print(f"Failed to reset robot after safety stop: {exc}")


def align_slave_to_master(slave_dashboard, slave_move, master_joints, slave_joints):
    delta = max_joint_delta(master_joints, slave_joints)
    if delta <= ALIGN_THRESHOLD:
        print(f"Arms are already aligned. Max joint delta: {delta:.3f} deg")
        return slave_joints

    violations = joint_limit_violations(master_joints)
    if violations:
        raise SafetyStop("Can not align slave: master pose is outside JOINT_LIMITS. " + "; ".join(violations))

    align_target = master_joints[:]
    near_singularity, reason = is_near_singularity(align_target)
    if near_singularity:
        raise SafetyStop(f"Can not align slave to unsafe master pose: {reason}. target={align_target}")

    print(f"Aligning slave to master. Max joint delta: {delta:.3f} deg")
    slave_dashboard.SpeedFactor(ALIGN_SPEED)
    slave_move.JointMovJ(
        align_target[0],
        align_target[1],
        align_target[2],
        align_target[3],
        align_target[4],
        align_target[5],
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
        assert_pose_safe("Master initial", master_start)
        assert_pose_safe("Slave initial", slave_start)

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
        assert_pose_safe("Master follow-start", master_start)
        assert_pose_safe("Slave follow-start", slave_start)
        print("Follow started. Press Ctrl+C to stop.")

        if ENABLE_MASTER_DRAG:
            print("Starting master drag mode...")
            master_dashboard.StartDrag()
            master_drag_started = True

        last_skip_log_time = 0.0

        while True:
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
            target = clamp_to_joint_limits(target)
            target = smooth_target(last_target, target)
            target = limit_step(last_target, target)

            near_singularity, reason = is_near_singularity(target)
            if near_singularity:
                raise SafetyStop(f"Target near singularity: {reason}. target={target}")

            closer, reason = moving_closer_to_singularity(last_target, target)
            if closer:
                raise SafetyStop(f"Target moving closer to singularity: {reason}. target={target}")

            if max_joint_delta(target, last_target) >= JOINT_DEADBAND:
                slave_move.ServoJ(
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
                last_target = target

            time.sleep(FOLLOW_INTERVAL)

    except KeyboardInterrupt:
        print("\nStop requested, exiting...")
    except SafetyStop as exc:
        print(f"\nSafety stop: {exc}")
        safety_stop_robot(slave_dashboard)
        print("Follow control has stopped. Move both arms back to a safe pose before running again.")
        print("Recommended start: keep joints inside JOINT_LIMITS, and keep J3/J5 away from 0 deg.")
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
