#!/usr/bin/env python3
import math
import os
import re
from collections import Counter

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import Empty, Trigger
from snake_path import ROOMS

MATRIX_TOPIC = "/found_blocks_info"
RESULT_TOPIC = "/least_frequent_digits"
RESULT_FILE = "recognized_digits_summary.txt"
EXACT_TSP_MAX_POINTS = 10

APPROACH_OFFSETS = [1.7]
GOAL_DISTANCE_TOLERANCE = 0.40
GOAL_YAW_TOLERANCE = 0.60
GOAL_TIMEOUT_SEC = 50
NO_PROGRESS_TIMEOUT_SEC = 3.0
MIN_PROGRESS_DISTANCE = 0.20
RECOVERY_RETRY_LIMIT = 1
RECOVERY_WAIT_SEC = 0.8
RECOVERY_REVERSE_STEP = 0.8
RECOVERY_SIDE_STEP = 0.5
RECOVERY_NAV_TIMEOUT_SEC = 12.0
RECOVERY_NO_PROGRESS_TIMEOUT_SEC = 3.0
APPROACH_GOAL_TIMEOUT_SEC = 30
APPROACH_NO_PROGRESS_TIMEOUT_SEC = 3.0
APPROACH_RECOVERY_RETRY_LIMIT = 0
BLOCKED_APPROACH_RADIUS = 1.2
MAX_CONSECUTIVE_APPROACH_FAILURES = 4
RECOGNITION_DURATION_SEC = 3.0
BOX_APPROACH_DISTANCE_TOLERANCE = 0.30
BOX_APPROACH_YAW_TOLERANCE = 0.10
ROTATION_TIMEOUT_SEC = 8.0
ROTATION_SETTLE_TIME = 0.4
MIN_ROTATION_SPEED = 0.18
MAX_ROTATION_SPEED = 0.60
ENABLE_WAYPOINT_RELOCALIZATION = True
RELOCALIZE_SETTLE_TIME = 1.0
RELOCALIZE_MAX_DISTANCE = 0.75
RELOCALIZE_MAX_YAW_ERROR = math.pi
RELOCALIZE_POSITION_ONLY_AT_WAYPOINTS = True
MIN_VALID_DIGIT = 1
MAX_VALID_DIGIT = 9
RETURN_HOME_X = 1.73315
RETURN_HOME_Y = 7.41871
RETURN_HOME_YAW = -0.04030936956405639
WALL_BLOCK_SEGMENTS = [
    ((-11.9, 12.5), (-2.99, 12.7)),
]
WALL_CROSSING_PENALTY = 1000.0

current_pose = None


def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose


def pose_to_xyyaw(pose):
    q = pose.orientation
    yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    return pose.position.x, pose.position.y, yaw


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_pose_to_point(pose, target_x, target_y):
    return math.atan2(target_y - pose.position.y, target_x - pose.position.x)


def yaw_from_xy_to_point(from_x, from_y, target_x, target_y):
    return math.atan2(target_y - from_y, target_x - from_x)


def publish_initial_pose(x, y, yaw, publisher):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    msg.pose.pose.orientation.x = quaternion[0]
    msg.pose.pose.orientation.y = quaternion[1]
    msg.pose.pose.orientation.z = quaternion[2]
    msg.pose.pose.orientation.w = quaternion[3]

    msg.pose.covariance[0] = 0.20
    msg.pose.covariance[7] = 0.20
    msg.pose.covariance[35] = 0.10
    publisher.publish(msg)
    rospy.loginfo("发布重定位位姿: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw)


def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def _orientation(a, b, c):
    value = ((b[1] - a[1]) * (c[0] - b[0])) - ((b[0] - a[0]) * (c[1] - b[1]))
    if abs(value) < 1e-9:
        return 0
    return 1 if value > 0 else 2


def _on_segment(a, b, c):
    return (
        min(a[0], c[0]) - 1e-9 <= b[0] <= max(a[0], c[0]) + 1e-9
        and min(a[1], c[1]) - 1e-9 <= b[1] <= max(a[1], c[1]) + 1e-9
    )


def segments_intersect(p1, q1, p2, q2):
    o1 = _orientation(p1, q1, p2)
    o2 = _orientation(p1, q1, q2)
    o3 = _orientation(p2, q2, p1)
    o4 = _orientation(p2, q2, q1)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and _on_segment(p1, p2, q1):
        return True
    if o2 == 0 and _on_segment(p1, q2, q1):
        return True
    if o3 == 0 and _on_segment(p2, p1, q2):
        return True
    if o4 == 0 and _on_segment(p2, q1, q2):
        return True

    return False


def path_crosses_blocked_wall(start_xy, end_xy):
    return any(
        segments_intersect(start_xy, end_xy, wall_start, wall_end)
        for wall_start, wall_end in WALL_BLOCK_SEGMENTS
    )


def travel_cost(start_xy, end_xy):
    base_cost = math.hypot(start_xy[0] - end_xy[0], start_xy[1] - end_xy[1])
    if path_crosses_blocked_wall(start_xy, end_xy):
        return base_cost + WALL_CROSSING_PENALTY
    return base_cost


def find_containing_room(point_xy):
    for room_name, room_info in ROOMS.items():
        if point_in_polygon(point_xy, room_info["polygon"]):
            return room_name
    return None


def relocalize_at_target(target_x, target_y, target_yaw, publisher, block_id):
    if current_pose is None:
        rospy.logwarn("方块 %d 校正时缺少当前位姿，跳过重定位。", block_id)
        return

    target_room = find_containing_room((target_x, target_y))
    if target_room is None:
        rospy.logwarn(
            "方块 %d 参考点 x=%.2f, y=%.2f 不在预设房间内，跳过重定位。",
            block_id, target_x, target_y
        )
        return

    current_room = find_containing_room((current_pose.position.x, current_pose.position.y))
    if current_room is None:
        rospy.logwarn(
            "方块 %d 当前位姿 x=%.2f, y=%.2f 不在预设房间内，跳过重定位。",
            block_id, current_pose.position.x, current_pose.position.y
        )
        return

    if current_room != target_room:
        rospy.logwarn(
            "方块 %d 当前位姿在 %s，但参考点在 %s，跳过跨房间重定位。",
            block_id,
            ROOMS[current_room]["label"],
            ROOMS[target_room]["label"],
        )
        return

    distance = math.hypot(current_pose.position.x - target_x, current_pose.position.y - target_y)
    if distance > RELOCALIZE_MAX_DISTANCE:
        rospy.logwarn(
            "方块 %d 距离参考点过远(%.2fm > %.2fm)，跳过重定位。",
            block_id, distance, RELOCALIZE_MAX_DISTANCE
        )
        return

    q = current_pose.orientation
    current_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    yaw_error = abs(math.atan2(math.sin(current_yaw - target_yaw), math.cos(current_yaw - target_yaw)))
    if yaw_error > RELOCALIZE_MAX_YAW_ERROR:
        rospy.logwarn(
            "方块 %d 朝向误差过大(%.2f > %.2f)，跳过重定位。",
            block_id, yaw_error, RELOCALIZE_MAX_YAW_ERROR
        )
        return

    relocalize_yaw = current_yaw if RELOCALIZE_POSITION_ONLY_AT_WAYPOINTS else target_yaw
    publish_initial_pose(target_x, target_y, relocalize_yaw, publisher)
    rospy.sleep(RELOCALIZE_SETTLE_TIME)


def create_goal(x, y, yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    return goal


def call_clear_costmaps(clear_costmaps_service):
    if clear_costmaps_service is None:
        return False
    try:
        clear_costmaps_service()
        rospy.logwarn("已调用 /move_base/clear_costmaps 清理局部障碍。")
        return True
    except rospy.ServiceException as exc:
        rospy.logwarn("清理 costmap 失败: %s", exc)
        return False


def build_reverse_recovery_goals(goal_x, goal_y):
    if current_pose is None:
        return []

    start_x = current_pose.position.x
    start_y = current_pose.position.y
    _, _, current_yaw = pose_to_xyyaw(current_pose)
    dx = goal_x - start_x
    dy = goal_y - start_y
    distance = math.hypot(dx, dy)
    if distance < 1e-3:
        ux = math.cos(current_yaw)
        uy = math.sin(current_yaw)
    else:
        ux = dx / distance
        uy = dy / distance

    left_x = -uy
    left_y = ux
    reverse_x = start_x - ux * RECOVERY_REVERSE_STEP
    reverse_y = start_y - uy * RECOVERY_REVERSE_STEP

    return [
        (reverse_x, reverse_y, yaw_from_xy_to_point(reverse_x, reverse_y, goal_x, goal_y), "reverse recovery"),
        (
            reverse_x + left_x * RECOVERY_SIDE_STEP,
            reverse_y + left_y * RECOVERY_SIDE_STEP,
            yaw_from_xy_to_point(reverse_x + left_x * RECOVERY_SIDE_STEP, reverse_y + left_y * RECOVERY_SIDE_STEP, goal_x, goal_y),
            "reverse-left recovery",
        ),
        (
            reverse_x - left_x * RECOVERY_SIDE_STEP,
            reverse_y - left_y * RECOVERY_SIDE_STEP,
            yaw_from_xy_to_point(reverse_x - left_x * RECOVERY_SIDE_STEP, reverse_y - left_y * RECOVERY_SIDE_STEP, goal_x, goal_y),
            "reverse-right recovery",
        ),
    ]


def run_reverse_recovery(client, goal_x, goal_y, clear_costmaps_service, no_progress_timeout_sec):
    if current_pose is None:
        rospy.logwarn("恢复流程中缺少当前位姿，无法执行反向避障。")
        return False

    call_clear_costmaps(clear_costmaps_service)
    rospy.sleep(RECOVERY_WAIT_SEC)

    recovered = False
    for recovery_x, recovery_y, recovery_yaw, recovery_name in build_reverse_recovery_goals(goal_x, goal_y):
        rospy.logwarn(
            "尝试 %s: x=%.2f, y=%.2f, yaw=%.2f",
            recovery_name,
            recovery_x,
            recovery_y,
            recovery_yaw,
        )
        recovery_ok, recovery_reason = send_goal(
            client,
            recovery_x,
            recovery_y,
            recovery_yaw,
            timeout_sec=RECOVERY_NAV_TIMEOUT_SEC,
            clear_costmaps_service=None,
            no_progress_timeout_sec=min(no_progress_timeout_sec, RECOVERY_NO_PROGRESS_TIMEOUT_SEC),
            recovery_retry_limit=0,
            return_reason=True,
            require_heading=False,
        )
        if recovery_ok:
            rospy.loginfo("%s succeeded; retrying original goal from a new position.", recovery_name)
            recovered = True
            break

        rospy.logwarn("%s failed during recovery, reason=%s.", recovery_name, recovery_reason)

    if not recovered:
        rospy.logwarn("反向避障恢复失败，将直接重试原目标。")
    return recovered


def rotate_in_place(cmd_vel_pub, target_yaw, tolerance=BOX_APPROACH_YAW_TOLERANCE, timeout_sec=ROTATION_TIMEOUT_SEC):
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)
    twist = Twist()

    while not rospy.is_shutdown():
        if current_pose is None:
            rate.sleep()
            continue

        _, _, current_yaw = pose_to_xyyaw(current_pose)
        yaw_error = normalize_angle(target_yaw - current_yaw)
        if abs(yaw_error) <= tolerance:
            break

        if timeout_sec is not None and rospy.Time.now() - start_time > rospy.Duration(timeout_sec):
            rospy.logwarn(
                "原地转向超时，当前朝向误差 %.2f rad，目标 yaw=%.2f。",
                yaw_error,
                target_yaw,
            )
            break

        angular_speed = max(MIN_ROTATION_SPEED, min(MAX_ROTATION_SPEED, 1.2 * abs(yaw_error)))
        twist.angular.z = math.copysign(angular_speed, yaw_error)
        cmd_vel_pub.publish(twist)
        rate.sleep()

    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    rospy.sleep(ROTATION_SETTLE_TIME)

    if current_pose is None:
        return False

    _, _, final_yaw = pose_to_xyyaw(current_pose)
    final_error = abs(normalize_angle(target_yaw - final_yaw))
    if final_error > tolerance:
        rospy.logwarn("原地转向后仍未充分对正，剩余误差 %.2f rad。", final_error)
        return False

    rospy.loginfo("原地转向完成，最终朝向误差 %.2f rad。", final_error)
    return True


def align_camera_to_block_center(cmd_vel_pub, cube_x, cube_y, fallback_yaw):
    final_yaw = fallback_yaw

    if current_pose is None:
        rospy.logwarn("缺少当前位姿，使用候选点预设 yaw 对准方块。")
        aligned = rotate_in_place(
            cmd_vel_pub,
            fallback_yaw,
            tolerance=BOX_APPROACH_YAW_TOLERANCE,
        )
    else:
        aligned = True
        for alignment_pass in range(2):
            target_yaw = yaw_from_pose_to_point(current_pose, cube_x, cube_y)
            rospy.loginfo(
                "第 %d 次根据当前位置对准方块中心: robot=(%.2f, %.2f), cube=(%.2f, %.2f), target_yaw=%.2f",
                alignment_pass + 1,
                current_pose.position.x,
                current_pose.position.y,
                cube_x,
                cube_y,
                target_yaw,
            )
            aligned = rotate_in_place(
                cmd_vel_pub,
                target_yaw,
                tolerance=BOX_APPROACH_YAW_TOLERANCE,
            ) and aligned
            final_yaw = target_yaw
            rospy.sleep(0.2)

    if current_pose is not None:
        _, _, final_yaw = pose_to_xyyaw(current_pose)

    return aligned, final_yaw


def send_goal(
    client,
    x,
    y,
    yaw,
    timeout_sec=GOAL_TIMEOUT_SEC,
    clear_costmaps_service=None,
    no_progress_timeout_sec=NO_PROGRESS_TIMEOUT_SEC,
    recovery_retry_limit=RECOVERY_RETRY_LIMIT,
    return_reason=False,
    require_heading=False,
    yaw_tolerance=GOAL_YAW_TOLERANCE,
    distance_tolerance=GOAL_DISTANCE_TOLERANCE,
):
    def finish(success, reason):
        if return_reason:
            return success, reason
        return success

    for recovery_attempt in range(recovery_retry_limit + 1):
        goal = create_goal(x, y, yaw)
        rospy.loginfo(
            "Sending goal: x=%.2f, y=%.2f, yaw=%.2f (attempt %d/%d)",
            x,
            y,
            yaw,
            recovery_attempt + 1,
            recovery_retry_limit + 1,
        )
        client.send_goal(goal)

        start_time = rospy.Time.now()
        last_progress_time = start_time
        best_distance = float("inf")
        failure_reason = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if current_pose is not None:
                dx = current_pose.position.x - x
                dy = current_pose.position.y - y
                distance = math.hypot(dx, dy)

                _, _, current_yaw = pose_to_xyyaw(current_pose)
                angle_diff = abs(normalize_angle(current_yaw - yaw))
                rospy.loginfo("Distance to goal: %.2f, Orientation diff: %.2f", distance, angle_diff)

                if best_distance == float("inf") or best_distance - distance >= MIN_PROGRESS_DISTANCE:
                    best_distance = distance
                    last_progress_time = rospy.Time.now()

                if distance <= distance_tolerance and angle_diff <= yaw_tolerance:
                    rospy.loginfo("Reached goal with position and heading tolerance.")
                    client.cancel_goal()
                    return finish(True, "reached")

                if distance <= distance_tolerance and not require_heading:
                    rospy.loginfo("Reached goal position, skipping final heading alignment.")
                    client.cancel_goal()
                    return finish(True, "reached")

                if rospy.Time.now() - last_progress_time > rospy.Duration(no_progress_timeout_sec):
                    rospy.logwarn(
                        "连续 %.1f 秒没有明显接近目标，视为被障碍阻塞。",
                        no_progress_timeout_sec,
                    )
                    failure_reason = "no_progress"
                    break

            state = client.get_state()
            if state == GoalStatus.SUCCEEDED:
                if not require_heading:
                    rospy.loginfo("move_base reported success.")
                    return finish(True, "succeeded")

                if current_pose is None:
                    rospy.logwarn("move_base 已成功，但当前位姿缺失，无法确认最终朝向。")
                    failure_reason = "missing_pose"
                    break

                dx = current_pose.position.x - x
                dy = current_pose.position.y - y
                distance = math.hypot(dx, dy)
                _, _, current_yaw = pose_to_xyyaw(current_pose)
                angle_diff = abs(normalize_angle(current_yaw - yaw))
                if distance <= distance_tolerance and angle_diff <= yaw_tolerance:
                    rospy.loginfo("move_base reported success and final heading is within tolerance.")
                    return finish(True, "succeeded")

                rospy.logwarn(
                    "move_base reported success but final pose is not aligned enough: distance=%.2f, yaw_diff=%.2f",
                    distance,
                    angle_diff,
                )
                failure_reason = "heading_mismatch"
                break
            if state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                rospy.logwarn("Navigation failed with state %d.", state)
                failure_reason = "planner_failed"
                break

            if timeout_sec is not None and rospy.Time.now() - start_time > rospy.Duration(timeout_sec):
                rospy.logwarn("Navigation timed out.")
                failure_reason = "timeout"
                break

            rate.sleep()

        client.cancel_goal()
        if recovery_attempt >= recovery_retry_limit:
            rospy.logwarn("Navigation failed after recovery attempts, reason=%s.", failure_reason or "failed")
            return finish(False, failure_reason or "failed")

        rospy.logwarn("开始恢复流程: 先反向退一段避开障碍物，再重试当前目标。")
        run_reverse_recovery(
            client,
            x,
            y,
            clear_costmaps_service,
            no_progress_timeout_sec,
        )

    return finish(False, "failed")


def compute_tsp_order(start, points):
    n = len(points)
    if n == 0:
        return []

    if n > EXACT_TSP_MAX_POINTS:
        rospy.logwarn(
            "Block count %d is too large for exact TSP; switching to greedy nearest-neighbor ordering.",
            n,
        )
        remaining = list(points)
        ordered = []
        current_x, current_y = start

        while remaining:
            next_idx = min(
                range(len(remaining)),
                key=lambda i: travel_cost(
                    (current_x, current_y),
                    (remaining[i]["cube_x"], remaining[i]["cube_y"]),
                ),
            )
            next_point = remaining.pop(next_idx)
            ordered.append(next_point)
            current_x = next_point["cube_x"]
            current_y = next_point["cube_y"]

        return ordered

    dp = {}
    parent = {}
    for i in range(n):
        dp[(1 << i, i)] = travel_cost(start, (points[i]["cube_x"], points[i]["cube_y"]))

    for mask in range(1, 1 << n):
        for i in range(n):
            if not mask & (1 << i):
                continue
            prev_mask = mask ^ (1 << i)
            if prev_mask == 0:
                continue
            best_cost = float("inf")
            best_prev = None
            for j in range(n):
                if not prev_mask & (1 << j):
                    continue
                cost = dp[(prev_mask, j)] + travel_cost(
                    (points[j]["cube_x"], points[j]["cube_y"]),
                    (points[i]["cube_x"], points[i]["cube_y"]),
                )
                if cost < best_cost:
                    best_cost = cost
                    best_prev = j
            dp[(mask, i)] = best_cost
            parent[(mask, i)] = best_prev

    full_mask = (1 << n) - 1
    end_idx = min(
        range(n),
        key=lambda i: dp[(full_mask, i)] + travel_cost(
            (points[i]["cube_x"], points[i]["cube_y"]),
            start,
        ),
    )

    path = [end_idx]
    mask = full_mask
    while (mask, path[-1]) in parent:
        prev = parent[(mask, path[-1])]
        mask ^= 1 << path[-1]
        path.append(prev)
    path.reverse()
    return [points[i] for i in path]


def parse_block_string(data_str):
    pattern = r"\(\s*(\d+)\s*,\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*,\s*(-?\d+)\s*\)"
    points = []
    for pid, x_text, y_text, label_text in re.findall(pattern, data_str):
        try:
            cube_x = float(x_text)
            cube_y = float(y_text)
        except ValueError:
            rospy.logwarn("Skipping malformed block entry: (%s, %s, %s, %s)", pid, x_text, y_text, label_text)
            continue
        points.append(
            {
                "id": int(pid),
                "cube_x": cube_x,
                "cube_y": cube_y,
                "label": int(label_text),
            }
        )
    return points


def wait_for_block_list():
    rospy.loginfo("Waiting for a stable /found_blocks_info message...")
    stable_points = []
    stable_count = 0
    last_signature = None

    while not rospy.is_shutdown():
        msg = rospy.wait_for_message(MATRIX_TOPIC, String)
        points = parse_block_string(msg.data)
        if not points:
            rospy.loginfo("No blocks detected yet, waiting for the next update.")
            continue

        signature = tuple((p["id"], round(p["cube_x"], 2), round(p["cube_y"], 2)) for p in points)
        if signature == last_signature:
            stable_count += 1
        else:
            stable_count = 1
            stable_points = points
            last_signature = signature

        rospy.loginfo("Observed %d candidate blocks. Stability counter: %d/2", len(points), stable_count)
        if stable_count >= 2:
            return stable_points


def recognize_digit_once():
    try:
        rospy.wait_for_service("start_recognition", timeout=3)
        rospy.wait_for_service("stop_recognition", timeout=5)
        start_srv = rospy.ServiceProxy("start_recognition", Trigger)
        stop_srv = rospy.ServiceProxy("stop_recognition", Trigger)
        start_resp = start_srv()
        rospy.loginfo("Digit recognition started: %s", start_resp.message)
        rospy.sleep(RECOGNITION_DURATION_SEC)
        stop_resp = stop_srv()
        rospy.loginfo("Digit recognition stopped: %s", stop_resp.message)
    except rospy.ServiceException as exc:
        rospy.logerr("Digit recognition service call failed: %s", exc)
        return -1

    match = re.search(r"最佳识别结果为：(\d)", stop_resp.message)
    if not match:
        return -1
    return int(match.group(1))


def summarize_digits(recognized_digits):
    valid_digits = [digit for digit in recognized_digits if MIN_VALID_DIGIT <= digit <= MAX_VALID_DIGIT]
    counts = Counter(valid_digits)
    if not counts:
        return {
            "valid_digits": valid_digits,
            "counts": {},
            "least_frequent_digits": [],
            "least_count": 0,
        }

    least_count = min(counts.values())
    least_frequent_digits = sorted([digit for digit, count in counts.items() if count == least_count])
    return {
        "valid_digits": valid_digits,
        "counts": dict(sorted(counts.items())),
        "least_frequent_digits": least_frequent_digits,
        "least_count": least_count,
    }


def write_recognition_summary(output_path, recognized_digits, visit_summaries):
    summary = summarize_digits(recognized_digits)
    summary_text = (
        "recognized_digits={}; counts={}; least_frequent_digits={}; least_count={}".format(
            recognized_digits,
            summary["counts"],
            summary["least_frequent_digits"],
            summary["least_count"],
        )
    )

    with open(output_path, "w") as result_file:
        result_file.write(summary_text + "\n")
        for item in visit_summaries:
            result_file.write(
                "block_id={id}, cube_x={cube_x:.3f}, cube_y={cube_y:.3f}, recognized_digit={recognized_digit}\n".format(
                    **item
                )
            )

    return summary, summary_text


def build_approach_candidates(cube_x, cube_y):
    candidates = []
    for offset in APPROACH_OFFSETS:
        candidates.extend(
            [
                (cube_x + offset, cube_y, yaw_from_xy_to_point(cube_x + offset, cube_y, cube_x, cube_y)),
                (cube_x - offset, cube_y, yaw_from_xy_to_point(cube_x - offset, cube_y, cube_x, cube_y)),
                (cube_x, cube_y + offset, yaw_from_xy_to_point(cube_x, cube_y + offset, cube_x, cube_y)),
                (cube_x, cube_y - offset, yaw_from_xy_to_point(cube_x, cube_y - offset, cube_x, cube_y)),
            ]
        )
    return candidates


def order_candidates_by_current_pose(candidates):
    if current_pose is None:
        return candidates
    return sorted(
        candidates,
        key=lambda item: travel_cost(
            (current_pose.position.x, current_pose.position.y),
            (item[0], item[1]),
        )
    )


def count_blocked_future_candidates(candidates, start_index, blocked_center, blocked_radius):
    blocked_count = 0
    for target_x, target_y, _ in candidates[start_index:]:
        if math.hypot(target_x - blocked_center[0], target_y - blocked_center[1]) <= blocked_radius:
            blocked_count += 1
        else:
            break
    return blocked_count


class PointNavigator:
    def __init__(self):
        rospy.init_node("block_digit_mission")
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base")

        self.initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.digit_pub = rospy.Publisher("/recognized_digits", Int32MultiArray, queue_size=10, latch=True)
        self.summary_pub = rospy.Publisher(RESULT_TOPIC, String, queue_size=10, latch=True)
        self.clear_costmaps_service = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        self.mission_start_pose = None

    def return_to_start_and_relocalize(self):
        start_x = RETURN_HOME_X
        start_y = RETURN_HOME_Y
        start_yaw = RETURN_HOME_YAW
        rospy.loginfo(
            "Returning to fixed home pose: x=%.2f, y=%.2f, yaw=%.2f",
            start_x,
            start_y,
            start_yaw,
        )
        nav_ok = send_goal(
            self.client,
            start_x,
            start_y,
            start_yaw,
            clear_costmaps_service=self.clear_costmaps_service,
        )
        if not nav_ok:
            rospy.logwarn("Failed to return to the fixed home pose; skipping final relocalization.")
            return

        publish_initial_pose(start_x, start_y, start_yaw, self.initial_pose_pub)
        rospy.sleep(RELOCALIZE_SETTLE_TIME)
        rospy.loginfo("Final relocalization at fixed home pose completed.")

    def navigate_and_recognize(self):
        while current_pose is None and not rospy.is_shutdown():
            rospy.logwarn("Waiting for AMCL pose...")
            rospy.sleep(0.5)

        self.mission_start_pose = pose_to_xyyaw(current_pose)
        rospy.loginfo(
            "Recorded mission start pose: x=%.2f, y=%.2f, yaw=%.2f",
            self.mission_start_pose[0],
            self.mission_start_pose[1],
            self.mission_start_pose[2],
        )

        points = wait_for_block_list()
        rospy.loginfo("Stable block list acquired: %s", points)

        start_xy = (current_pose.position.x, current_pose.position.y)
        ordered_points = compute_tsp_order(start_xy, points)
        rospy.loginfo("Visit order: %s", [(p['id'], round(p['cube_x'], 2), round(p['cube_y'], 2)) for p in ordered_points])

        recognized_digits = []
        visit_summaries = []
        output_path = os.path.join(os.getcwd(), RESULT_FILE)

        for index, point in enumerate(ordered_points, 1):
            cube_x = point["cube_x"]
            cube_y = point["cube_y"]
            candidates = order_candidates_by_current_pose(build_approach_candidates(cube_x, cube_y))
            recognized_digit = -1
            successful_approach = None
            consecutive_approach_failures = 0

            rospy.loginfo(
                "--- Visiting block %d/%d: id=%d cube=(%.2f, %.2f) ---",
                index, len(ordered_points), point["id"], cube_x, cube_y
            )
            attempt_index = 0
            while attempt_index < len(candidates):
                target_x, target_y, yaw = candidates[attempt_index]
                rospy.loginfo(
                    "Trying approach %d/%d for block %d: approach=(%.2f, %.2f), yaw=%.2f",
                    attempt_index + 1, len(candidates), point["id"], target_x, target_y, yaw
                )
                nav_ok, failure_reason = send_goal(
                    self.client,
                    target_x,
                    target_y,
                    yaw,
                    timeout_sec=APPROACH_GOAL_TIMEOUT_SEC,
                    clear_costmaps_service=self.clear_costmaps_service,
                    no_progress_timeout_sec=APPROACH_NO_PROGRESS_TIMEOUT_SEC,
                    recovery_retry_limit=APPROACH_RECOVERY_RETRY_LIMIT,
                    return_reason=True,
                    require_heading=False,
                    yaw_tolerance=BOX_APPROACH_YAW_TOLERANCE,
                    distance_tolerance=BOX_APPROACH_DISTANCE_TOLERANCE,
                )
                if not nav_ok:
                    consecutive_approach_failures += 1
                    blocked_center = (target_x, target_y)
                    if current_pose is not None:
                        blocked_center = (current_pose.position.x, current_pose.position.y)
                    blocked_count = count_blocked_future_candidates(
                        candidates,
                        attempt_index,
                        blocked_center,
                        BLOCKED_APPROACH_RADIUS,
                    )
                    blocked_count = max(1, blocked_count)
                    rospy.logwarn(
                        (
                            "Approach %d/%d for block %d failed in navigation, reason=%s. "
                            "Skipping %d nearby candidate(s); consecutive failures: %d/%d."
                        ),
                        attempt_index + 1,
                        len(candidates),
                        point["id"],
                        failure_reason,
                        blocked_count,
                        consecutive_approach_failures,
                        MAX_CONSECUTIVE_APPROACH_FAILURES,
                    )
                    if consecutive_approach_failures >= MAX_CONSECUTIVE_APPROACH_FAILURES:
                        rospy.logwarn(
                            "Block %d has too many consecutive blocked approaches; switching to the next block.",
                            point["id"],
                        )
                        break
                    call_clear_costmaps(self.clear_costmaps_service)
                    rospy.sleep(RECOVERY_WAIT_SEC)
                    attempt_index += blocked_count
                    continue

                consecutive_approach_failures = 0
                rospy.loginfo(
                    "Reached block %d from approach=(%.2f, %.2f), yaw=%.2f",
                    point["id"], target_x, target_y, yaw
                )
                aligned, final_yaw = align_camera_to_block_center(
                    self.cmd_vel_pub,
                    cube_x,
                    cube_y,
                    yaw,
                )
                if not aligned:
                    rospy.logwarn("Block %d final camera alignment was not fully within tolerance.", point["id"])
                rospy.sleep(0.5)
                successful_approach = (target_x, target_y, final_yaw)
                if ENABLE_WAYPOINT_RELOCALIZATION:
                    rospy.loginfo("Relocalizing before recognizing block %d.", point["id"])
                    relocalize_at_target(
                        target_x,
                        target_y,
                        final_yaw,
                        self.initial_pose_pub,
                        point["id"],
                    )
                recognized_digit = recognize_digit_once()
                if recognized_digit != -1:
                    rospy.loginfo(
                        "Block %d recognized as %d from approach %d/%d.",
                        point["id"], recognized_digit, attempt_index + 1, len(candidates)
                    )
                    break

                rospy.logwarn(
                    "Block %d recognition failed at approach %d/%d, trying another viewing angle.",
                    point["id"], attempt_index + 1, len(candidates)
                )
                attempt_index += 1

            if successful_approach is None:
                rospy.logwarn("Skipping block %d because all approach navigations failed.", point["id"])
            elif recognized_digit == -1:
                rospy.logwarn("Block %d was reached, but recognition failed from all attempted viewing angles.", point["id"])

            recognized_digits.append(recognized_digit)
            visit_summaries.append(
                {
                    "id": point["id"],
                    "cube_x": cube_x,
                    "cube_y": cube_y,
                    "recognized_digit": recognized_digit,
                }
            )
            self.digit_pub.publish(Int32MultiArray(data=recognized_digits))
            rospy.loginfo("Recognized digits so far: %s", recognized_digits)
            _, partial_summary_text = write_recognition_summary(output_path, recognized_digits, visit_summaries)
            rospy.loginfo("Updated recognition summary file after block %d: %s", point["id"], output_path)
            self.summary_pub.publish(String(data=partial_summary_text))
            rospy.sleep(0.5)

        summary, summary_text = write_recognition_summary(output_path, recognized_digits, visit_summaries)
        self.summary_pub.publish(String(data=summary_text))
        rospy.loginfo("Mission summary: %s", summary_text)
        rospy.loginfo("Wrote recognition summary to %s", output_path)
        self.return_to_start_and_relocalize()


if __name__ == "__main__":
    try:
        navigator = PointNavigator()
        navigator.navigate_and_recognize()
    except rospy.ROSInterruptException:
        pass
