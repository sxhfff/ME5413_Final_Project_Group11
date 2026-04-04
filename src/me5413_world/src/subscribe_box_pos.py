#!/usr/bin/env python3
import math
import os
import re
from collections import Counter

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32MultiArray, String
from std_srvs.srv import Trigger

MATRIX_TOPIC = "/found_blocks_info"
RESULT_TOPIC = "/least_frequent_digits"
RESULT_FILE = "recognized_digits_summary.txt"

APPROACH_OFFSET_X = 1.3
GOAL_DISTANCE_TOLERANCE = 0.40
GOAL_YAW_TOLERANCE = 0.60
GOAL_TIMEOUT_SEC = 50
RECOGNITION_DURATION_SEC = 1.5
MIN_VALID_DIGIT = 0
MAX_VALID_DIGIT = 9

current_pose = None


def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose


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


def send_goal(client, x, y, yaw, timeout_sec=GOAL_TIMEOUT_SEC):
    goal = create_goal(x, y, yaw)
    rospy.loginfo("Sending goal: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw)
    client.send_goal(goal)

    start_time = rospy.Time.now()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if current_pose is not None:
            dx = current_pose.position.x - x
            dy = current_pose.position.y - y
            distance = math.hypot(dx, dy)

            q = current_pose.orientation
            current_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            angle_diff = abs(math.atan2(math.sin(current_yaw - yaw), math.cos(current_yaw - yaw)))
            rospy.loginfo("Distance to goal: %.2f, Orientation diff: %.2f", distance, angle_diff)

            if distance <= GOAL_DISTANCE_TOLERANCE and angle_diff <= GOAL_YAW_TOLERANCE:
                rospy.loginfo("Reached goal with position and heading tolerance.")
                client.cancel_goal()
                return True

            if distance <= GOAL_DISTANCE_TOLERANCE:
                rospy.loginfo("Reached goal position, skipping final heading alignment.")
                client.cancel_goal()
                return True

        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("move_base reported success.")
            return True
        if state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
            rospy.logwarn("Navigation failed with state %d.", state)
            return False

        if timeout_sec is not None and rospy.Time.now() - start_time > rospy.Duration(timeout_sec):
            rospy.logwarn("Navigation timed out. Cancelling goal.")
            client.cancel_goal()
            return False

        rate.sleep()


def compute_tsp_order(start, points):
    n = len(points)
    if n == 0:
        return []

    dp = {}
    parent = {}
    for i in range(n):
        dp[(1 << i, i)] = math.hypot(start[0] - points[i]["cube_x"], start[1] - points[i]["cube_y"])

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
                cost = dp[(prev_mask, j)] + math.hypot(
                    points[j]["cube_x"] - points[i]["cube_x"],
                    points[j]["cube_y"] - points[i]["cube_y"],
                )
                if cost < best_cost:
                    best_cost = cost
                    best_prev = j
            dp[(mask, i)] = best_cost
            parent[(mask, i)] = best_prev

    full_mask = (1 << n) - 1
    end_idx = min(range(n), key=lambda i: dp[(full_mask, i)])

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
        cube_x = float(x_text)
        cube_y = float(y_text)
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


class PointNavigator:
    def __init__(self):
        rospy.init_node("block_digit_mission")
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base")

        self.digit_pub = rospy.Publisher("/recognized_digits", Int32MultiArray, queue_size=10, latch=True)
        self.summary_pub = rospy.Publisher(RESULT_TOPIC, String, queue_size=10, latch=True)

    def navigate_and_recognize(self):
        while current_pose is None and not rospy.is_shutdown():
            rospy.logwarn("Waiting for AMCL pose...")
            rospy.sleep(0.5)

        points = wait_for_block_list()
        rospy.loginfo("Stable block list acquired: %s", points)

        start_xy = (current_pose.position.x, current_pose.position.y)
        ordered_points = compute_tsp_order(start_xy, points)
        rospy.loginfo("Visit order: %s", [(p['id'], round(p['cube_x'], 2), round(p['cube_y'], 2)) for p in ordered_points])

        recognized_digits = []
        visit_summaries = []

        for index, point in enumerate(ordered_points, 1):
            cube_x = point["cube_x"]
            cube_y = point["cube_y"]
            target_x = cube_x + APPROACH_OFFSET_X
            target_y = cube_y
            yaw = math.pi

            rospy.loginfo(
                "--- Visiting block %d/%d: id=%d cube=(%.2f, %.2f) approach=(%.2f, %.2f) ---",
                index, len(ordered_points), point["id"], cube_x, cube_y, target_x, target_y
            )
            nav_ok = send_goal(self.client, target_x, target_y, yaw)
            if not nav_ok:
                rospy.logwarn("Skipping block %d because navigation failed.", point["id"])
                recognized_digit = -1
            else:
                rospy.sleep(0.5)
                recognized_digit = recognize_digit_once()

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
            rospy.sleep(0.5)

        summary = summarize_digits(recognized_digits)
        summary_text = (
            "recognized_digits={}; counts={}; least_frequent_digits={}; least_count={}".format(
                recognized_digits,
                summary["counts"],
                summary["least_frequent_digits"],
                summary["least_count"],
            )
        )
        self.summary_pub.publish(String(data=summary_text))
        rospy.loginfo("Mission summary: %s", summary_text)

        output_path = os.path.join(os.getcwd(), RESULT_FILE)
        with open(output_path, "w") as result_file:
            result_file.write(summary_text + "\n")
            for item in visit_summaries:
                result_file.write(
                    "block_id={id}, cube_x={cube_x:.3f}, cube_y={cube_y:.3f}, recognized_digit={recognized_digit}\n".format(
                        **item
                    )
                )
        rospy.loginfo("Wrote recognition summary to %s", output_path)


if __name__ == "__main__":
    try:
        navigator = PointNavigator()
        navigator.navigate_and_recognize()
    except rospy.ROSInterruptException:
        pass
