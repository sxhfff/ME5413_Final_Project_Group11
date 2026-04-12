#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import tf
import math

ROOMS = {
    "room_1": {
        "label": "房间1",
        "polygon": [
            (2.04, 2.87),
            (-17.0, 2.97),
            (-16.8, 12.1),
            (2.28, 12.1),
        ],
    },
    "room_2": {
        "label": "房间2",
        "polygon": [
            (2.25, 12.7),
            (-16.9, 12.9),
            (-16.8, 22.0),
            (2.2, 21.6),
        ],
    },
}
DOORS = [
    {"label": "右门", "center": (-1.98, 12.5)},
    {"label": "左门", "center": (-12.9, 12.5)},
]
POSITION_GOAL_TOLERANCE = 0.55
ORIENTATION_GOAL_TOLERANCE = 1.20
MAX_SEGMENT_LENGTH = 5.0
MIN_WAYPOINT_SPACING = 1.8
WALL_MARGIN = 1.0
ROOM_SNAKE_COLUMNS = 4
DOOR_APPROACH_OFFSET = 0.85
MAX_CONSECUTIVE_NAV_FAILURES = 6
GOAL_TIMEOUT_SEC = 40
NO_PROGRESS_TIMEOUT_SEC = 10.0
MIN_PROGRESS_DISTANCE = 0.20
TRANSITION_SETTLE_TIME = 0.0
CRUISE_SETTLE_TIME = 0.0
RELOCALIZE_AFTER_TRANSITION = False
RELOCALIZE_SETTLE_TIME = 1.0
ENABLE_WAYPOINT_RELOCALIZATION = True
RELOCALIZE_EVERY_N_WAYPOINTS = 1
RELOCALIZE_MAX_DISTANCE = 0.75
RELOCALIZE_MAX_YAW_ERROR = math.pi
RELOCALIZE_POSITION_ONLY_AT_WAYPOINTS = True
BLOCKED_WAYPOINT_RADIUS = 2.2
RECOVERY_RETRY_LIMIT = 1
RECOVERY_WAIT_SEC = 0.8
current_pose = None

def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

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
    rospy.loginfo("发布重定位位姿: x=%.3f, y=%.3f, yaw=%.3f", x, y, yaw)

def maybe_relocalize_at_checkpoint(target_x, target_y, target_yaw, publisher, label):
    if current_pose is None:
        rospy.logwarn("%s 时缺少当前位姿，跳过重定位。", label)
        return

    target_room = find_containing_room((target_x, target_y))
    if target_room is None:
        rospy.logwarn(
            "%s 参考点 x=%.2f, y=%.2f 不在预设房间内，跳过重定位。",
            label, target_x, target_y
        )
        return

    current_room = find_containing_room((current_pose.position.x, current_pose.position.y))
    if current_room is None:
        rospy.logwarn(
            "%s 当前位姿 x=%.2f, y=%.2f 不在预设房间内，跳过重定位。",
            label, current_pose.position.x, current_pose.position.y
        )
        return

    if current_room != target_room:
        rospy.logwarn(
            "%s 当前位姿在 %s，但参考点在 %s，跳过跨房间重定位。",
            label,
            ROOMS[current_room]["label"],
            ROOMS[target_room]["label"],
        )
        return

    dx = current_pose.position.x - target_x
    dy = current_pose.position.y - target_y
    distance = math.hypot(dx, dy)

    q = current_pose.orientation
    current_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    yaw_error = abs(math.atan2(math.sin(current_yaw - target_yaw), math.cos(current_yaw - target_yaw)))

    if distance > RELOCALIZE_MAX_DISTANCE:
        rospy.logwarn(
            "%s 距离参考点过远(%.2fm > %.2fm)，跳过重定位。",
            label, distance, RELOCALIZE_MAX_DISTANCE
        )
        return

    if yaw_error > RELOCALIZE_MAX_YAW_ERROR:
        rospy.logwarn(
            "%s 朝向误差过大(%.2f > %.2f)，跳过重定位。",
            label, yaw_error, RELOCALIZE_MAX_YAW_ERROR
        )
        return

    relocalize_yaw = target_yaw
    if RELOCALIZE_POSITION_ONLY_AT_WAYPOINTS:
        relocalize_yaw = current_yaw

    publish_initial_pose(target_x, target_y, relocalize_yaw, publisher)
    rospy.sleep(RELOCALIZE_SETTLE_TIME)


def compute_centroid(polygon):
    if not polygon:
        return (0.0, 0.0)
    return (
        sum(point[0] for point in polygon) / float(len(polygon)),
        sum(point[1] for point in polygon) / float(len(polygon)),
    )


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

def get_y_segments(x, polygon):
    """
    计算垂直线 x 与多边形边界的交点，返回该列所有可通行的 y 段。
    对于有内凹角的一楼平面，单列可能出现多个独立区间。
    """
    intersections = []
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]

        if x1 == x2:
            if abs(x - x1) < 1e-6:
                intersections.extend([y1, y2])
            continue

        if (x >= min(x1, x2)) and (x < max(x1, x2)):
            y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            intersections.append(y)

    if not intersections:
        return []

    intersections = sorted(intersections)
    segments = []
    for idx in range(0, len(intersections) - 1, 2):
        y_bottom = intersections[idx]
        y_top = intersections[idx + 1]
        if y_top - y_bottom > 1e-3:
            segments.append((y_bottom, y_top))
    return segments

def generate_polygon_snake_path(polygon, num_cols):
    """
    在任意多边形内生成蛇形航点
    """
    # 提取所有点的 X 坐标，找到最左和最右的边界
    xs = [p[0] for p in polygon]
    min_x = min(xs)
    max_x = max(xs)
    
    waypoints = []
    
    # 从右向左生成 X 坐标列
    x_list = [max_x - i * (max_x - min_x) / float(num_cols - 1) for i in range(num_cols)]
    
    for idx, x in enumerate(x_list):
        segments = get_y_segments(x, polygon)
        if not segments:
            continue

        else:
            ordered_segments = segments if idx % 2 == 0 else list(reversed(segments))
            for seg_idx, (y_bottom, y_top) in enumerate(ordered_segments):
                go_up = (idx + seg_idx) % 2 == 0
                if go_up:
                    waypoints.append((x, y_bottom))
                    waypoints.append((x, y_top))
                else:
                    waypoints.append((x, y_top))
                    waypoints.append((x, y_bottom))
            
    return waypoints

def offset_polygon_inward(polygon, margin):
    """
    Move each polygon vertex toward the centroid so the patrol path stays away from walls.
    """
    if not polygon:
        return polygon

    centroid_x = sum(p[0] for p in polygon) / float(len(polygon))
    centroid_y = sum(p[1] for p in polygon) / float(len(polygon))

    adjusted = []
    for x, y in polygon:
        dx = centroid_x - x
        dy = centroid_y - y
        distance = math.hypot(dx, dy)
        if distance <= margin or distance == 0.0:
            adjusted.append((x, y))
            continue
        scale = margin / distance
        adjusted.append((x + dx * scale, y + dy * scale))
    return adjusted


def infer_room(point_xy):
    containing_room = find_containing_room(point_xy)
    if containing_room is not None:
        return containing_room

    room_name, _ = min(
        ROOMS.items(),
        key=lambda item: math.hypot(
            point_xy[0] - compute_centroid(item[1]["polygon"])[0],
            point_xy[1] - compute_centroid(item[1]["polygon"])[1],
        )
    )
    rospy.logwarn(
        "当前位置 x=%.2f, y=%.2f 未严格落在任一房间内，按最近房间 %s 处理。",
        point_xy[0],
        point_xy[1],
        ROOMS[room_name]["label"],
    )
    return room_name


def find_containing_room(point_xy):
    for room_name, room_info in ROOMS.items():
        if point_in_polygon(point_xy, room_info["polygon"]):
            return room_name
    return None


def build_room_waypoints(room_name, start_xy):
    room_info = ROOMS[room_name]
    polygon_boundary = offset_polygon_inward(room_info["polygon"], WALL_MARGIN)
    rospy.loginfo(
        "%s 应用 %.2fm 墙边安全余量后的边界: %s",
        room_info["label"],
        WALL_MARGIN,
        polygon_boundary,
    )

    waypoints = generate_polygon_snake_path(polygon_boundary, ROOM_SNAKE_COLUMNS)
    rospy.loginfo(
        "%s 使用 %d 列蛇形扫描，生成了 %d 个原始航点。",
        room_info["label"],
        ROOM_SNAKE_COLUMNS,
        len(waypoints),
    )
    waypoints = select_best_path_direction(waypoints, start_xy)
    waypoints = remove_close_waypoints(waypoints, MIN_WAYPOINT_SPACING)
    rospy.loginfo(
        "%s 去除过密点后的航点数: %d",
        room_info["label"],
        len(waypoints),
    )
    return waypoints


def choose_best_door(from_room_name, to_room_name, from_waypoints, to_waypoints):
    if not from_waypoints or not to_waypoints:
        return DOORS[0]

    exit_point = from_waypoints[-1]
    entry_point = to_waypoints[0]

    best_door = min(
        DOORS,
        key=lambda door: (
            math.hypot(exit_point[0] - door["center"][0], exit_point[1] - door["center"][1]) +
            math.hypot(entry_point[0] - door["center"][0], entry_point[1] - door["center"][1])
        )
    )
    rospy.loginfo(
        "在 %s -> %s 过渡时选择 %s，门中心 x=%.2f, y=%.2f。",
        ROOMS[from_room_name]["label"],
        ROOMS[to_room_name]["label"],
        best_door["label"],
        best_door["center"][0],
        best_door["center"][1],
    )
    return best_door


def build_door_transition_path(door, from_room_name, to_room_name):
    door_x, door_y = door["center"]
    moving_up = compute_centroid(ROOMS[to_room_name]["polygon"])[1] > compute_centroid(ROOMS[from_room_name]["polygon"])[1]
    direction_sign = 1.0 if moving_up else -1.0
    yaw = math.pi / 2.0 if moving_up else -math.pi / 2.0

    transition_path = [
        (door_x, door_y - direction_sign * DOOR_APPROACH_OFFSET),
        (door_x, door_y),
        (door_x, door_y + direction_sign * DOOR_APPROACH_OFFSET),
    ]
    rospy.loginfo(
        "生成 %s 的过门序列，共 %d 个点，朝向 %.2f。",
        door["label"],
        len(transition_path),
        yaw,
    )
    return transition_path, yaw

def densify_waypoints_with_flags(waypoints, max_segment_length):
    """
    Split long path segments into shorter sub-goals so move_base is not asked to
    jump across a large portion of the floor in one shot.
    Only the original waypoints keep a True flag; inserted interpolation points
    are marked False so relocalization can be restricted to waypoint stops.
    """
    if len(waypoints) < 2:
        return waypoints, [True] * len(waypoints)

    dense_waypoints = [waypoints[0]]
    original_flags = [True]
    for start, end in zip(waypoints[:-1], waypoints[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.hypot(dx, dy)
        segments = max(1, int(math.ceil(distance / max_segment_length)))
        for step in range(1, segments + 1):
            ratio = float(step) / float(segments)
            dense_waypoints.append((
                start[0] + dx * ratio,
                start[1] + dy * ratio,
            ))
            original_flags.append(step == segments)
    return dense_waypoints, original_flags

def remove_close_waypoints(waypoints, min_spacing):
    """
    Drop nearly overlapping consecutive waypoints so the robot does not keep
    braking for tiny target updates.
    """
    if len(waypoints) < 2:
        return waypoints

    filtered = [waypoints[0]]
    for waypoint in waypoints[1:]:
        prev_x, prev_y = filtered[-1]
        distance = math.hypot(waypoint[0] - prev_x, waypoint[1] - prev_y)
        if distance >= min_spacing:
            filtered.append(waypoint)

    if filtered[-1] != waypoints[-1]:
        filtered.append(waypoints[-1])
    return filtered

def select_best_path_direction(waypoints, start_xy):
    """
    Start from the waypoint closest to the robot, then traverse toward the nearer end
    first to avoid a large initial jump from the spawn room.
    """
    if not waypoints:
        return waypoints

    nearest_idx = min(
        range(len(waypoints)),
        key=lambda i: math.hypot(waypoints[i][0] - start_xy[0], waypoints[i][1] - start_xy[1])
    )
    nearest_wp = waypoints[nearest_idx]
    rospy.loginfo(
        "最近航点: idx=%d, x=%.2f, y=%.2f",
        nearest_idx,
        nearest_wp[0],
        nearest_wp[1],
    )

    dist_to_head = nearest_idx
    dist_to_tail = len(waypoints) - 1 - nearest_idx

    if dist_to_tail <= dist_to_head:
        rospy.loginfo("从最近航点开始，先沿当前方向巡航。")
        return waypoints[nearest_idx:] + list(reversed(waypoints[:nearest_idx]))

    rospy.loginfo("从最近航点开始，先反向巡航。")
    reversed_waypoints = list(reversed(waypoints))
    reversed_idx = len(waypoints) - 1 - nearest_idx
    return reversed_waypoints[reversed_idx:] + list(reversed(reversed_waypoints[:reversed_idx]))

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


def build_goal(x, y, yaw):
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


def send_goal(client, x, y, yaw, timeout_sec=50, clear_costmaps_service=None):
    """
    发送导航目标并监听结果
    """
    for attempt in range(RECOVERY_RETRY_LIMIT + 1):
        goal = build_goal(x, y, yaw)
        rospy.loginfo(
            "前往: x=%.2f, y=%.2f, 朝向=%.2f 弧度 (尝试 %d/%d)",
            x, y, yaw, attempt + 1, RECOVERY_RETRY_LIMIT + 1
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

                q = current_pose.orientation
                current_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                angle_diff = abs(math.atan2(math.sin(current_yaw - yaw), math.cos(current_yaw - yaw)))
                rospy.loginfo("距离目标: %.2f, 朝向差: %.2f", distance, angle_diff)

                if best_distance - distance >= MIN_PROGRESS_DISTANCE:
                    best_distance = distance
                    last_progress_time = rospy.Time.now()
                elif best_distance == float("inf"):
                    best_distance = distance
                    last_progress_time = rospy.Time.now()

                if distance <= POSITION_GOAL_TOLERANCE and angle_diff <= ORIENTATION_GOAL_TOLERANCE:
                    rospy.loginfo("达到位置与朝向容差，取消目标并继续。")
                    client.cancel_goal()
                    return True, "reached"

                if distance <= POSITION_GOAL_TOLERANCE:
                    rospy.loginfo("位置已到达，忽略最终朝向并继续。")
                    client.cancel_goal()
                    return True, "reached"

                if rospy.Time.now() - last_progress_time > rospy.Duration(NO_PROGRESS_TIMEOUT_SEC):
                    rospy.logwarn(
                        "连续 %.1f 秒没有明显接近目标，视为被障碍阻塞。",
                        NO_PROGRESS_TIMEOUT_SEC,
                    )
                    failure_reason = "no_progress"
                    break

            state = client.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("成功到达！")
                return True, "succeeded"
            if state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                rospy.logwarn("导航失败 (状态码: %d)。", state)
                failure_reason = "planner_failed"
                break

            if timeout_sec is not None and (rospy.Time.now() - start_time > rospy.Duration(timeout_sec)):
                rospy.logwarn("导航超时。")
                failure_reason = "timeout"
                break

            rate.sleep()

        client.cancel_goal()
        if attempt >= RECOVERY_RETRY_LIMIT:
            return False, failure_reason or "failed"

        rospy.logwarn("开始恢复流程: 清空 costmap 后重试当前目标。")
        call_clear_costmaps(clear_costmaps_service)
        rospy.sleep(RECOVERY_WAIT_SEC)

    return False, "failed"


def count_blocked_future_waypoints(full_path, start_index, blocked_center, blocked_radius):
    blocked_count = 0
    for waypoint_x, waypoint_y in full_path[start_index:]:
        if math.hypot(waypoint_x - blocked_center[0], waypoint_y - blocked_center[1]) <= blocked_radius:
            blocked_count += 1
        else:
            break
    return blocked_count


def navigate_transition_waypoints(client, transition_waypoints, yaw):
    """
    Navigate through a short sequence of door-aligned sub-goals.
    """
    for index, (x, y) in enumerate(transition_waypoints, 1):
        rospy.loginfo(
            "尝试门口过渡点 %d/%d: x=%.2f, y=%.2f, yaw=%.2f",
            index, len(transition_waypoints), x, y, yaw
        )
        success, _ = send_goal(client, x, y, yaw, timeout_sec=35)
        if success:
            if TRANSITION_SETTLE_TIME > 0.0:
                rospy.sleep(TRANSITION_SETTLE_TIME)
            continue
        rospy.logwarn("门口过渡点 %d 失败，继续尝试下一个。", index)
    return current_pose is not None and math.hypot(
        current_pose.position.x - transition_waypoints[-1][0],
        current_pose.position.y - transition_waypoints[-1][1],
    ) <= 1.0

def main():
    rospy.init_node("polygon_snake_nav")
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
    initial_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
    clear_costmaps_service = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
    
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("等待 move_base...")
    client.wait_for_server()
    rospy.loginfo("连接成功！")
    rospy.sleep(1.0)

    amcl_msg = rospy.wait_for_message("amcl_pose", PoseWithCovarianceStamped)
    start_xy = (
        amcl_msg.pose.pose.position.x,
        amcl_msg.pose.pose.position.y,
    )
    rospy.loginfo(f"当前机器人位置: x={start_xy[0]:.2f}, y={start_xy[1]:.2f}")

    start_room = infer_room(start_xy)
    remaining_rooms = [room_name for room_name in ROOMS if room_name != start_room]
    room_visit_order = [start_room] + remaining_rooms
    rospy.loginfo(
        "房间扫描顺序: %s",
        " -> ".join(ROOMS[room_name]["label"] for room_name in room_visit_order),
    )

    room_waypoints = {}
    reference_xy = start_xy
    for room_name in room_visit_order:
        room_waypoints[room_name] = build_room_waypoints(room_name, reference_xy)
        if room_waypoints[room_name]:
            reference_xy = room_waypoints[room_name][-1]

    full_path = []
    for index, room_name in enumerate(room_visit_order):
        current_room_waypoints = room_waypoints[room_name]
        if not current_room_waypoints:
            rospy.logwarn("%s 没有生成可用航点，跳过。", ROOMS[room_name]["label"])
        full_path.extend(current_room_waypoints)

        if index == len(room_visit_order) - 1:
            continue

        next_room = room_visit_order[index + 1]
        door = choose_best_door(
            room_name,
            next_room,
            current_room_waypoints,
            room_waypoints[next_room],
        )
        transition_path, _ = build_door_transition_path(door, room_name, next_room)
        room_waypoints[next_room] = build_room_waypoints(next_room, transition_path[-1])
        full_path.extend(transition_path)

    if not full_path:
        rospy.logwarn("未生成任何巡航航点，停止执行。")
        return

    original_waypoint_count = len(full_path)
    full_path, original_waypoint_flags = densify_waypoints_with_flags(full_path, MAX_SEGMENT_LENGTH)
    rospy.loginfo(f"平滑后的总巡航航点数: {len(full_path)}")
    rospy.loginfo(
        "原始航点数: %d，仅在到达原始航点后允许校正，行驶过程中不发布重定位；当前配置为每个原始航点都校正一次。",
        original_waypoint_count,
    )

    # 开始巡回
    consecutive_failures = 0
    reached_original_waypoints = 0
    i = 0
    while i < len(full_path):
        x, y = full_path[i]
        is_original_waypoint = original_waypoint_flags[i]
        
        # 预判车头朝向
        if i < len(full_path) - 1:
            next_x, next_y = full_path[i+1]
            yaw = math.atan2(next_y - y, next_x - x)
        else:
            yaw = 0.0 

        rospy.loginfo(f"--- 进度: {i+1}/{len(full_path)} ---")
        success, reason = send_goal(
            client,
            x,
            y,
            yaw,
            timeout_sec=GOAL_TIMEOUT_SEC,
            clear_costmaps_service=clear_costmaps_service,
        )
        if not success:
            consecutive_failures += 1
            blocked_center = (x, y)
            if current_pose is not None:
                blocked_center = (current_pose.position.x, current_pose.position.y)
            blocked_count = count_blocked_future_waypoints(
                full_path,
                i,
                blocked_center,
                BLOCKED_WAYPOINT_RADIUS,
            )
            blocked_count = max(1, blocked_count)
            rospy.logwarn(
                "航点 %d 导航失败，原因=%s。将跳过该阻塞区附近 %d 个航点。连续失败次数: %d/%d",
                i + 1, reason, blocked_count, consecutive_failures, MAX_CONSECUTIVE_NAV_FAILURES
            )
            if consecutive_failures >= MAX_CONSECUTIVE_NAV_FAILURES:
                rospy.logwarn("连续多个航点失败，停止本次蛇形巡航。")
                return
            i += blocked_count
            continue
        consecutive_failures = 0
        if is_original_waypoint:
            reached_original_waypoints += 1
        if (
            ENABLE_WAYPOINT_RELOCALIZATION and
            is_original_waypoint and
            (reached_original_waypoints % RELOCALIZE_EVERY_N_WAYPOINTS == 0)
        ):
            maybe_relocalize_at_checkpoint(
                x,
                y,
                yaw,
                initial_pose_pub,
                "原始航点 %d" % reached_original_waypoints,
            )
        elif CRUISE_SETTLE_TIME > 0.0:
            rospy.sleep(CRUISE_SETTLE_TIME)
        i += 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
