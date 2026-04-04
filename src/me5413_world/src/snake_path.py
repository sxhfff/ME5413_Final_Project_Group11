#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import math

START_ROOM_POSE = (0.0895, 0.00393)
DOOR_TRANSITION_WAYPOINT = (2.51, 0.588)
TRANSITION_TRIGGER_DISTANCE = 1.0
POSITION_GOAL_TOLERANCE = 0.35
ORIENTATION_GOAL_TOLERANCE = 0.60
MAX_SEGMENT_LENGTH = 1.5
WALL_MARGIN = 1.5
current_pose = None

def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def get_y_intersections(x, polygon):
    """
    计算垂直线 x 与多边形边界的交点，返回该列的 y_min 和 y_max
    """
    intersections = []
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        
        # 检查垂直线是否穿过这条边
        if min(x1, x2) <= x <= max(x1, x2):
            if x1 == x2:  # 如果边本身就是垂直的
                intersections.append(y1)
                intersections.append(y2)
            else:
                # 根据直线方程计算交点的 Y 坐标
                y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
                intersections.append(y)
    
    if not intersections:
        return None, None
        
    # 去重并排序，找出最底端和最顶端
    intersections = sorted(list(set(intersections)))
    return min(intersections), max(intersections)

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
        y_bottom, y_top = get_y_intersections(x, polygon)
        if y_bottom is None or y_top is None:
            continue
            
        # 偶数列：从下往上走
        if idx % 2 == 0:
            waypoints.append((x, y_bottom))
            waypoints.append((x, y_top))
        # 奇数列：从上往下走
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

def densify_waypoints(waypoints, max_segment_length):
    """
    Split long path segments into shorter sub-goals so move_base is not asked to
    jump across a large portion of the floor in one shot.
    """
    if len(waypoints) < 2:
        return waypoints

    dense_waypoints = [waypoints[0]]
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
    return dense_waypoints

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

def send_goal(client, x, y, yaw, timeout_sec=50):
    """
    发送导航目标并监听结果
    """
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

    rospy.loginfo(f"前往: x={x:.2f}, y={y:.2f}, 朝向={yaw:.2f} 弧度")
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
            rospy.loginfo("距离目标: %.2f, 朝向差: %.2f", distance, angle_diff)

            if distance <= POSITION_GOAL_TOLERANCE and angle_diff <= ORIENTATION_GOAL_TOLERANCE:
                rospy.loginfo("达到位置与朝向容差，取消目标并继续。")
                client.cancel_goal()
                return True

            if distance <= POSITION_GOAL_TOLERANCE:
                rospy.loginfo("位置已到达，忽略最终朝向并继续。")
                client.cancel_goal()
                return True

        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("成功到达！")
            return True
        if state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
            rospy.logwarn(f"导航失败 (状态码: {state})。跳过...")
            return False

        if timeout_sec is not None and (rospy.Time.now() - start_time > rospy.Duration(timeout_sec)):
            rospy.logwarn("超时！取消当前目标...")
            client.cancel_goal()
            return False

        rate.sleep()

def main():
    rospy.init_node("polygon_snake_nav")
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)
    
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("等待 move_base...")
    client.wait_for_server()
    rospy.loginfo("连接成功！")

    # 第一层楼的不规则四边形边界（按顺时针顺序填写）。
    # 坐标来自手工测量，保留少量近似误差。
    polygon_boundary = [
        (3.33, -1.52),   # 左下
        (22.3, 3.3),     # 右下
        (17.7, 21.9),    # 右上
        (-1.31, 17.3),   # 左上
    ]
    polygon_boundary = offset_polygon_inward(polygon_boundary, WALL_MARGIN)
    rospy.loginfo("应用 %.2fm 墙边安全余量后的边界: %s", WALL_MARGIN, polygon_boundary)

    # 定义蛇形路线的列数（控制路线的密集程度）
    # 如果觉得漏掉箱子，就把 4 改成 6 或 8；如果觉得扫得太慢，就改小。
    num_columns = 4 

    # 自动计算多边形内的航点
    waypoints = generate_polygon_snake_path(polygon_boundary, num_columns)
    rospy.loginfo(f"根据多边形边界，自动生成了 {len(waypoints)} 个航点。")

    amcl_msg = rospy.wait_for_message("amcl_pose", PoseWithCovarianceStamped)
    start_xy = (
        amcl_msg.pose.pose.position.x,
        amcl_msg.pose.pose.position.y,
    )
    rospy.loginfo(f"当前机器人位置: x={start_xy[0]:.2f}, y={start_xy[1]:.2f}")

    start_room_dist = math.hypot(start_xy[0] - START_ROOM_POSE[0], start_xy[1] - START_ROOM_POSE[1])
    if start_room_dist <= TRANSITION_TRIGGER_DISTANCE:
        rospy.loginfo(
            f"机器人仍在起始房间附近，先前往门口过渡点: x={DOOR_TRANSITION_WAYPOINT[0]:.2f}, y={DOOR_TRANSITION_WAYPOINT[1]:.2f}"
        )
        transition_ok = send_goal(client, DOOR_TRANSITION_WAYPOINT[0], DOOR_TRANSITION_WAYPOINT[1], 0.0, timeout_sec=40)
        if not transition_ok:
            rospy.logwarn("门口过渡点导航失败，停止蛇形扫描。")
            return
        rospy.sleep(0.5)
        amcl_msg = rospy.wait_for_message("amcl_pose", PoseWithCovarianceStamped)
        start_xy = (
            amcl_msg.pose.pose.position.x,
            amcl_msg.pose.pose.position.y,
        )
        rospy.loginfo(f"通过门口后的机器人位置: x={start_xy[0]:.2f}, y={start_xy[1]:.2f}")

    waypoints = select_best_path_direction(waypoints, start_xy)
    waypoints = densify_waypoints(waypoints, MAX_SEGMENT_LENGTH)
    rospy.loginfo(f"细分后的巡航航点数: {len(waypoints)}")

    # 开始巡回
    for i in range(len(waypoints)):
        x, y = waypoints[i]
        
        # 预判车头朝向
        if i < len(waypoints) - 1:
            next_x, next_y = waypoints[i+1]
            yaw = math.atan2(next_y - y, next_x - x)
        else:
            yaw = 0.0 

        rospy.loginfo(f"--- 进度: {i+1}/{len(waypoints)} ---")
        success = send_goal(client, x, y, yaw, timeout_sec=60)
        if not success:
            rospy.logwarn("航点 %d 导航失败，停止本次蛇形巡航。", i + 1)
            return
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
