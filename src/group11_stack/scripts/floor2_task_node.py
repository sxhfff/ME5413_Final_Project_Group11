#!/usr/bin/env python3
from enum import Enum

import actionlib
import rospy
import tf.transformations as tft
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32, String


class TaskState(Enum):
    WAIT_RESULT = "WAIT_RESULT"
    GO_TO_RAMP = "GO_TO_RAMP"
    ENTER_FLOOR2 = "ENTER_FLOOR2"
    GO_TO_TARGET_ROOM = "GO_TO_TARGET_ROOM"
    FINAL_PARK = "FINAL_PARK"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class Floor2TaskNode:
    def __init__(self):
        self.result_topic = rospy.get_param("~task/result_topic", "/group11/least_box_type")
        self.status_topic = rospy.get_param("~task/status_topic", "/group11/floor2_task/status")
        self.dry_run = rospy.get_param("~task/dry_run", False)
        self.goal_timeout_sec = float(rospy.get_param("~task/goal_timeout_sec", 120.0))
        self.pause_before_final_park_sec = float(
            rospy.get_param("~task/pause_before_final_park_sec", 1.0)
        )
        self.room_goal_retry_limit = int(rospy.get_param("~task/room_goal_retry_limit", 1))
        self.fallback_to_other_entry = bool(rospy.get_param("~task/fallback_to_other_entry", True))
        self.active_map_file = rospy.get_param("~task/active_map_file", "")
        self.box_type_to_room = rospy.get_param("~box_type_to_room", {})
        self.navigation_goals = rospy.get_param("~navigation_goals", {})
        self.room_entry_priority = rospy.get_param("~room_entry_priority", {})

        self.state = TaskState.WAIT_RESULT
        self.least_box_type = None
        self.target_room = None
        self.selected_entry = None
        self.active_goal_name = None
        self.entry_candidates = []
        self.entry_attempt_index = 0
        self.room_goal_attempts = 0

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.result_sub = rospy.Subscriber(self.result_topic, Int32, self.result_callback, queue_size=1)

        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("floor2_task_node: waiting for move_base action server")
        self.move_base_client.wait_for_server()
        rospy.loginfo("floor2_task_node: connected to move_base")

        self.publish_status(
            "state=%s map=%s result_topic=%s"
            % (self.state.value, self.active_map_file, self.result_topic)
        )

    def result_callback(self, msg):
        if self.least_box_type is not None:
            return
        self.least_box_type = int(msg.data)
        self.target_room = self.resolve_target_room(self.least_box_type)
        rospy.loginfo(
            "floor2_task_node: received least_box_type=%d target_room=%s",
            self.least_box_type,
            self.target_room,
        )

    def resolve_target_room(self, box_type):
        key_variants = [box_type, str(box_type)]
        for key in key_variants:
            if key in self.box_type_to_room:
                return self.box_type_to_room[key]
        raise KeyError("No room mapping configured for least_box_type=%s" % box_type)

    def publish_status(self, message):
        self.status_pub.publish(String(data=message))
        rospy.loginfo("floor2_task_node: %s", message)

    def fail_task(self, reason):
        self.state = TaskState.FAILED
        self.publish_status(
            "state=%s least_box_type=%s target_room=%s active_goal=%s reason=%s"
            % (
                self.state.value,
                self.least_box_type,
                self.target_room,
                self.active_goal_name,
                reason,
            )
        )

    def make_pose_goal(self, goal_name):
        if goal_name not in self.navigation_goals:
            raise KeyError("Missing navigation goal '%s'" % goal_name)

        cfg = self.navigation_goals[goal_name]
        q = tft.quaternion_from_euler(0.0, 0.0, cfg["yaw"])

        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = cfg.get("frame_id", "map")
        goal.target_pose.pose.position.x = cfg["x"]
        goal.target_pose.pose.position.y = cfg["y"]
        goal.target_pose.pose.orientation = Quaternion(*q)
        return goal

    def send_goal_and_wait(self, goal_name):
        self.active_goal_name = goal_name
        goal = self.make_pose_goal(goal_name)
        self.publish_status("dispatch_goal=%s state=%s" % (goal_name, self.state.value))

        if self.dry_run:
            rospy.sleep(0.5)
            return True

        self.move_base_client.send_goal(goal)
        finished = self.move_base_client.wait_for_result(rospy.Duration(self.goal_timeout_sec))
        if not finished:
            self.move_base_client.cancel_goal()
            self.publish_status("goal_timeout=%s" % goal_name)
            return False

        status = self.move_base_client.get_state()
        if status == GoalStatus.SUCCEEDED:
            return True

        self.publish_status("goal_failed=%s status=%d" % (goal_name, status))
        return False

    def select_floor2_entry(self):
        preferred = self.room_entry_priority.get(self.target_room, [])
        if preferred:
            return preferred
        return ["floor2_entry_a", "floor2_entry_b"]

    def run_state_machine(self):
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown() and self.state not in {
            TaskState.COMPLETED,
            TaskState.FAILED,
        }:
            try:
                self.step_once()
            except Exception as exc:
                rospy.logerr("floor2_task_node: %s", exc)
                self.fail_task(str(exc))
            rate.sleep()

    def step_once(self):
        if self.state == TaskState.WAIT_RESULT:
            if self.least_box_type is None:
                return
            self.state = TaskState.GO_TO_RAMP
            self.room_goal_attempts = 0
            self.publish_status(
                "state=%s least_box_type=%d target_room=%s"
                % (self.state.value, self.least_box_type, self.target_room)
            )
            return

        if self.state == TaskState.GO_TO_RAMP:
            if not self.send_goal_and_wait("ramp_goal"):
                self.fail_task("failed_to_reach_ramp_goal")
                return
            self.state = TaskState.ENTER_FLOOR2
            self.entry_candidates = self.select_floor2_entry()
            self.entry_attempt_index = 0
            self.selected_entry = self.entry_candidates[self.entry_attempt_index]
            self.publish_status(
                "state=%s selected_entry=%s entry_candidates=%s"
                % (self.state.value, self.selected_entry, self.entry_candidates)
            )
            return

        if self.state == TaskState.ENTER_FLOOR2:
            if not self.send_goal_and_wait(self.selected_entry):
                if (
                    self.fallback_to_other_entry
                    and self.entry_attempt_index + 1 < len(self.entry_candidates)
                ):
                    failed_entry = self.selected_entry
                    self.entry_attempt_index += 1
                    self.selected_entry = self.entry_candidates[self.entry_attempt_index]
                    self.publish_status(
                        "state=%s failed_entry=%s fallback_entry=%s"
                        % (self.state.value, failed_entry, self.selected_entry)
                    )
                    return
                self.fail_task("failed_to_enter_floor2")
                return
            self.state = TaskState.GO_TO_TARGET_ROOM
            self.room_goal_attempts = 0
            self.publish_status(
                "state=%s target_room=%s via_entry=%s"
                % (self.state.value, self.target_room, self.selected_entry)
            )
            return

        if self.state == TaskState.GO_TO_TARGET_ROOM:
            if not self.send_goal_and_wait(self.target_room):
                self.room_goal_attempts += 1
                if self.room_goal_attempts <= self.room_goal_retry_limit:
                    self.publish_status(
                        "state=%s retry_target_room=%s attempt=%d/%d"
                        % (
                            self.state.value,
                            self.target_room,
                            self.room_goal_attempts,
                            self.room_goal_retry_limit,
                        )
                    )
                    return
                self.fail_task("failed_to_reach_target_room")
                return
            self.state = TaskState.FINAL_PARK
            self.publish_status(
                "state=%s target_room=%s attempts=%d"
                % (self.state.value, self.target_room, self.room_goal_attempts + 1)
            )
            return

        if self.state == TaskState.FINAL_PARK:
            rospy.sleep(self.pause_before_final_park_sec)
            self.state = TaskState.COMPLETED
            self.publish_status(
                "state=%s least_box_type=%d target_room=%s final_goal=%s"
                % (
                    self.state.value,
                    self.least_box_type,
                    self.target_room,
                    self.active_goal_name,
                )
            )


if __name__ == "__main__":
    rospy.init_node("floor2_task_node")
    node = Floor2TaskNode()
    node.run_state_machine()
