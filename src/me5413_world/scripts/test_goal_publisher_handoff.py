#!/usr/bin/env python3
import os
import subprocess
import sys
import time

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, Int16


HANDOFF_X = 1.73315
HANDOFF_Y = 7.41871
HANDOFF_QZ = -0.02015332029956148
HANDOFF_QW = 0.9997969012158936

AUTO_SEQUENCE_STATE_TOPIC = "/me5413_world/auto_sequence_state"
START_AUTO_SEQUENCE_TOPIC = "/me5413_world/start_auto_sequence"
EXPECTED_DIGIT_TOPIC = "/me5413_world/expected_digit"
AUTO_SEQUENCE_STATE_NAMES = {
    0: "IDLE",
    1: "WAITING_FOR_RESPAWN",
    2: "NAVIGATING_TO_FIRST",
    3: "NAVIGATING_TO_SECOND",
    4: "NAVIGATING_TO_INTERMEDIATE",
    5: "NAVIGATING_TO_PRE_FINAL",
    6: "NAVIGATING_TO_INSPECTION_POINTS",
    7: "NAVIGATING_TO_MATCHED_TARGET",
}


def build_child_env():
    env = os.environ.copy()
    env["PYTHONNOUSERSITE"] = "1"
    return env


def launch_goal_publisher_if_needed():
    try:
        rospy.wait_for_message(AUTO_SEQUENCE_STATE_TOPIC, Int16, timeout=1.5)
        rospy.loginfo("Detected an existing goal_publisher_node state publisher.")
        return None
    except rospy.ROSException:
        rospy.loginfo("goal_publisher_node is not running yet, launching it now.")
        return subprocess.Popen(["rosrun", "me5413_world", "goal_publisher_node"], env=build_child_env())


def navigate_to_handoff_pose():
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    if not client.wait_for_server(rospy.Duration(30.0)):
        raise RuntimeError("move_base action server is unavailable.")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = HANDOFF_X
    goal.target_pose.pose.position.y = HANDOFF_Y
    goal.target_pose.pose.orientation.z = HANDOFF_QZ
    goal.target_pose.pose.orientation.w = HANDOFF_QW

    rospy.loginfo(
        "Navigating to handoff pose: x=%.5f, y=%.5f, qz=%.8f, qw=%.8f",
        HANDOFF_X,
        HANDOFF_Y,
        HANDOFF_QZ,
        HANDOFF_QW,
    )
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(180.0)):
        client.cancel_goal()
        raise RuntimeError("Timed out while navigating to the handoff pose.")

    state = client.get_state()
    rospy.loginfo("move_base finished with state=%d", state)


def trigger_and_check(expected_digit=None):
    trigger_pub = rospy.Publisher(START_AUTO_SEQUENCE_TOPIC, Bool, queue_size=1, latch=True)
    expected_digit_pub = rospy.Publisher(EXPECTED_DIGIT_TOPIC, Int16, queue_size=1, latch=True)
    rospy.sleep(1.0)

    if expected_digit is not None:
        rospy.loginfo("Publishing expected digit %d on %s", expected_digit, EXPECTED_DIGIT_TOPIC)
        expected_digit_pub.publish(Int16(data=expected_digit))
        rospy.sleep(0.5)

    rospy.loginfo("Publishing handoff trigger on %s", START_AUTO_SEQUENCE_TOPIC)
    trigger_pub.publish(Bool(data=True))

    deadline = time.time() + 20.0
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message(AUTO_SEQUENCE_STATE_TOPIC, Int16, timeout=2.0)
        except rospy.ROSException:
            rospy.loginfo("Still waiting for /me5413_world/auto_sequence_state ...")
            continue

        rospy.loginfo("goal_publisher_node state=%d", msg.data)
        if msg.data != 0:
            rospy.loginfo("Success: goal_publisher_node accepted the trigger.")
            return True

    return False


def wait_for_auto_sequence_to_finish(timeout_sec=180.0):
    deadline = time.time() + timeout_sec
    started = False
    last_state = None

    while time.time() < deadline and not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message(AUTO_SEQUENCE_STATE_TOPIC, Int16, timeout=5.0)
        except rospy.ROSException:
            rospy.loginfo("Still waiting for /me5413_world/auto_sequence_state updates ...")
            continue

        state = msg.data
        if state != last_state:
            rospy.loginfo(
                "goal_publisher_node state=%d (%s)",
                state,
                AUTO_SEQUENCE_STATE_NAMES.get(state, "UNKNOWN"),
            )
            last_state = state

        if state != 0:
            started = True

        if started and state == 0:
            rospy.loginfo("goal_publisher_node returned to IDLE. The follow-up mission is complete.")
            return True

    return False


def main():
    rospy.init_node("test_goal_publisher_handoff", anonymous=True)
    goal_publisher_process = launch_goal_publisher_if_needed()
    keep_goal_publisher_running = False

    try:
        if goal_publisher_process is not None:
            rospy.sleep(2.0)

        navigate_to_handoff_pose()
        if trigger_and_check(expected_digit=6):
            keep_goal_publisher_running = True
            rospy.loginfo("Handoff trigger accepted. Continuing to observe the follow-up mission...")
            if wait_for_auto_sequence_to_finish(timeout_sec=300.0):
                rospy.loginfo("Handoff test completed successfully.")
            else:
                rospy.logwarn("Handoff was accepted, but the follow-up mission did not finish before timeout.")
        else:
            rospy.logerr("Handoff test failed: goal_publisher_node stayed in IDLE.")
            sys.exit(1)
    finally:
        if (
            goal_publisher_process is not None
            and goal_publisher_process.poll() is None
            and not keep_goal_publisher_running
        ):
            goal_publisher_process.terminate()
            try:
                goal_publisher_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                goal_publisher_process.kill()


if __name__ == "__main__":
    main()
