#!/usr/bin/env python3
# This script implements a state machine that manages the execution of multiple ROS/Python scripts.
# Some scripts are run continuously (and auto-restarted if they exit) whereas others are run once.
# It uses subprocesses to start the scripts and threads to manage their continuous execution.

import os
import re
import subprocess      # For launching external scripts as subprocesses
import threading       # For running functions concurrently in separate threads
import time            # For sleep functions and timing operations
import sys             # To access the current Python interpreter executable
import math

# Detection region for the first floor.
FINDCUBE_REGION_ARGS = [
    "_map_yaml:=/home/xu/ME5413_Final_Project-1/src/me5413_world/maps/manual_cut_direct_2d_map_v3_thick.yaml",
    "_region_min_x:=-17.5",
    "_region_max_x:=3.0",
    "_region_min_y:=2.0",
    "_region_max_y:=22.5",
]

PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
GOAL_PUBLISHER_HANDOFF_X = 1.51
GOAL_PUBLISHER_HANDOFF_Y = 6.15
GOAL_PUBLISHER_HANDOFF_YAW = 0.0
AUTO_SEQUENCE_STATE_TOPIC = "/me5413_world/auto_sequence_state"
EXPECTED_DIGIT_TOPIC = "/me5413_world/expected_digit"
START_AUTO_SEQUENCE_TOPIC = "/me5413_world/start_auto_sequence"
RESULT_FILE = "recognized_digits_summary.txt"
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

SCRIPT_PATHS = {
    "digit_recognition_service_node.py": os.path.join(PACKAGE_ROOT, "src", "digit_recognition_service_node.py"),
    "findcube.py": os.path.join(PACKAGE_ROOT, "scripts", "findcube.py"),
    "snake_path.py": os.path.join(PACKAGE_ROOT, "scripts", "snake_path.py"),
    "subscribe_box_pos.py": os.path.join(PACKAGE_ROOT, "scripts", "subscribe_box_pos.py"),
}


def resolve_script_path(script):
    path = SCRIPT_PATHS.get(script, script)
    if not os.path.isfile(path):
        raise FileNotFoundError("Cannot find script: {}".format(path))
    return path


def build_child_env():
    env = os.environ.copy()
    # Prefer the system ROS Python environment over user-site packages so
    # apt-installed sklearn stays compatible with apt-installed numpy/scipy.
    env["PYTHONNOUSERSITE"] = "1"
    return env


def launch_goal_publisher_node():
    print("Starting goal_publisher_node ...")
    return subprocess.Popen(["rosrun", "me5413_world", "goal_publisher_node"], env=build_child_env())


def extract_expected_digit():
    candidate_paths = [
        os.path.join(os.getcwd(), RESULT_FILE),
        os.path.join(os.path.dirname(__file__), RESULT_FILE),
    ]
    for path in candidate_paths:
        if not os.path.isfile(path):
            continue
        with open(path, "r") as result_file:
            content = result_file.read()
        match = re.search(r"least_frequent_digits=\[([^\]]*)\]", content)
        if not match:
            continue
        digits_text = match.group(1).strip()
        if not digits_text:
            continue
        digits = [int(part.strip()) for part in digits_text.split(",") if part.strip()]
        if not digits:
            continue
        if len(digits) > 1:
            print(f"Multiple least-frequent digits found in {path}: {digits}. Using the first one.")
        print(f"Using expected digit {digits[0]} from {path}.")
        return digits[0]
    raise RuntimeError("Could not determine the expected digit from recognized_digits_summary.txt.")


def switch_digit_recognizer(mode):
    import rospy
    from std_srvs.srv import Trigger

    if not rospy.core.is_initialized():
        rospy.init_node("start_robot_recognizer_switch", anonymous=True, disable_signals=True)

    service_name = {
        "firstfloor": "/switch_to_firstfloor_recognizer",
        "secondfloor": "/switch_to_secondfloor_recognizer",
    }.get(mode)
    if service_name is None:
        raise RuntimeError("Unknown recognizer switch mode: {}".format(mode))

    print(f"Switching digit recognizer to {mode} via {service_name} ...")
    rospy.wait_for_service(service_name, timeout=10.0)
    switch_srv = rospy.ServiceProxy(service_name, Trigger)
    response = switch_srv()
    if not response.success:
        raise RuntimeError(f"Failed to switch digit recognizer to {mode}: {response.message}")
    print(response.message)


def publish_initial_pose(x, y, yaw, publisher):
    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from tf.transformations import quaternion_from_euler

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = 0.0

    quaternion = quaternion_from_euler(0, 0, yaw)
    msg.pose.pose.orientation.x = quaternion[0]
    msg.pose.pose.orientation.y = quaternion[1]
    msg.pose.pose.orientation.z = quaternion[2]
    msg.pose.pose.orientation.w = quaternion[3]

    msg.pose.covariance[0] = 0.20
    msg.pose.covariance[7] = 0.20
    msg.pose.covariance[35] = 0.10
    publisher.publish(msg)
    print(f"Published handoff relocalization /initialpose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")


def relocalize_at_handoff_pose():
    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from tf.transformations import euler_from_quaternion

    initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    rospy.sleep(1.0)

    relocalize_yaw = GOAL_PUBLISHER_HANDOFF_YAW
    try:
        amcl_msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=2.0)
        q = amcl_msg.pose.pose.orientation
        relocalize_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        yaw_error = math.atan2(
            math.sin(relocalize_yaw - GOAL_PUBLISHER_HANDOFF_YAW),
            math.cos(relocalize_yaw - GOAL_PUBLISHER_HANDOFF_YAW),
        )
        print(
            "Using current AMCL yaw for handoff relocalization: "
            f"current_yaw={relocalize_yaw:.2f}, handoff_yaw={GOAL_PUBLISHER_HANDOFF_YAW:.2f}, "
            f"yaw_error={abs(yaw_error):.2f}"
        )
    except rospy.ROSException:
        print(
            "Timed out waiting for /amcl_pose before handoff relocalization. "
            "Falling back to the nominal handoff yaw."
        )

    publish_initial_pose(
        GOAL_PUBLISHER_HANDOFF_X,
        GOAL_PUBLISHER_HANDOFF_Y,
        relocalize_yaw,
        initial_pose_pub,
    )
    rospy.sleep(1.0)


def run_goal_publisher_handoff():
    import actionlib
    import rospy
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from std_msgs.msg import Bool, Int16
    from tf.transformations import quaternion_from_euler

    if not rospy.core.is_initialized():
        rospy.init_node("start_robot_handoff", anonymous=True, disable_signals=True)

    expected_digit = extract_expected_digit()

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    print("Waiting for move_base action server for goal_publisher handoff ...")
    if not client.wait_for_server(rospy.Duration(30.0)):
        raise RuntimeError("move_base action server is unavailable for the goal_publisher handoff.")

    handoff_goal = MoveBaseGoal()
    handoff_goal.target_pose.header.frame_id = "map"
    handoff_goal.target_pose.header.stamp = rospy.Time.now()
    handoff_goal.target_pose.pose.position.x = GOAL_PUBLISHER_HANDOFF_X
    handoff_goal.target_pose.pose.position.y = GOAL_PUBLISHER_HANDOFF_Y
    q = quaternion_from_euler(0, 0, GOAL_PUBLISHER_HANDOFF_YAW)
    handoff_goal.target_pose.pose.orientation.x = q[0]
    handoff_goal.target_pose.pose.orientation.y = q[1]
    handoff_goal.target_pose.pose.orientation.z = q[2]
    handoff_goal.target_pose.pose.orientation.w = q[3]

    print(
        "Navigating to goal_publisher handoff pose "
        f"({GOAL_PUBLISHER_HANDOFF_X:.2f}, {GOAL_PUBLISHER_HANDOFF_Y:.2f}, {GOAL_PUBLISHER_HANDOFF_YAW:.2f}) ..."
    )
    client.send_goal(handoff_goal)
    if not client.wait_for_result(rospy.Duration(180.0)):
        client.cancel_goal()
        raise RuntimeError("Timed out while navigating to the goal_publisher handoff pose.")

    relocalize_at_handoff_pose()

    expected_digit_pub = rospy.Publisher(EXPECTED_DIGIT_TOPIC, Int16, queue_size=1, latch=True)
    start_sequence_pub = rospy.Publisher(START_AUTO_SEQUENCE_TOPIC, Bool, queue_size=1, latch=True)
    rospy.sleep(1.0)

    expected_digit_pub.publish(Int16(data=expected_digit))
    print(f"Published expected digit {expected_digit} to {EXPECTED_DIGIT_TOPIC}.")
    rospy.sleep(0.5)

    print(f"Publishing start trigger on {START_AUTO_SEQUENCE_TOPIC} ...")
    handoff_started = False
    for attempt in range(1, 11):
        start_sequence_pub.publish(Bool(data=True))
        try:
            state_msg = rospy.wait_for_message(AUTO_SEQUENCE_STATE_TOPIC, Int16, timeout=2.0)
        except rospy.ROSException:
            continue
        if state_msg.data != 0:
            handoff_started = True
            print(f"goal_publisher_node accepted the handoff trigger on attempt {attempt}.")
            break

    if not handoff_started:
        raise RuntimeError("goal_publisher_node did not leave IDLE after repeated handoff triggers.")

    print("Waiting for goal_publisher_node to complete the post-handoff mission ...")
    started = True
    deadline = time.time() + 900.0
    last_reported_state = None
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            state_msg = rospy.wait_for_message(AUTO_SEQUENCE_STATE_TOPIC, Int16, timeout=5.0)
        except rospy.ROSException:
            print("Still waiting for /me5413_world/auto_sequence_state updates ...")
            continue
        state = state_msg.data
        if state != last_reported_state:
            state_name = AUTO_SEQUENCE_STATE_NAMES.get(state, f"UNKNOWN_{state}")
            print(f"goal_publisher_node state: {state} ({state_name})")
            last_reported_state = state
        if state != 0:
            started = True
        if started and state == 0:
            print("goal_publisher_node reported the auto sequence is complete.")
            return

    raise RuntimeError("Timed out while waiting for goal_publisher_node to finish the post-handoff mission.")

def run_continuous(script, stop_event, extra_args=None):
    """
    Continuously runs the specified script.
    If the script exits unexpectedly, it is automatically restarted until the stop_event is set.
    
    Parameters:
      script (str): The filename of the script to run (e.g., "findcube.py").
      stop_event (threading.Event): A threading event used to signal when to stop running the script.
      extra_args (list[str] | None): Optional ROS-style private parameters passed to the script.
    """
    if extra_args is None:
        extra_args = []

    # Loop until the stop_event has been triggered.
    while not stop_event.is_set():
        script_path = resolve_script_path(script)
        # Launch the script using the current Python interpreter.
        process = subprocess.Popen([sys.executable, script_path] + extra_args, env=build_child_env())
        print(f"Starting {script_path} {' '.join(extra_args)} ...".strip())
        
        # Continuously check whether the process is still running.
        while process.poll() is None:
            # If a stop signal is received during execution...
            if stop_event.is_set():
                # Terminate the running process gracefully.
                process.terminate()
                print(f"Terminating {script_path} ...")
                break
            # Wait for 1 second before checking again.
            time.sleep(1)
        
        # If the process exited on its own and the stop_event isn't set, restart after a delay.
        if not stop_event.is_set():
            print(f"{script_path} exited, restarting in 1 second...")
            time.sleep(1)

def run_once(script):
    """
    Runs a script a single time and waits until it completes.
    
    Parameters:
      script (str): The filename of the script to execute.
    """
    script_path = resolve_script_path(script)
    print(f"Starting {script_path} ...")
    # Launch the script as a subprocess.
    process = subprocess.Popen([sys.executable, script_path], env=build_child_env())
    # Wait for the script to finish execution.
    process.wait()
    print(f"{script} completed.")

def main():
    """
    Main function that defines the various states of the overall process.
    It orchestrates the continuous and one-time execution of various scripts.
    """
    goal_publisher_process = None
    # State: Continuously run digit_recognition_service_node.py
    stop_event_digit = threading.Event()  # Create an event to signal when to stop the script
    thread_digit = threading.Thread(
        target=run_continuous,
        args=("digit_recognition_service_node.py", stop_event_digit),
        daemon=True   # Daemon thread to ensure it exits when the main program ends
    )
    thread_digit.start()  # Start the continuous execution thread for digit_recognition_service_node.py

    # State 1: Continuously run findcube.py
    stop_event_findcube = threading.Event()  # Event for findcube.py
    thread_findcube = threading.Thread(
        target=run_continuous,
        args=("findcube.py", stop_event_findcube, FINDCUBE_REGION_ARGS),
        daemon=True
    )
    thread_findcube.start()  # Start findcube.py as a continuously running script

    time.sleep(1)  # Wait for 1 second to allow findcube.py to fully start
    try:
        switch_digit_recognizer("firstfloor")

        # State 2: Run snake_path.py one time (only once)
        run_once("snake_path.py")

        # State 3: Run subscribe_box_pos.py one time (only once)
        run_once("subscribe_box_pos.py")

        # findcube is no longer needed once the block list and digit summary are ready.
        stop_event_findcube.set()
        thread_findcube.join()

        switch_digit_recognizer("secondfloor")

        # State 4: Launch goal_publisher_node and hand off to the second mission stage.
        goal_publisher_process = launch_goal_publisher_node()
        time.sleep(2)
        run_goal_publisher_handoff()
    finally:
        print("All states completed. Terminating continuously running scripts...")
        stop_event_digit.set()
        stop_event_findcube.set()

        if goal_publisher_process is not None and goal_publisher_process.poll() is None:
            goal_publisher_process.terminate()
            try:
                goal_publisher_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                goal_publisher_process.kill()

        thread_digit.join()
        thread_findcube.join()

    print("State machine completed.")

# Main entry point: run the state machine.
if __name__ == "__main__":
    main()
