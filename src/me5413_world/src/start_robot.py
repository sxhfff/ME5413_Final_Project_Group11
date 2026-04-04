#!/usr/bin/env python
# This script implements a state machine that manages the execution of multiple ROS/Python scripts.
# Some scripts are run continuously (and auto-restarted if they exit) whereas others are run once.
# It uses subprocesses to start the scripts and threads to manage their continuous execution.

import subprocess      # For launching external scripts as subprocesses
import threading       # For running functions concurrently in separate threads
import time            # For sleep functions and timing operations
import sys             # To access the current Python interpreter executable

# Detection region for the first floor.
FINDCUBE_REGION_ARGS = [
    "_region_min_x:=-2.0",
    "_region_max_x:=23.0",
    "_region_min_y:=-2.0",
    "_region_max_y:=22.5",
]

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
        # Launch the script using the current Python interpreter.
        process = subprocess.Popen([sys.executable, script] + extra_args)
        print(f"Starting {script} {' '.join(extra_args)} ...".strip())
        
        # Continuously check whether the process is still running.
        while process.poll() is None:
            # If a stop signal is received during execution...
            if stop_event.is_set():
                # Terminate the running process gracefully.
                process.terminate()
                print(f"Terminating {script} ...")
                break
            # Wait for 1 second before checking again.
            time.sleep(1)
        
        # If the process exited on its own and the stop_event isn't set, restart after a delay.
        if not stop_event.is_set():
            print(f"{script} exited, restarting in 1 second...")
            time.sleep(1)

def run_once(script):
    """
    Runs a script a single time and waits until it completes.
    
    Parameters:
      script (str): The filename of the script to execute.
    """
    print(f"Starting {script} ...")
    # Launch the script as a subprocess.
    process = subprocess.Popen([sys.executable, script])
    # Wait for the script to finish execution.
    process.wait()
    print(f"{script} completed.")

def main():
    """
    Main function that defines the various states of the overall process.
    It orchestrates the continuous and one-time execution of various scripts.
    """
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

    # State 2: Run snake_path.py one time (only once)
    run_once("snake_path.py")

    # State 3: Run subscribe_box_pos.py one time (only once)
    run_once("subscribe_box_pos.py")

    # Terminate all continuously running scripts.
    print("All states completed. Terminating continuously running scripts...")
    stop_event_digit.set()       # Stop digit_recognition_service_node.py
    stop_event_findcube.set()    # Stop findcube.py

    # Wait for the threads to complete cleanly.
    thread_digit.join()
    thread_findcube.join()

    print("State machine completed.")

# Main entry point: run the state machine.
if __name__ == "__main__":
    main()
