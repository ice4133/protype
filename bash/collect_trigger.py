#!/usr/bin/env python3
"""
Terminal keyboard trigger for data collection.
Press 's' to start recording, 'd' to stop recording, 'q' to quit.
Works in any terminal without X11/DISPLAY.

Usage: pixi run python3 bash/collect_trigger.py
"""

import sys
import tty
import termios
import rclpy
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from threading import Thread

def getch():
    """Read a single character from terminal without requiring Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def call_service(node, client, name):
    if not client.wait_for_service(timeout_sec=3.0):
        print(f"\r  [{name}] service not available!\r")
        return None
    future = client.call_async(Trigger.Request())
    # spin is running in background thread, just wait on the future
    timeout = 30.0
    start = __import__('time').time()
    while not future.done():
        if __import__('time').time() - start > timeout:
            break
        __import__('time').sleep(0.05)
    result = future.result()
    if result is not None:
        print(f"\r  [{name}] {result.message}\r")
    else:
        print(f"\r  [{name}] call timed out\r")
    return result

def main():
    rclpy.init()
    node = rclpy.create_node('collect_trigger')
    start_client = node.create_client(Trigger, 'start_collect')
    stop_client = node.create_client(Trigger, 'stop_collect')

    is_recording = False

    def status_callback(msg):
        nonlocal is_recording
        was_recording = is_recording
        is_recording = msg.data
        if was_recording and not is_recording:
            print("\r>>> Auto stopped by data_collector.\r")
            print("Waiting for input...\r")

    node.create_subscription(Bool, 'collecting_status', status_callback, 10)
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("========================================")
    print("  Data Collection Trigger")
    print("========================================")
    print("  s  - Start recording")
    print("  d  - Stop recording and save")
    print("  q  - Quit")
    print("========================================")
    print("Waiting for input...")

    try:
        while True:
            ch = getch().lower()
            if ch == 's':
                print("\r>>> START recording...\r")
                result = call_service(node, start_client, "start")
                if result and result.success:
                    is_recording = True
            elif ch == 'd':
                print("\r>>> STOP recording, saving...\r")
                result = call_service(node, stop_client, "stop")
                is_recording = False
            elif ch == 'q':
                if is_recording:
                    print("\r>>> Stopping recording before quit...\r")
                    call_service(node, stop_client, "stop")
                print("\rBye!\r")
                break
    except KeyboardInterrupt:
        if is_recording:
            print("\r>>> Stopping recording...\r")
            call_service(node, stop_client, "stop")
        print("\rBye!\r")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
