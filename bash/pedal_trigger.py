#!/usr/bin/env python3
"""
Terminal pedal trigger for data collection.
Pedal 'a' to start recording, 'b' to stop recording, 'c' to quit.
Works in any terminal without X11/DISPLAY.

Usage: pixi run python3 bash/pedal_trigger.py
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
    node = rclpy.create_node('pedal_trigger')
    start_client = node.create_client(Trigger, 'start_collect')
    stop_client = node.create_client(Trigger, 'stop_collect')

    is_recording = False

    def status_callback(msg):
        nonlocal is_recording
        was_recording = is_recording
        is_recording = msg.data
        if was_recording and not is_recording:
            print("\r>>> Auto stopped by data_collector.\r")
            print("Waiting for pedal...\r")

    node.create_subscription(Bool, 'collecting_status', status_callback, 10)
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("========================================")
    print("  Pedal Data Collection Trigger")
    print("========================================")
    print("  a (left pedal)   - Start recording")
    print("  b (middle pedal) - Stop recording and save")
    print("  c (right pedal)  - Quit")
    print("========================================")
    print("Waiting for pedal...")

    try:
        while True:
            ch = getch().lower()
            if ch == 'a':
                if is_recording:
                    continue
                print("\r>>> START recording...\r")
                result = call_service(node, start_client, "start")
                if result and result.success:
                    is_recording = True
            elif ch == 'b':
                if not is_recording:
                    continue
                print("\r>>> STOP recording, saving...\r")
                result = call_service(node, stop_client, "stop")
                is_recording = False
            elif ch == 'c':
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
