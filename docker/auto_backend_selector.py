#!/usr/bin/env python3

import os
import subprocess
import serial.tools.list_ports
import time

def is_mavlink_device(port, baudrate=921600):
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            ser.write(b'\xfe\x00\x00\x00\x01\x01\x00\x00\x00')
            time.sleep(0.5)
            response = ser.read(100)
            if b'PX4' in response or b'ARDU' in response:
                return True
    except Exception:
        pass
    return False

def find_mavlink_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if is_mavlink_device(port.device):
            return port.device
    return None

def main():
    forced = os.getenv("FORCE_BACKEND", "").lower()
    if forced == "mavros":
        print("[INFO] FORCING backend: MAVROS")
        subprocess.call(["roslaunch", "mower_comm_mavros", "mower_mavros.launch", "fcu_url:=/dev/ttyUSB0:921600"])
        return
    elif forced == "openmower":
        print("[INFO] FORCING backend: OpenMower")
        subprocess.call(["roslaunch", "open_mower", "open_mower.launch"])
        return

    port = find_mavlink_port()
    if port:
        print(f"[INFO] MAVLink device detected on {port}. Launching MAVROS backend...")
        subprocess.call(["roslaunch", "mower_comm_mavros", "mower_mavros.launch", f"fcu_url:={port}:921600"])
    else:
        print("[INFO] No MAVLink device found. Launching OpenMower backend...")
        subprocess.call(["roslaunch", "open_mower", "open_mower.launch"])

if __name__ == "__main__":
    main()
