#!/usr/bin/env python3
import sys
import tty
import termios
import select
import json
import numpy as np
import math
import time
import paho.mqtt.client as mqtt

# ================= CONFIGURATION =================
BOT_ID = 2              # <--- CHANGE THIS TO THE BOT YOU ARE TUNING
MQTT_IP = "192.168.0.183"
# =================================================

class ThetaTuner:
    def __init__(self):
        # ---------------- Kinematics Setup (Copied from your code) ----------------
        self.alphas = [math.radians(30), math.radians(150), math.radians(270)]
        M = np.array([
            [math.cos(a + math.pi/2) for a in self.alphas],
            [math.sin(a + math.pi/2) for a in self.alphas],
            [1, 1, 1]
        ])
        self.M_inv = np.linalg.pinv(M)

        # ---------------- MQTT Setup ----------------
        self.client = mqtt.Client()
        try:
            self.client.connect(MQTT_IP, 1883, 60)
            self.client.loop_start()
            print(f"Connected to MQTT Broker at {MQTT_IP}")
        except Exception as e:
            print(f"Failed to connect to MQTT: {e}")
            sys.exit(1)

        self.current_omega = 0.0
        self.step_size = 0.5  # Increment size

    def map_vel(self, v):
        """ Maps velocity -50 to 50 to servo range """
        v = max(min(v, 50), -50)
        return int(90 + (v / 50.0) * 90)

    def publish_cmd(self):
        # We assume vx=0, vy=0, only testing omega
        vx = 0.0
        vy = 0.0
        omega = self.current_omega

        # Calculate wheel velocities using your matrix
        wheel_vel = self.M_inv @ np.array([vx, vy, omega])
        
        # Prepare Payload
        m1 = self.map_vel(wheel_vel[0])
        m2 = self.map_vel(wheel_vel[1])
        m3 = self.map_vel(wheel_vel[2])

        payload = {
            "m1": m2, "m2": m1, "m3": m3,
            "base": 45, "elbow": 135, "mag": 0
        }

        topic = f"bot{BOT_ID}/cmd"
        self.client.publish(topic, json.dumps(payload))
        
        return wheel_vel

    def stop(self):
        self.current_omega = 0.0
        self.publish_cmd()

# ---------------- Key Input Helpers ----------------
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# ---------------- Main Loop ----------------
settings = termios.tcgetattr(sys.stdin)
tuner = ThetaTuner()

print(f"""
=============================================
      THETA STICTION TUNER | BOT {BOT_ID}
=============================================
w/s : Increase/Decrease Omega Command (Step: {tuner.step_size})
Space : STOP (Zero)
q     : Quit

INSTRUCTIONS:
1. Increase Omega until the robot *just* starts to rotate.
2. If it hums but doesn't move, go higher.
3. If it spins fast, go lower.
4. The lowest value that causes PHYSICAL movement is your stiction.
=============================================
""")

try:
    while True:
        key = getKey()
        
        if key == 'w':
            tuner.current_omega += tuner.step_size
        elif key == 's':
            tuner.current_omega -= tuner.step_size
        elif key == ' ':
            tuner.current_omega = 0.0
        elif key == 'q':
            break
        elif key == '\x03': # Ctrl+C
            break

        # Publish and Get Raw Wheel Speeds
        raw_wheels = tuner.publish_cmd()

        # Print Status line (clears previous line)
        sys.stdout.write(f"\rCmd Omega: {tuner.current_omega:.2f} | M1: {raw_wheels[0]:.2f} M2: {raw_wheels[1]:.2f} M3: {raw_wheels[2]:.2f}   ")
        sys.stdout.flush()

except Exception as e:
    print(e)

finally:
    tuner.stop()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print("\n\nFinished. Use the found 'Cmd Omega' as your min_stiction_omega.")