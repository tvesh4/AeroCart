#!/usr/bin/env python3
"""
Robot Follower using DWM1001 in Binary Mode
Slave robot follows master robot maintaining fixed distance
"""

import serial
import time
import struct
import math
import threading
from dataclasses import dataclass
from enum import Enum

# ============================================================
# DWM1001 Binary Communication Class
# ============================================================

class DWM1001Binary:
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.distance = 0.0
        self.quality = 1.0
        self.last_update = 0
        self.running = False
        
    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            time.sleep(2)
            print(f"✓ DWM1001 connected on {self.port}")
            return True
        except Exception as e:
            print(f"✗ DWM1001 connection failed: {e}")
            return False
    
    def send_command(self, cmd_byte: int, data: bytes = b'\x00\x00\x00') -> bytes:
        if not self.ser:
            return b''
        cmd = bytes([cmd_byte]) + data[:3]
        self.ser.write(cmd)
        time.sleep(0.03)
        return self.ser.read(100)
    
    def update_distance(self) -> float:
        """Read and update current distance"""
        resp = self.send_command(0x10)
        
        if len(resp) > 7:
            # Try to find float value
            for offset in range(3, min(len(resp) - 3, 12)):
                try:
                    val = struct.unpack('<f', resp[offset:offset+4])[0]
                    if 0.1 < val < 50:  # Reasonable distance
                        # Simple low-pass filter
                        self.distance = 0.7 * self.distance + 0.3 * val
                        self.last_update = time.time()
                        return self.distance
                except:
                    pass
        return self.distance
    
    def get_distance(self) -> float:
        """Get current distance"""
        return self.update_distance()
    
    def is_data_fresh(self, timeout: float = 0.5) -> bool:
        """Check if distance data is recent"""
        return (time.time() - self.last_update) < timeout
    
    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()


# ============================================================
# Robot Motor Control
# ============================================================

class MotorController:
    def __init__(self, ena=9, enb=10, in1=4, in2=5, in3=6, in4=7):
        self.ena = ena
        self.enb = enb
        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4
        
        # For software PWM if hardware PWM not available
        self.last_time = 0
        
    def setup(self):
        """Setup motor pins (call in setup on Arduino)"""
        # This will be implemented in Arduino code
        pass
    
    def move(self, left_speed: int, right_speed: int):
        """Set motor speeds (-255 to 255)"""
        # Negative = backward, Positive = forward
        # This will be sent to Arduino via serial
        print(f"Motor: L={left_speed:4d}, R={right_speed:4d}")


# ============================================================
# PID Controller for Smooth Following
# ============================================================

class PIDController:
    def __init__(self, kp=2.0, ki=0.1, kd=0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.time()
        
    def compute(self, error: float, dt: float = None) -> float:
        if dt is None:
            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            if dt <= 0:
                dt = 0.05
        
        # Proportional
        p = self.kp * error
        
        # Integral (with anti-windup)
        self.integral += error * dt
        self.integral = max(-100, min(100, self.integral))
        i = self.ki * self.integral
        
        # Derivative
        d = self.kd * (error - self.prev_error) / dt
        
        # Calculate output
        output = p + i + d
        
        # Save for next iteration
        self.prev_error = error
        
        return output
    
    def reset(self):
        self.integral = 0
        self.prev_error = 0
        self.last_time = time.time()


# ============================================================
# Main Follower Robot Class
# ============================================================

class UWBFollowerRobot:
    def __init__(self, port: str, target_distance: float = 1.0):
        self.uwb = DWM1001Binary(port)
        self.target_distance = target_distance
        self.deadzone = 0.1  # Ignore errors within 10cm
        self.max_speed = 200
        self.min_speed = 60
        
        # PID controller
        self.pid = PIDController(kp=2.5, ki=0.05, kd=1.2)
        
        # State
        self.current_distance = 0.0
        self.left_speed = 0
        self.right_speed = 0
        self.running = False
        
    def connect(self) -> bool:
        return self.uwb.connect()
    
    def calculate_speeds(self, distance: float) -> tuple:
        """Calculate left and right motor speeds based on distance error"""
        error = distance - self.target_distance
        
        # Apply deadzone
        if abs(error) < self.deadzone:
            error = 0
        
        # PID output
        pid_output = self.pid.compute(error)
        
        # Convert to motor speeds
        if pid_output > 0:
            # Too far - move forward
            speed = min(self.min_speed + pid_output, self.max_speed)
            return int(speed), int(speed)
        elif pid_output < 0:
            # Too close - move backward
            speed = min(self.min_speed + abs(pid_output), self.max_speed)
            return -int(speed), -int(speed)
        else:
            return 0, 0
    
    def update(self) -> float:
        """Main update loop - call this frequently"""
        # Get current distance
        self.current_distance = self.uwb.get_distance()
        
        # Calculate motor speeds
        self.left_speed, self.right_speed = self.calculate_speeds(self.current_distance)
        
        return self.current_distance
    
    def run(self):
        """Main robot control loop"""
        if not self.connect():
            print("Failed to connect to UWB module")
            return
        
        self.running = True
        print(f"\nRobot Follower Started")
        print(f"Target distance: {self.target_distance} meters")
        print(f"Deadzone: {self.deadzone} meters")
        print("-" * 50)
        
        try:
            while self.running:
                # Update UWB and calculate speeds
                distance = self.update()
                
                # Display status
                print(f"Dist: {distance:6.3f}m | "
                      f"Err: {distance - self.target_distance:6.3f}m | "
                      f"L: {self.left_speed:4d} | "
                      f"R: {self.right_speed:4d}")
                
                # Here you would send motor commands to Arduino
                # For now, just print
                
                time.sleep(0.05)  # 20Hz control loop
                
        except KeyboardInterrupt:
            print("\nStopping robot...")
        finally:
            self.stop()
    
    def stop(self):
        self.running = False
        self.uwb.stop()
        print("Robot stopped")


# ============================================================
# Arduino Communication Bridge (for actual robot)
# ============================================================

class ArduinoBridge:
    """Send motor commands to Arduino Mega"""
    def __init__(self, port: str = None, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser = None
        
    def connect(self):
        if self.port:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)
            return True
        return False
    
    def send_motor_speeds(self, left: int, right: int):
        """Send motor commands to Arduino"""
        if self.ser:
            self.ser.write(f"M{left},{right}\n".encode())
    
    def close(self):
        if self.ser:
            self.ser.close()


# ============================================================
# Main Entry Point
# ============================================================

if __name__ == "__main__":
    import glob
    
    # Find DWM1001 port
    ports = glob.glob('/dev/tty.usbmodem*')
    if not ports:
        print("ERROR: DWM1001 not found!")
        print("Check USB connection")
        exit(1)
    
    PORT = ports[0]
    print(f"Found DWM1001 at: {PORT}")
    
    # Create and run follower robot
    robot = UWBFollowerRobot(PORT, target_distance=1.0)
    robot.run()
