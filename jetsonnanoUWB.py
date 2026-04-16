#!/usr/bin/env python3
"""
Jetson Nano UWB Robot Control System
Handles both Master and Slave DWM1001 modules for autonomous following
"""

import serial
import time
import struct
import threading
import queue
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple
import json
import logging
from enum import Enum

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ============================================================
# Data Structures
# ============================================================

@dataclass
class UWBData:
    """UWB ranging data structure"""
    distance: float
    quality: float
    timestamp: float
    anchor_id: int = 0
    valid: bool = False

@dataclass
class RobotCommand:
    """Robot motor commands"""
    left_speed: int  # -255 to 255
    right_speed: int
    command_type: str = "motor"

class RobotState(Enum):
    """Robot operational states"""
    IDLE = "idle"
    FOLLOWING = "following"
    AVOIDING = "avoiding"
    SEARCHING = "searching"
    EMERGENCY_STOP = "emergency_stop"

# ============================================================
# DWM1001 Binary Communication Handler
# ============================================================

class DWM1001Handler:
    """Handles binary communication with DWM1001 module"""
    
    def __init__(self, port: str, baud: int = 115200, name: str = "UWB"):
        self.port = port
        self.baud = baud
        self.name = name
        self.ser = None
        self.running = False
        self.read_thread = None
        self.data_queue = queue.Queue(maxsize=100)
        
        # Latest data
        self.current_distance = 0.0
        self.current_quality = 1.0
        self.last_update = 0
        self.filtered_distance = 0.0
        
        # Filter parameters
        self.alpha = 0.7  # Low-pass filter coefficient
        
    def connect(self) -> bool:
        """Connect to DWM1001 module"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            time.sleep(2)
            logger.info(f"✓ {self.name} connected on {self.port}")
            return True
        except Exception as e:
            logger.error(f"✗ {self.name} connection failed: {e}")
            return False
    
    def send_binary_command(self, cmd_byte: int, data: bytes = b'\x00\x00\x00') -> bytes:
        """Send binary command to module"""
        if not self.ser:
            return b''
        
        cmd = bytes([cmd_byte]) + data[:3]
        try:
            self.ser.reset_input_buffer()
            self.ser.write(cmd)
            time.sleep(0.03)
            return self.ser.read(100)
        except:
            return b''
    
    def parse_distance(self, response: bytes) -> Optional[float]:
        """Extract distance from binary response"""
        if len(response) < 7:
            return None
        
        # Try different offsets to find valid float
        for offset in range(3, min(len(response) - 3, 12)):
            try:
                val = struct.unpack('<f', response[offset:offset+4])[0]
                # Reasonable distance range (0.1m to 50m)
                if 0.1 < val < 50:
                    return val
            except:
                pass
        return None
    
    def read_distance(self) -> Optional[float]:
        """Read distance from module"""
        response = self.send_binary_command(0x10)
        distance = self.parse_distance(response)
        
        if distance is not None:
            self.current_distance = distance
            self.last_update = time.time()
            
            # Apply low-pass filter
            if self.filtered_distance == 0:
                self.filtered_distance = distance
            else:
                self.filtered_distance = self.alpha * distance + (1 - self.alpha) * self.filtered_distance
            
            return self.filtered_distance
        
        return None
    
    def get_distance(self) -> float:
        """Get latest filtered distance"""
        return self.filtered_distance
    
    def start_continuous_reading(self):
        """Start background thread for continuous reading"""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
    
    def _read_loop(self):
        """Background read loop"""
        while self.running:
            distance = self.read_distance()
            if distance is not None:
                try:
                    self.data_queue.put_nowait(UWBData(
                        distance=distance,
                        quality=0.05,
                        timestamp=time.time(),
                        valid=True
                    ))
                except queue.Full:
                    pass
            time.sleep(0.05)  # 20Hz update rate
    
    def get_latest_data(self) -> Optional[UWBData]:
        """Get latest UWB data from queue"""
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            if self.filtered_distance > 0:
                return UWBData(
                    distance=self.filtered_distance,
                    quality=0.05,
                    timestamp=self.last_update,
                    valid=(time.time() - self.last_update) < 0.5
                )
        return None
    
    def stop(self):
        """Stop communication"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1)
        if self.ser:
            self.ser.close()


# ============================================================
# PID Controller for Smooth Following
# ============================================================

class PIDController:
    """PID controller for smooth robot movement"""
    
    def __init__(self, kp: float = 2.5, ki: float = 0.05, kd: float = 1.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()
        self.integral_limit = 100.0
        
    def compute(self, error: float, dt: float = None) -> float:
        """Compute PID output"""
        now = time.time()
        if dt is None:
            dt = now - self.last_time
            if dt <= 0:
                dt = 0.05
        
        # Proportional
        p = self.kp * error
        
        # Integral (with anti-windup)
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i = self.ki * self.integral
        
        # Derivative
        d = self.kd * (error - self.prev_error) / dt
        
        # Update state
        self.prev_error = error
        self.last_time = now
        
        return p + i + d
    
    def reset(self):
        """Reset PID state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()


# ============================================================
# Robot Decision Making Engine
# ============================================================

class RobotDecisionEngine:
    """Makes decisions based on UWB data"""
    
    def __init__(self, target_distance: float = 1.0):
        self.target_distance = target_distance
        self.deadzone = 0.1  # 10cm deadzone
        self.min_speed = 60
        self.max_speed = 220
        self.safe_distance = 0.5  # Emergency stop if closer than 50cm
        
        # PID controllers for different modes
        self.pid_follow = PIDController(kp=2.5, ki=0.05, kd=1.2)
        self.pid_avoid = PIDController(kp=3.0, ki=0.08, kd=1.5)
        
        # State tracking
        self.state = RobotState.IDLE
        self.last_command = RobotCommand(0, 0)
        self.consecutive_no_data = 0
        
    def calculate_motor_speeds(self, distance: float, quality: float) -> RobotCommand:
        """Calculate motor speeds based on distance and quality"""
        
        # Check for no data
        if distance <= 0 or quality > 0.2:
            self.consecutive_no_data += 1
            if self.consecutive_no_data > 10:
                self.state = RobotState.SEARCHING
                return RobotCommand(0, 0)  # Stop and wait
        else:
            self.consecutive_no_data = 0
        
        # Emergency stop if too close
        if 0 < distance < self.safe_distance:
            self.state = RobotState.EMERGENCY_STOP
            logger.warning(f"EMERGENCY STOP! Distance: {distance:.2f}m")
            return RobotCommand(0, 0)
        
        # Calculate error
        error = distance - self.target_distance
        
        # Apply deadzone
        if abs(error) < self.deadzone:
            error = 0
            self.state = RobotState.IDLE
            return RobotCommand(0, 0)
        
        # Determine state based on error
        if error > 0:
            self.state = RobotState.FOLLOWING
            pid_output = self.pid_follow.compute(error)
        else:
            self.state = RobotState.AVOIDING
            pid_output = self.pid_avoid.compute(error)
        
        # Convert PID output to motor speeds
        if pid_output > 0:
            # Move forward
            speed = min(self.min_speed + abs(pid_output), self.max_speed)
            return RobotCommand(int(speed), int(speed))
        else:
            # Move backward
            speed = min(self.min_speed + abs(pid_output), self.max_speed)
            return RobotCommand(-int(speed), -int(speed))
    
    def get_state(self) -> RobotState:
        """Get current robot state"""
        return self.state
    
    def reset(self):
        """Reset decision engine"""
        self.pid_follow.reset()
        self.pid_avoid.reset()
        self.consecutive_no_data = 0
        self.state = RobotState.IDLE


# ============================================================
# Master-Slave UWB System
# ============================================================

class MasterSlaveUWBRobot:
    """Complete system handling master and slave robots"""
    
    def __init__(self, master_port: str, slave_port: str):
        self.master = DWM1001Handler(master_port, name="MASTER")
        self.slave = DWM1001Handler(slave_port, name="SLAVE")
        
        self.decision_engine = RobotDecisionEngine(target_distance=1.0)
        self.running = False
        
        # Data storage
        self.master_distance = 0.0
        self.slave_distance = 0.0
        self.relative_distance = 0.0
        
        # For data logging
        self.data_log = []
        
    def connect(self) -> bool:
        """Connect to both UWB modules"""
        master_ok = self.master.connect()
        slave_ok = self.slave.connect()
        
        if master_ok and slave_ok:
            logger.info("✓ Both UWB modules connected successfully")
            return True
        else:
            logger.error("Failed to connect to one or both UWB modules")
            return False
    
    def start(self):
        """Start the system"""
        if not self.connect():
            return
        
        self.running = True
        
        # Start continuous reading on both modules
        self.master.start_continuous_reading()
        self.slave.start_continuous_reading()
        
        logger.info("System started. Monitoring UWB data...")
        
        try:
            while self.running:
                # Read data from both modules
                master_data = self.master.get_latest_data()
                slave_data = self.slave.get_latest_data()
                
                if master_data and master_data.valid:
                    self.master_distance = master_data.distance
                
                if slave_data and slave_data.valid:
                    self.slave_distance = slave_data.distance
                
                # Calculate relative distance (slave to master)
                if self.master_distance > 0 and self.slave_distance > 0:
                    self.relative_distance = abs(self.master_distance - self.slave_distance)
                else:
                    self.relative_distance = self.slave_distance
                
                # Make decision based on slave distance
                command = self.decision_engine.calculate_motor_speeds(
                    self.slave_distance, 
                    0.05  # Assuming good quality
                )
                
                # Log data
                self.log_data(command)
                
                # Display status
                self.print_status(command)
                
                # Here you would send commands to Arduino
                # self.send_to_arduino(command)
                
                time.sleep(0.05)  # 20Hz control loop
                
        except KeyboardInterrupt:
            logger.info("Stopping system...")
        finally:
            self.stop()
    
    def print_status(self, command: RobotCommand):
        """Print current status"""
        status_line = (
            f"\rState: {self.decision_engine.get_state().value:12} | "
            f"Master: {self.master_distance:5.2f}m | "
            f"Slave: {self.slave_distance:5.2f}m | "
            f"Target: 1.00m | "
            f"Err: {self.slave_distance - 1.0:5.2f}m | "
            f"Motor L:{command.left_speed:4d} R:{command.right_speed:4d}"
        )
        print(status_line, end='', flush=True)
    
    def log_data(self, command: RobotCommand):
        """Log data for analysis"""
        self.data_log.append({
            'timestamp': time.time(),
            'master_distance': self.master_distance,
            'slave_distance': self.slave_distance,
            'relative_distance': self.relative_distance,
            'state': self.decision_engine.get_state().value,
            'left_speed': command.left_speed,
            'right_speed': command.right_speed
        })
        
        # Keep only last 1000 entries
        if len(self.data_log) > 1000:
            self.data_log.pop(0)
    
    def save_log(self, filename: str = "uwb_log.json"):
        """Save logged data to file"""
        import json
        with open(filename, 'w') as f:
            json.dump(self.data_log, f, indent=2)
        logger.info(f"Data saved to {filename}")
    
    def send_to_arduino(self, command: RobotCommand):
        """Send motor commands to Arduino Mega"""
        # This will be implemented when Arduino is connected
        # For now, just a placeholder
        pass
    
    def stop(self):
        """Stop all operations"""
        self.running = False
        self.master.stop()
        self.slave.stop()
        logger.info("System stopped")


# ============================================================
# Simple Test Mode (Without Actual UWB)
# ============================================================

class SimulatedUWBRobot:
    """Simulated version for testing without hardware"""
    
    def __init__(self):
        self.distance = 1.0
        self.direction = 1
        self.decision_engine = RobotDecisionEngine(target_distance=1.0)
        
    def run(self):
        """Run simulated robot"""
        print("SIMULATION MODE - No UWB hardware required")
        print("Press Ctrl+C to stop\n")
        
        try:
            while True:
                # Simulate moving target
                self.distance += self.direction * 0.02
                if self.distance > 3.0:
                    self.direction = -1
                elif self.distance < 0.3:
                    self.direction = 1
                
                # Calculate motor speeds
                command = self.decision_engine.calculate_motor_speeds(self.distance, 0.05)
                
                # Display
                print(f"\rDistance: {self.distance:5.2f}m | "
                      f"Target: 1.00m | "
                      f"Error: {self.distance - 1.0:5.2f}m | "
                      f"State: {self.decision_engine.get_state().value:12} | "
                      f"Motor L:{command.left_speed:4d} R:{command.right_speed:4d}", 
                      end='', flush=True)
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\nSimulation stopped")


# ============================================================
# Main Entry Point
# ============================================================

def find_uwb_ports():
    """Find connected DWM1001 ports"""
    import glob
    
    ports = glob.glob('/dev/tty.usbmodem*')
    if not ports:
        ports = glob.glob('/dev/ttyACM*')
    if not ports:
        ports = glob.glob('/dev/ttyUSB*')
    
    return ports

def main():
    """Main function"""
    print("=" * 60)
    print("Jetson Nano UWB Robot Control System")
    print("=" * 60)
    
    # Find UWB modules
    ports = find_uwb_ports()
    
    if len(ports) >= 2:
        print(f"\nFound {len(ports)} UWB modules:")
        for i, port in enumerate(ports):
            print(f"  {i+1}. {port}")
        
        print("\nStarting Master-Slave system...")
        robot = MasterSlaveUWBRobot(ports[0], ports[1])
        robot.start()
        
    elif len(ports) == 1:
        print(f"\nFound 1 UWB module: {ports[0]}")
        print("Running in single module mode...")
        
        # Single module mode - just monitor
        uwb = DWM1001Handler(ports[0], name="UWB")
        if uwb.connect():
            uwb.start_continuous_reading()
            print("\nMonitoring UWB data (Press Ctrl+C to stop):\n")
            try:
                while True:
                    data = uwb.get_latest_data()
                    if data:
                        print(f"\rDistance: {data.distance:6.3f}m | Quality: {data.quality:.3f}", end='', flush=True)
                    time.sleep(0.05)
            except KeyboardInterrupt:
                print("\nStopped")
            uwb.stop()
        else:
            print("Failed to connect!")
    
    else:
        print("\nNo UWB modules found!")
        print("Running in simulation mode...")
        sim = SimulatedUWBRobot()
        sim.run()

if __name__ == "__main__":
    main()
