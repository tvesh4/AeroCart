import serial
import time
import cv2
import numpy as np
import math
from collections import deque
import threading

# ============ Arduino Mega Communication ============
class ArduinoMegaController:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None
        self.connect()
        
    def connect(self):
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            print(f"✓ Arduino Mega connected on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"✗ Error connecting to Arduino Mega: {e}")
            return False
    
    def send_command(self, cmd):
        """Send command to Arduino"""
        self.serial_connection.write(f"{cmd}\n".encode())
    
    def mecanum_move(self, vx, vy, vz):
        """
        Mecanum wheel control
        vx: forward/backward (-100 to 100)
        vy: strafe left/right (-100 to 100)
        vz: rotation (-100 to 100)
        """
        # Mecanum wheel kinematics
        speeds = {
            'A': vx - vy - vz,  # Front left
            'B': vx + vy + vz,  # Front right
            'C': vx + vy - vz,  # Rear left
            'D': vx - vy + vz   # Rear right
        }
        
        # Normalize speeds to -100 to 100
        max_speed = max(abs(max(speeds.values())), abs(min(speeds.values())), 1)
        if max_speed > 100:
            scale = 100.0 / max_speed
            for k in speeds:
                speeds[k] = int(speeds[k] * scale)
        else:
            for k in speeds:
                speeds[k] = int(speeds[k])
        
        # Send to Arduino
        cmd = f"MOVE:{speeds['A']},{speeds['B']},{speeds['C']},{speeds['D']}"
        self.send_command(cmd)
        return speeds
    
    def stop(self):
        """Stop all motors"""
        self.send_command("STOP")
    
    def close(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.stop()
            self.serial_connection.close()

# ============ UWB Distance Measurement (Master at Start) ============
class UWBDistanceSensor:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """
        UWB tag on robot, measuring distance from master UWB at starting point
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None
        self.current_distance = 0.0  # Distance from start point
        self.distance_history = deque(maxlen=20)  # For filtering
        self.running = True
        self.connect()
        
    def connect(self):
        """Connect to UWB module"""
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(1)
            print(f"✓ UWB tag connected on {self.port}")
            print("  Measuring distance from master UWB at START point")
            return True
        except serial.SerialException as e:
            print(f"✗ Error connecting to UWB: {e}")
            return False
    
    def read_distance(self):
        """
        Read distance from master UWB at start point
        Returns distance in meters from start
        """
        if not self.serial_connection:
            return None
        
        try:
            if self.serial_connection.in_waiting > 0:
                line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                
                # Parse distance (format depends on your UWB module)
                if "DIST:" in line:
                    distance_str = line.split("DIST:")[1].split()[0]
                elif "RANGE:" in line:
                    distance_str = line.split("RANGE:")[1].split()[0]
                else:
                    distance_str = line
                
                # Extract number
                import re
                numbers = re.findall(r"[-+]?\d*\.\d+|\d+", distance_str)
                if numbers:
                    distance = float(numbers[0])
                    
                    # Apply moving average filter
                    self.distance_history.append(distance)
                    if len(self.distance_history) > 0:
                        self.current_distance = sum(self.distance_history) / len(self.distance_history)
                    else:
                        self.current_distance = distance
                    
                    return self.current_distance
                    
        except Exception as e:
            print(f"UWB read error: {e}")
        
        return self.current_distance
    
    def get_distance(self):
        """Get latest filtered distance from start"""
        return self.current_distance
    
    def reset_origin(self):
        """Reset distance measurement (call when at start position)"""
        self.distance_history.clear()
        self.current_distance = 0.0
        print("  UWB origin reset to current position")
    
    def close(self):
        self.running = False
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()

# ============ Gate Assignment Store ============
class GateAssignmentStore:
    """
    In-code mapping for person -> gate assignment.
    Keep gate values limited to 1 or 2.
    """
    def __init__(self):
        self.face_gate_map = {
            "person_1": 1,
            "person_2": 2,
        }
    
    def get_gate(self, person_id):
        """Return assigned gate for person, defaulting to gate 1."""
        gate_number = self.face_gate_map.get(str(person_id), 1)
        return gate_number if gate_number in (1, 2) else 1


# ============ Gate Database ============
class GateDatabase:
    def __init__(self):
        # Gate number -> distance from START point (in meters)
        self.gate_distances_from_start = {
            1: 2.5,   # Gate 1 is 2.5 meters from start
            2: 3.0,   # Gate 2 is 3.0 meters from start
        }
        self.assignment_store = GateAssignmentStore()
    
    def get_gate_for_face(self, person_id):
        """Get gate number for recognized face"""
        return self.assignment_store.get_gate(person_id)
    
    def get_distance_from_start(self, gate_number):
        """Get distance from start point for specific gate"""
        return self.gate_distances_from_start.get(gate_number, 2.5)
    
    def get_turn_direction(self, gate_number):
        """Return turn direction based on gate parity"""
        return "right" if gate_number % 2 == 0 else "left"
    
    def get_all_gates(self):
        """Get all gate information"""
        return self.gate_distances_from_start

# ============ Face Recognition ============
class FaceRecognizer:
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        # Load pre-trained face recognizer if available
        # self.recognizer = cv2.face.LBPHFaceRecognizer_create()
        # self.recognizer.read("trained_model.yml")
        
    def recognize_face(self):
        """
        Capture and recognize face from camera
        Returns person_id or None
        """
        cap = cv2.VideoCapture(0)
        
        print("\n📸 Face Recognition Phase")
        print("Please position face in front of camera...")
        
        # Countdown
        for i in range(3, 0, -1):
            print(f"  {i}...")
            time.sleep(1)
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            print("  ✗ Failed to capture image")
            return None
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 5)
        
        if len(faces) > 0:
            print(f"  ✓ Detected {len(faces)} face(s)")
            # For demo, return default person
            # In production, use actual face recognition
            return "person_1"
        else:
            print("  ✗ No face detected")
            return None

# ============ Line Following (Camera-based) ============
class LineFollower:
    def __init__(self):
        self.camera = None
        self.init_camera()
        
    def init_camera(self):
        """Initialize camera for line following"""
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
    def detect_line(self):
        """
        Detect black line and calculate steering
        Returns: steering value (-1 to 1)
        """
        if not self.camera:
            return 0
        
        ret, frame = self.camera.read()
        if not ret:
            return 0
        
        # Convert to HSV for better line detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define black color range (adjust based on your floor)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        
        # Create mask for black line
        mask = cv2.inRange(hsv, lower_black, upper_black)
        
        # Get region of interest (bottom part of frame)
        height, width = mask.shape
        roi = mask[height-60:height, :]
        
        # Find line position
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Get largest contour (the line)
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])  # Center of line
                frame_center = width // 2
                
                # Calculate error (-1 to 1)
                error = (cx - frame_center) / (width // 2)
                error = max(-1, min(1, error))
                
                return error
        
        return 0  # No line detected
    
    def release(self):
        """Release camera"""
        if self.camera:
            self.camera.release()

# ============ Main Navigation System ============
class AutonomousNavigator:
    def __init__(self, arduino_port='/dev/ttyACM0', uwb_port='/dev/ttyUSB0'):
        # Initialize hardware
        self.arduino = ArduinoMegaController(port=arduino_port)
        self.uwb = UWBDistanceSensor(port=uwb_port)
        self.line_follower = LineFollower()
        self.face_recognizer = FaceRecognizer()
        self.gate_db = GateDatabase()
        
        # Navigation state
        self.target_gate = None
        self.target_distance = 0.0  # Distance to travel from start (meters)
        self.start_distance = 0.0    # Distance at start point
        self.current_distance = 0.0  # Current distance from start
        self.destination_reached = False
        
        # Movement parameters
        self.base_speed = 40  # Forward speed (0-100)
        self.turn_speed = 50  # Turning speed
        
        # Distance tolerance (meters)
        self.distance_tolerance = 0.1
        
        print("\n" + "="*50)
        print("🤖 AUTONOMOUS NAVIGATION SYSTEM INITIALIZED")
        print("="*50)
        
    def calibrate_uwb_at_start(self):
        """
        Calibrate UWB at starting position
        Sets the origin point for distance measurement
        """
        print("\n📍 UWB Calibration Phase")
        print("Robot at STARTING point. Calibrating distance measurement...")
        
        # Read multiple samples to establish baseline
        samples = []
        for i in range(10):
            dist = self.uwb.read_distance()
            if dist is not None:
                samples.append(dist)
            time.sleep(0.1)
        
        if samples:
            self.start_distance = sum(samples) / len(samples)
            print(f"  ✓ Baseline distance from master: {self.start_distance:.2f}m")
            print("  ✓ Origin set - robot will measure distance traveled from this point")
        else:
            self.start_distance = 0.0
            print("  ⚠ No UWB reading, using 0 as baseline")
        
        # Reset UWB origin if supported
        self.uwb.reset_origin()
        
        return True
    
    def get_distance_traveled(self):
        """
        Calculate distance traveled from start point
        Returns: distance in meters
        """
        current_raw = self.uwb.get_distance()
        
        if current_raw is not None:
            # Distance traveled = current distance from master - distance at start
            distance_traveled = abs(current_raw - self.start_distance)
            return distance_traveled
        
        return 0.0
    
    def get_face_and_gate(self):
        """
        Step 1: Recognize face and determine target gate
        """
        print("\n" + "="*50)
        print("🎯 STEP 1: GATE IDENTIFICATION")
        print("="*50)
        
        person_id = self.face_recognizer.recognize_face()
        
        if person_id:
            self.target_gate = self.gate_db.get_gate_for_face(person_id)
            self.target_distance = self.gate_db.get_distance_from_start(self.target_gate)
            print(f"\n  ✓ Recognized: {person_id}")
            print(f"  ✓ Assigned Gate: {self.target_gate}")
            print(f"  ✓ Distance to travel from start: {self.target_distance:.2f}m")
            
            # Show gate info
            direction = self.gate_db.get_turn_direction(self.target_gate)
            print(f"  ✓ Gate {self.target_gate} is {'even' if self.target_gate % 2 == 0 else 'odd'}")
            print(f"  ✓ Will turn {direction.upper()} at destination")
            
            return True
        else:
            print("\n  ⚠ Face not recognized! Using default gate 1")
            self.target_gate = 1
            self.target_distance = self.gate_db.get_distance_from_start(1)
            print(f"  ✓ Default Gate: {self.target_gate}")
            print(f"  ✓ Distance to travel: {self.target_distance:.2f}m")
            return False
    
    def navigate_to_gate(self):
        """
        Step 2: Follow black line while monitoring distance traveled from start
        Stop when traveled distance >= target gate distance
        """
        print("\n" + "="*50)
        print("🚗 STEP 2: NAVIGATION TO GATE")
        print("="*50)
        print(f"Target Gate: {self.target_gate}")
        print(f"Target Distance from Start: {self.target_distance:.2f}m")
        print(f"Following black line...")
        print("-" * 50)
        
        no_progress_counter = 0
        last_distance_traveled = 0
        start_time = time.time()
        
        try:
            while not self.destination_reached:
                # Get current distance traveled from start
                self.current_distance = self.get_distance_traveled()
                
                if self.current_distance is not None:
                    # Calculate progress percentage
                    progress = (self.current_distance / self.target_distance) * 100
                    progress = min(100, progress)
                    
                    # Display distance info
                    print(f"  Traveled: {self.current_distance:.2f}m | Target: {self.target_distance:.2f}m | Progress: {progress:.1f}%")
                    
                    # Check if reached target distance
                    if self.current_distance >= (self.target_distance - self.distance_tolerance):
                        print("\n" + "🎯" * 10)
                        print(f"✓ REACHED GATE {self.target_gate}!")
                        print(f"  Distance traveled: {self.current_distance:.2f}m")
                        print(f"  Target distance: {self.target_distance:.2f}m")
                        print("🎯" * 10)
                        self.destination_reached = True
                        self.arduino.stop()
                        break
                    
                    # Check for progress (avoid stuck situations)
                    if abs(self.current_distance - last_distance_traveled) < 0.02:
                        no_progress_counter += 1
                    else:
                        no_progress_counter = 0
                    
                    last_distance_traveled = self.current_distance
                    
                    # If stuck, perform maneuver
                    if no_progress_counter > 100:  # ~10 seconds
                        print("  ⚠ Warning: No progress detected, performing unstuck maneuver...")
                        self.unstick_maneuver()
                        no_progress_counter = 0
                    
                    # Adjust speed based on proximity to target
                    remaining_distance = self.target_distance - self.current_distance
                    if remaining_distance < 0.5:
                        # Slow down when approaching gate
                        current_speed = int(self.base_speed * (remaining_distance / 0.5))
                        current_speed = max(20, min(self.base_speed, current_speed))
                    else:
                        current_speed = self.base_speed
                else:
                    current_speed = self.base_speed
                
                # Follow black line with steering
                steering = self.line_follower.detect_line()
                
                # Apply steering to mecanum movement
                if abs(steering) < 0.1:
                    # Go straight
                    self.arduino.mecanum_move(current_speed, 0, 0)
                elif steering < 0:
                    # Turn left (negative steering = line is left)
                    turn_amount = int(abs(steering) * 30)
                    self.arduino.mecanum_move(current_speed, 0, -turn_amount)
                else:
                    # Turn right (positive steering = line is right)
                    turn_amount = int(steering * 30)
                    self.arduino.mecanum_move(current_speed, 0, turn_amount)
                
                time.sleep(0.05)  # 20Hz control loop
                
        except KeyboardInterrupt:
            print("\n  ⚠ Navigation interrupted by user")
            self.arduino.stop()
        except Exception as e:
            print(f"\n  ✗ Navigation error: {e}")
            self.arduino.stop()
    
    def unstick_maneuver(self):
        """Execute maneuver when robot is stuck"""
        print("  🔄 Executing unstuck maneuver...")
        
        # Stop
        self.arduino.stop()
        time.sleep(0.3)
        
        # Move backward
        self.arduino.mecanum_move(-30, 0, 0)
        time.sleep(0.3)
        self.arduino.stop()
        time.sleep(0.3)
        
        # Strafe left
        self.arduino.mecanum_move(0, 30, 0)
        time.sleep(0.2)
        
        # Strafe right
        self.arduino.mecanum_move(0, -30, 0)
        time.sleep(0.2)
        self.arduino.stop()
        time.sleep(0.3)
    
    def turn_at_gate(self):
        """
        Step 3: Turn based on gate parity
        Even gate = Turn Right
        Odd gate = Turn Left
        """
        print("\n" + "="*50)
        print("🔄 STEP 3: GATE TURNING")
        print("="*50)
        
        direction = self.gate_db.get_turn_direction(self.target_gate)
        
        print(f"Gate {self.target_gate}: {'Even' if self.target_gate % 2 == 0 else 'Odd'} gate")
        print(f"Turning {direction.upper()}")
        
        if direction == "right":
            self.turn_right()
        else:
            self.turn_left()
    
    def turn_right(self):
        """Execute 90-degree right turn"""
        print("  Executing 90° right turn...")
        
        # Rotate in place right (clockwise)
        self.arduino.mecanum_move(0, 0, -self.turn_speed)
        time.sleep(0.8)  # Adjust timing for 90 degrees
        
        self.arduino.stop()
        time.sleep(0.3)
        
        print("  ✓ Right turn complete")
    
    def turn_left(self):
        """Execute 90-degree left turn"""
        print("  Executing 90° left turn...")
        
        # Rotate in place left (counter-clockwise)
        self.arduino.mecanum_move(0, 0, self.turn_speed)
        time.sleep(0.8)  # Adjust timing for 90 degrees
        
        self.arduino.stop()
        time.sleep(0.3)
        
        print("  ✓ Left turn complete")
    
    def run(self):
        """Main execution sequence"""
        print("\n" + "🚀" * 25)
        print("STARTING AUTONOMOUS NAVIGATION SEQUENCE")
        print("🚀" * 25)
        
        try:
            # Step 0: Calibrate UWB at start position
            self.calibrate_uwb_at_start()
            time.sleep(1)
            
            # Step 1: Identify gate via face recognition
            self.get_face_and_gate()
            time.sleep(1)
            
            # Step 2: Navigate to gate (follow line until distance threshold)
            self.navigate_to_gate()
            
            # Step 3: Turn based on gate number
            if self.destination_reached:
                self.turn_at_gate()
                
                # Mission complete
                print("\n" + "✅" * 20)
                print("MISSION COMPLETE!")
                print(f"Robot successfully arrived at Gate {self.target_gate}")
                print(f"Total distance traveled: {self.current_distance:.2f}m")
                print(f"Turn executed: {'RIGHT' if self.target_gate % 2 == 0 else 'LEFT'}")
                print("✅" * 20)
            else:
                print("\n⚠ Mission incomplete - failed to reach gate")
                
        except KeyboardInterrupt:
            print("\n\n⚠ Program interrupted by user")
        except Exception as e:
            print(f"\n✗ Fatal error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up all resources"""
        print("\n🧹 Cleaning up resources...")
        self.arduino.close()
        self.uwb.close()
        self.line_follower.release()
        cv2.destroyAllWindows()
        print("✓ Cleanup complete")
        print("👋 Program terminated")

# ============ Test Functions ============
def test_uwb_distance():
    """Test UWB distance measurement from master at start"""
    print("Testing UWB distance measurement...")
    uwb = UWBDistanceSensor(port='/dev/ttyUSB0')
    
    print("Move robot away from start point to see distance increase")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            distance = uwb.get_distance()
            if distance is not None:
                print(f"Distance from start: {distance:.2f}m")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        uwb.close()

def test_motors():
    """Test all motors and mecanum movement"""
    arduino = ArduinoMegaController(port='/dev/ttyACM0')
    
    print("Testing motors...")
    
    print("  Forward")
    arduino.mecanum_move(50, 0, 0)
    time.sleep(2)
    
    print("  Backward")
    arduino.mecanum_move(-50, 0, 0)
    time.sleep(2)
    
    print("  Strafe Left")
    arduino.mecanum_move(0, 50, 0)
    time.sleep(2)
    
    print("  Strafe Right")
    arduino.mecanum_move(0, -50, 0)
    time.sleep(2)
    
    print("  Rotate Left")
    arduino.mecanum_move(0, 0, 50)
    time.sleep(2)
    
    print("  Rotate Right")
    arduino.mecanum_move(0, 0, -50)
    time.sleep(2)
    
    arduino.stop()
    arduino.close()
    print("Test complete")

# ============ Main Execution ============
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Jetson Nano Navigation with UWB Master at Start')
    parser.add_argument('--arduino-port', type=str, default='/dev/ttyACM0',
                       help='Arduino Mega serial port')
    parser.add_argument('--uwb-port', type=str, default='/dev/ttyUSB0',
                       help='UWB tag serial port')
    parser.add_argument('--test-uwb', action='store_true',
                       help='Test UWB distance measurement')
    parser.add_argument('--test-motors', action='store_true',
                       help='Test motor movement')
    
    args = parser.parse_args()
    
    if args.test_uwb:
        test_uwb_distance()
    elif args.test_motors:
        test_motors()
    else:
        # Run main navigation
        navigator = AutonomousNavigator(
            arduino_port=args.arduino_port,
            uwb_port=args.uwb_port
        )
        navigator.run()
