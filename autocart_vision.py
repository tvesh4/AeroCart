#!/usr/bin/env python3
"""
AutoCart - Jetson Nano Vision Processing
Handling face detection and communication with Arduino Mega
"""

import cv2
import numpy as np
import serial
import time
import threading
import sys

# ==================== CONFIGURATION ====================

ARDUINO_PORT = '/dev/ttyTHS1'  # For Jetson Nano UART
# ARDUINO_PORT = '/dev/ttyACM0'  # For USB connection
BAUD_RATE = 115200

CAMERA_ID = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

FACE_CASCADE_PATH = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'

# ==================== AUTOCART CONTROLLER ====================

class AutoCartVision:
    def __init__(self):
        self.serial = None
        self.running = True
        self.tag_uid = None
        
        self.init_serial()
        self.init_camera()
        
        # Load face cascade
        self.face_cascade = cv2.CascadeClassifier(FACE_CASCADE_PATH)
        
        print("AutoCart Vision System Initialized")
        print(f"Serial: {ARDUINO_PORT} at {BAUD_RATE}")
    
    def init_serial(self):
        try:
            self.serial = serial.Serial(
                port=ARDUINO_PORT,
                baudrate=BAUD_RATE,
                timeout=1
            )
            time.sleep(2)
            print("Serial communication established")
        except Exception as e:
            print(f"Serial error: {e}")
            self.serial = None
    
    def init_camera(self):
        self.cap = cv2.VideoCapture(CAMERA_ID)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        
        if not self.cap.isOpened():
            print("Camera error")
            self.running = False
    
    def send_command(self, cmd):
        if self.serial and self.serial.is_open:
            self.serial.write(f"{cmd}\n".encode())
    
    def set_tag(self, uid_hex):
        """Set target RFID tag (hex string like 'DE AD BE EF')"""
        self.tag_uid = uid_hex
        self.send_command(f"TAG:{uid_hex}")
        print(f"Target tag set: {uid_hex}")
    
    def run(self):
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            frame = cv2.flip(frame, 1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Face detection
            faces = self.face_cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(50, 50)
            )
            
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                self.send_command(f"FACE:{x},{y},{w},{h}")
            
            # Display
            cv2.putText(frame, "AutoCart Vision", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("AutoCart", frame)
            
            # Read Arduino data
            if self.serial and self.serial.in_waiting:
                line = self.serial.readline().decode().strip()
                if line.startswith("DATA:"):
                    print(line)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.send_command("STOP")
            elif key == ord('r'):
                self.send_command("RESUME")
            elif key == ord('a'):
                self.send_command("MODE:AUTO")
            elif key == ord('m'):
                self.send_command("MODE:MANUAL")
        
        self.cleanup()
    
    def cleanup(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.cap.release()
        cv2.destroyAllWindows()
        print("System shutdown")

# ==================== MAIN ====================

if __name__ == "__main__":
    vision = AutoCartVision()
    
    # Example: Set a specific tag UID (replace with actual tag ID)
    # vision.set_tag("DE AD BE EF")
    
    try:
        vision.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
        vision.cleanup()
