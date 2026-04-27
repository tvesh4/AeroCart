#!/usr/bin/env python3
import serial
import time
import json

SERIAL_PORT = '/dev/cu.usbmodem0007602170291'  # Replace with your port
ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)

print("Master UWB Reader - Person side")
print("Trolley distance streaming...")

distances = []

while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    
    # Anchor might show different data, look for any distance info
    if 'rdist' in line or 'dist' in line:
        timestamp = time.strftime('%H:%M:%S')
        print(f"[{timestamp}] Raw: {line}")
        
        # Record all readings
        distances.append({
            'time': timestamp,
            'raw': line
        })
        
        # Save every 10 readings
        if len(distances) % 10 == 0:
            with open('distance_log.json', 'w') as f:
                json.dump(distances[-50:], f, indent=2)
            print(f"Saved {len(distances)} readings to distance_log.json")
    
    time.sleep(0.1)
