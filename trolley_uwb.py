#!/usr/bin/env python3
import serial
import time
import json

SERIAL_PORT = '/dev/ttyUSB0'  # Check with: ls /dev/ttyUSB*
target_distance = 1.5

ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)

print("Trolley UWB Reader - Target: 1.5m")
print("Walk around to test distance")

while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if 'rdist 0:' in line:
        # Parse distance: "rdist 0: 1.234 m"
        dist = float(line.split(':')[1].split(' ')[0])
        timestamp = time.strftime('%H:%M:%S')
        
        data = {
            'time': timestamp,
            'distance': round(dist, 3),
            'status': 'follow' if dist > target_distance else 'stop'
        }
        
        print(json.dumps(data))
        print(f"[{timestamp}] Distance: {dist:.3f}m -> {'FOLLOW' if dist > target_distance else 'STOP'}")
    
    time.sleep(0.1)