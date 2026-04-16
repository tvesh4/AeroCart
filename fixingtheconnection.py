#!/usr/bin/env python3
import serial
import time
import subprocess
import glob

# Kill everything first
print("Cleaning up...")
subprocess.run(['sudo', 'killall', 'screen'], stderr=subprocess.DEVNULL)
subprocess.run(['sudo', 'killall', 'minicom'], stderr=subprocess.DEVNULL)
time.sleep(1)

# Find the port
ports = glob.glob('/dev/tty.usbmodem*')
if not ports:
    ports = glob.glob('/dev/cu.usbmodem*')
if not ports:
    print("No DWM1001 found! Check USB connection.")
    exit(1)

PORT = ports[0]
print(f"Found: {PORT}")

# Try to open
for baud in [115200, 9600, 57600, 230400]:
    print(f"\nTrying {baud} baud...")
    try:
        ser = serial.Serial(PORT, baud, timeout=2)
        time.sleep(2)
        
        # Send break to reset serial state
        ser.send_break()
        time.sleep(0.5)
        
        # Send command
        ser.write(b'at\r\n')
        time.sleep(0.5)
        
        response = ser.read(200)
        if response:
            print(f"✓ WORKING at {baud} baud!")
            print(f"Response: {response.decode('ascii', errors='ignore')}")
            
            # Configure
            print("\nConfiguring module...")
            commands = [b'at\r\n', b'le\r\n', b'tn\r\n', b'save\r\n', b'reset\r\n']
            for cmd in commands:
                ser.write(cmd)
                time.sleep(0.3)
                resp = ser.read(100)
                if resp:
                    print(f"  {cmd.decode().strip()}: OK")
            
            print("\n✅ Module configured!")
            ser.close()
            break
        else:
            ser.close()
    except Exception as e:
        print(f"  Error: {e}")

print("\nDone!")
