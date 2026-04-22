"""
# PC_UWB_Anchor.py - Run this on your MAC
import serial
import socket
import time

class PCUWBAnchor:
    def __init__(self, uwb_port='/dev/cu.usbmodem0007602170731', jetson_ip='192.168.1.100', jetson_port=5000):
        # ⭐ CHANGE THIS LINE ⭐ - UWB port on your MAC
        self.uwb_port = uwb_port  # Already set to your port
        self.jetson_ip = jetson_ip
        self.jetson_port = jetson_port
        
        # Connect to UWB anchor
        try:
            self.uwb = serial.Serial(uwb_port, 115200, timeout=1)
            print(f"✓ UWB Anchor connected on {uwb_port}")
        except serial.SerialException as e:
            print(f"✗ Error: {e}")
            exit(1)
        
        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    def run(self):
        print(f"Sending UWB data to Jetson at {self.jetson_ip}:{self.jetson_port}")
        while True:
            try:
                if self.uwb.in_waiting:
                    line = self.uwb.readline().decode().strip()
                    # Send to Jetson
                    self.sock.sendto(f"DIST:{line}".encode(), (self.jetson_ip, self.jetson_port))
                    print(f"Sent: {line}m")
                time.sleep(0.05)
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    # ⭐ CHANGE THESE LINES ⭐
    anchor = PCUWBAnchor(
        uwb_port='/dev/cu.usbmodem0007602170731',  # YOUR MAC UWB PORT
        jetson_ip='192.168.137.194',  # YOUR JETSON'S IP ADDRESS
        jetson_port=5000
    )
    anchor.run()
"""

# PC_UWB_Anchor.py - Run on MAC (only reads UWB anchor, radio transmits to Tag)
import serial
import time

class PCUWBAnchor:
    def __init__(self, uwb_port='/dev/cu.usbmodem0007602170291'):
        # UWB Anchor port on MAC
        self.uwb_port = uwb_port
        self.uwb = None
        
    def connect(self):
        """Connect to UWB anchor"""
        try:
            self.uwb = serial.Serial(self.uwb_port, 115200, timeout=0.1)
            time.sleep(2)
            print(f"✓ UWB Anchor connected on {self.uwb_port}")
            return True
        except serial.SerialException as e:
            print(f"✗ UWB Error: {e}")
            return False
    
    def run(self):
        """Read UWB anchor - radio automatically transmits to Tag on Jetson"""
        if not self.connect():
            return
        
        try:
            while True:
                if self.uwb.in_waiting:
                    line = self.uwb.readline().decode().strip()
                    try:
                        distance = float(line)
                        print(f"📡 Broadcasting: {distance:.2f}m (via radio)")
                    except:
                        print(f"📡 Broadcasting: {line}")
                
                time.sleep(0.01)  # 20Hz update
                
        except KeyboardInterrupt:
            print("\n\n✓ Stopped by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        if self.uwb:
            self.uwb.close()
        print("✓ Cleanup complete")

if __name__ == "__main__":
    # Your UWB anchor port on MAC
    UWB_PORT = '/dev/cu.usbmodem0007602170291'
    
    anchor = PCUWBAnchor(uwb_port=UWB_PORT)
    anchor.run()