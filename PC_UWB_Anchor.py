import serial
import time

class PCUWBAnchor:
    def __init__(self, uwb_port='/dev/cu.usbmodem0007602170291'):
        # UWB Anchor port on MAC
        self.uwb_port = uwb_port
        self.uwb = None
        
    def connect(self):
        #Connect to UWB anchor
        try:
            self.uwb = serial.Serial(self.uwb_port, 115200, timeout=0.1)
            time.sleep(2)
            print(f"✓ UWB Anchor connected on {self.uwb_port}")
            return True
        except serial.SerialException as e:
            print(f"✗ UWB Error: {e}")
            return False
    
    def run(self):
        #Read UWB anchor - radio automatically transmits to Tag on Jetson
        if not self.connect():
            return
        
        try:
            while True:
                if self.uwb.in_waiting:
                    line = self.uwb.readline().decode().strip()
                    try:
                        distance = float(line)
                        print(f"Broadcasting: {distance:.2f}m (via radio)")
                    except:
                        print(f"Broadcasting: {line}")
                
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
    #UWB anchor port on MAC
    UWB_PORT = '/dev/cu.usbmodem0007602170291'
    
    anchor = PCUWBAnchor(uwb_port=UWB_PORT)
    anchor.run()