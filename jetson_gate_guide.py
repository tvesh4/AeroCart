import serial
import time
import threading
import face_rec

ARDUINO_PORT   = "/dev/ttyUSB0"
ARDUINO_BAUD   = 115200

UWB_PORT       = "/dev/ttyACM0"
UWB_BAUD       = 115200

# Gate database: gate_number -> target distance in metres from start
GATE_DATABASE = {
    1: 0.4,    # Gate 1 (ODD)  -> turn LEFT  at 2.5 m
    2: 0.4,    # Gate 2 (EVEN) -> turn RIGHT at 3.0 m
    3: 0.8,
    4: 0.9,
    5: 1.2,
    6: 1.3
}

# Passenger database: name -> gate number
PASSENGER_DATABASE = {
    "person_a": 1,
    "person_b": 2,
    "person_c": 3,
    "person_d": 4,
    "person_e": 5,
    "person_f": 6,
}

# How close to target distance triggers a stop (metres tolerance)
DISTANCE_TOLERANCE = 0.10

# Turn duration in seconds for a 90-degree turn
TURN_DURATION = 1.2

# Default fallback values when no face is recognised
DEFAULT_GATE     = 1
DEFAULT_DISTANCE = 0.4
DEFAULT_USER     = "unknown"

# Arduino mode constants
MODE_LINE_TRACKING   = 0   # Jetson is in command
MODE_HUMAN_FOLLOWING = 1   # Arduino is autonomous

# ====================== GATE DATABASE HELPER ======================

class GateDB:
    def get_first_recognized_gate(self, recognized_users: list):
        #Given a list of recognised passenger names (from face_rec), returns (gate_number, target_distance, passenger_name), or None
        for name in recognized_users:
            if name in PASSENGER_DATABASE:
                gate     = PASSENGER_DATABASE[name]
                distance = GATE_DATABASE[gate]
                return gate, distance, name
        return None


# ====================== ARDUINO COMM ======================

class ArduinoComm:
    """
    Manages serial communication with the Arduino Mega.
    Lines starting with "MODE:" update self.current_mode automatically.
    All other lines are printed as Arduino debug output.
    Mode values:
    MODE_LINE_TRACKING   (0) -- Jetson sends TRACK/STOP/LEFT/RIGHT.
    MODE_HUMAN_FOLLOWING (1) -- Arduino autonomous; Jetson waits.
    """

    def __init__(self, port, baud):
        self.ser          = serial.Serial(port, baud, timeout=1)
        self.current_mode = MODE_LINE_TRACKING   # default
        self.last_line    = ""
        self._lock        = threading.Lock()
        self._running     = True

        time.sleep(2)   # Wait for Arduino bootloader reset

        self._reader_thread = threading.Thread(
            target=self._read_loop, daemon=True
        )
        self._reader_thread.start()
        print(f"[Arduino] Connected on {port} -- default Mode 0 (Line Tracking)")

    def _read_loop(self):
        while self._running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                with self._lock:
                    self.last_line = line

                #Mode-switch detection
                if line.startswith("MODE:"):
                    try:
                        new_mode = int(line.split(":")[1])
                        with self._lock:
                            old_mode      = self.current_mode
                            self.current_mode = new_mode

                        if new_mode == MODE_LINE_TRACKING:
                            print(f"\n[Arduino] MODE -> 0  Line Tracking "
                                  f"(was {old_mode})")
                        elif new_mode == MODE_HUMAN_FOLLOWING:
                            print(f"\n[Arduino] MODE -> 1  Human Following "
                                  f"(was {old_mode})")
                        else:
                            print(f"\n[Arduino] MODE -> {new_mode} (unknown)")
                    except ValueError:
                        pass

                # Print all other Arduino debug lines
                else:
                    print(f"[Arduino] << {line}")

            except Exception:
                pass

    def send(self, cmd: str):
        #Send a newline-terminated command string to the Arduino.
        msg = cmd.strip() + "\n"
        with self._lock:
            self.ser.write(msg.encode())
        print(f"[Arduino] >> {cmd}")

    def get_mode(self) -> int:
        #Thread-safe read of the current Arduino mode.
        with self._lock:
            return self.current_mode

    def wait_for_mode(self, target_mode: int, poll_interval: float = 0.1):
        """
        Block until Arduino reports target_mode.
        Used to pause Jetson commands while in Human Following mode
        and resume when the Arduino switches back to Line Tracking.
        """
        while self.get_mode() != target_mode:
            time.sleep(poll_interval)

    def stop_thread(self):
        self._running = False

    def close(self):
        self.ser.close()


class UWBComm:
    #Reads distance from UWB tag serial stream.
    def __init__(self, port, baud):
        self.ser      = serial.Serial(port, baud, timeout=1)
        self.distance = 0.0
        self.baseline = 0.0
        self._running = True
        self._thread  = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print(f"[UWB] Connected on {port}")

    def _read_loop(self):
        while self._running:
            try:
                line = self.ser.readline().decode().strip()
                dist = self._parse_line(line)
                if dist is not None:
                    self.distance = dist
            except Exception:
                pass

    def _parse_line(self, line):
        try:
            return float(line.split("=")[1])
        except Exception:
            pass
        return None

    def calibrate(self):
        """Set current position as the origin (zero point)."""
        print("[UWB] Calibrating baseline...")
        time.sleep(1.0)
        self.baseline = self.distance
        print(f"[UWB] Baseline set to {self.baseline:.3f}m")

    def get_travelled(self) -> float:
        """Distance travelled since last calibrate() call."""
        return max(0.0, self.distance - self.baseline)

    def stop(self):
        self._running = False


# ====================== GATE GUIDANCE SYSTEM ======================

class GateGuidanceSystem:
    def __init__(self):
        print("=" * 54)
        print("  GATE GUIDANCE SYSTEM - Initializing")
        print("=" * 54)

        self.arduino = ArduinoComm(ARDUINO_PORT, ARDUINO_BAUD)
        self.uwb     = UWBComm(UWB_PORT, UWB_BAUD)
        self.gate_db = GateDB()

        # Navigation state -- updated each passenger cycle
        self.target_gate     = DEFAULT_GATE
        self.target_distance = DEFAULT_DISTANCE
        self.recognized_user = DEFAULT_USER

    # ==================================================================
    def run(self):
        try:
            # STEP 0: Calibrate UWB at start position
            print("\n[STEP 0] Calibrating UWB at start position...")
            self.uwb.calibrate()
            self.arduino.send("STOP")
            print("[STEP 0] System ready.\n")

            while True:
                self._guide_one_passenger()
                time.sleep(2)
                return

        except KeyboardInterrupt:
            print("\n[System] Shutting down...")
        finally:
            self.arduino.send("STOP")
            self.arduino.stop_thread()
            self.uwb.stop()
            self.arduino.close()

    def _guide_one_passenger(self):

        # STEP 1 -- Face recognition
        print("[STEP 1] Running face recognition...")
        recognized_users = face_rec.face_rec()

        if recognized_users:
            result = self.gate_db.get_first_recognized_gate(recognized_users)
            if result:
                self.target_gate, self.target_distance, self.recognized_user = result
                print(f"\n  Recognized  : {recognized_users}")
                print(f"  Gate        : {self.target_gate}")
                print(f"  Target dist : {self.target_distance}m")
                print(f"  Turn        : {'RIGHT' if self.target_gate % 2 == 0 else 'LEFT'}")
            else:
                print(f"\n  Faces detected {recognized_users} but none in passenger DB")
                print(f"  Using defaults -> Gate {DEFAULT_GATE}, {DEFAULT_DISTANCE}m")
                self.target_gate     = DEFAULT_GATE
                self.target_distance = DEFAULT_DISTANCE
                self.recognized_user = DEFAULT_USER
        else:
            print("\n  No faces recognised -- using defaults")
            self.target_gate     = DEFAULT_GATE
            self.target_distance = DEFAULT_DISTANCE
            self.recognized_user = DEFAULT_USER

        gate_number     = self.target_gate
        target_distance = self.target_distance
        turn_direction  = "right" if gate_number % 2 == 0 else "left"

        print(f"\n[STEP 1] Passenger : {self.recognized_user}")
        print(f"         Gate      : {gate_number}")
        print(f"         Target    : {target_distance}m")
        print(f"         Turn      : {turn_direction}")

        # STEP 2 -- Navigate to gate
        print(f"\n[STEP 2] Starting navigation to Gate {gate_number}...")
        self.uwb.calibrate()   # Reset origin to current position

        # Make sure we start in Line Tracking mode before issuing TRACK
        if self.arduino.get_mode() == MODE_HUMAN_FOLLOWING:
            print("[STEP 2] Arduino is in Human Following mode at nav start.")
            self._wait_for_line_tracking_mode()

        self.arduino.send("forward")

        while True:
            # -- Mode 1: Human Following -- Jetson yields control ----------
            if self.arduino.get_mode() == MODE_HUMAN_FOLLOWING:
                self._handle_human_following_mode()
                # Arduino is back in Mode 0; re-issue TRACK to continue nav
                self.arduino.send("forward")

            # -- Mode 0: Line Tracking -- Jetson monitors UWB --------------
            travelled = self.uwb.get_travelled()
            remaining = target_distance - travelled

            print(
                f"[NAV] Mode: {self.arduino.get_mode()}  |  "
                f"Travelled: {travelled:.2f}m  |  "
                f"Remaining: {remaining:.2f}m",
                end="\r"
            )

            if remaining <= DISTANCE_TOLERANCE:
                print(f"\n[STEP 2] Reached Gate {gate_number} at {travelled:.2f}m")
                break

            time.sleep(0.05)   # 50 ms polling loop

        # STEP 3 -- Stop and turn
        print(f"\n[STEP 3] Stopping and turning {turn_direction}...")
        self.arduino.send("STOP")
        time.sleep(0.5)
        self.arduino.send(turn_direction)   # "LEFT" or "RIGHT"
        time.sleep(TURN_DURATION)
        self.arduino.send("STOP")
        time.sleep(0.2)
        

        # STEP 4 -- Done
        print(f"[STEP 4] Passenger '{self.recognized_user}' "
              f"delivered to Gate {gate_number}.")

    # MODE HELPERS

    def _handle_human_following_mode(self):
        #Called mid-navigation when the Arduino switches to Human Following.
        print("\n[MODE 1] Human Following -- Arduino is autonomous.")
        print("[MODE 1] Jetson suspended. Waiting for MODE:0 ...")

        self.arduino.wait_for_mode(MODE_LINE_TRACKING)

        print("[MODE 0] Line Tracking resumed -- Jetson back in command.")

    def _wait_for_line_tracking_mode(self):
        #Block until Arduino is in Mode 0, with a one-time warning.
        print(f"[WARN] Arduino is in Mode {self.arduino.get_mode()}. "
              f"Waiting for Line Tracking (Mode 0)...")
        self.arduino.wait_for_mode(MODE_LINE_TRACKING)
        print("[INFO] Mode 0 confirmed -- proceeding.")


if __name__ == "__main__":
    system = GateGuidanceSystem()
    system.run()
