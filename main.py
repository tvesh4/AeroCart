#!/usr/bin/env python3
"""
AeroCart - Autonomous Luggage-Carrying Robot FSM
Main control program for Jetson Nano
Integrates: UWB tracking, facial recognition, Arduino servo control, and motor commands
"""

import sys
import os
import time
import threading
import queue
import serial
import logging
import pickle
from enum import Enum
from dataclasses import dataclass
from typing import Optional

# Import project modules
try:
    import headshots
    import model_training
    import jetsonnanoUWB
    import jetson_gate_guide
    import face_recognition
    import face_rec
    import cv2
    import numpy as np
except ImportError as e:
    print(f"[ERROR] Missing required module: {e}")
    print("[ERROR] Make sure all project files are present")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(levelname)s] - %(message)s'
)
logger = logging.getLogger(__name__)


# ============================================================
# FSM State Definitions
# ============================================================

class SystemState(Enum):
    """Robot system states"""
    IDLE = "idle"
    TRAINING = "training"
    OPENING_COMPARTMENT = "opening_compartment"
    CLOSING_COMPARTMENT = "closing_compartment"
    FOLLOWING = "following"
    STORAGE = "storage"
    SHUTDOWN = "shutdown"


@dataclass
class UserProfile:
    """User profile data"""
    name: str
    face_encoding: Optional[np.ndarray] = None
    uwb_distance: float = 0.0
    last_seen: float = 0.0


# ============================================================
# Arduino Communication Handler
# ============================================================

class ArduinoHandler:
    """Manages communication with Arduino Mega"""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200):#1a86:7523
        self.port = port
        self.baud = baud
        self.ser = None
        self.connected = False

    def connect(self) -> bool:
        """Establish connection to Arduino"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            time.sleep(2)  # Wait for Arduino to initialize
            logger.info(f"✓ Arduino connected on {self.port}")
            self.connected = True
            return True
        except Exception as e:
            logger.error(f"✗ Arduino connection failed: {e}")
            self.connected = False
            return False

    def send_motor_command(self, left_speed: int, right_speed: int) -> bool:
        """Send motor speed command to Arduino"""
        if not self.connected or not self.ser:
            return False

        try:
            # Format: M,left_speed,right_speed\n
            command = f"M,{left_speed},{right_speed}\n"
            self.ser.write(command.encode())
            return True
        except Exception as e:
            logger.error(f"Failed to send motor command: {e}")
            return False

    def open_compartment(self) -> bool:
        """Send servo open command"""
        if not self.connected or not self.ser:
            return False

        try:
            # Format: O for open
            command = "open"
            self.ser.write(command.encode())
            logger.info("Compartment open command sent")
            return True
        except Exception as e:
            logger.error(f"Failed to send open command: {e}")
            return False

    def close_compartment(self) -> bool:
        """Send servo close command"""
        if not self.connected or not self.ser:
            return False

        try:
            # Format: C for close
            command = "close"
            self.ser.write(command.encode())
            logger.info("Compartment close command sent")
            return True
        except Exception as e:
            logger.error(f"Failed to send close command: {e}")
            return False

    def disconnect(self):
        """Close connection"""
        if self.ser:
            self.ser.close()
            self.connected = False
            logger.info("Arduino disconnected")


# ============================================================
# Button Input Handler
# ============================================================

class ButtonHandler:
    """Handles button input for state transitions"""

    def __init__(self, pin: int = 21):
        """
        Initialize button handler
        Note: Requires GPIO setup or modify for your specific hardware
        """
        self.pin = pin
        self.pressed = False
        self.last_press_time = 0
        self.debounce_time = 0.5  # seconds
        self.button_queue = queue.Queue()

        # Try to import RPi.GPIO if available
        try:
            import Jetson.GPIO as GPIO
            self.gpio = GPIO
            self.has_gpio = True
            self._setup_gpio()
        except ImportError:
            logger.warning("RPi.GPIO not available - using serial input for button simulation")
            self.has_gpio = False

    def _setup_gpio(self):
        """Setup GPIO for button"""
        try:
            self.gpio.setmode(self.gpio.BOARD)
            self.gpio.setup(self.pin, self.gpio.IN, pull_up_down=self.gpio.PUD_UP)
            self.gpio.add_event_detect(
                self.pin,
                self.gpio.FALLING,
                callback=self._button_callback,
                bouncetime=200
            )
            logger.info(f"✓ Button initialized on GPIO pin {self.pin}")
        except Exception as e:
            logger.error(f"Failed to setup GPIO: {e}")
            self.has_gpio = False

    def _button_callback(self, channel):
        """GPIO callback when button is pressed"""
        current_time = time.time()
        if current_time - self.last_press_time > self.debounce_time:
            self.pressed = True
            self.last_press_time = current_time
            logger.info("Button pressed!")

    def check_press(self) -> bool:
        """Check if button was pressed"""
        if self.has_gpio:
            pressed = self.pressed
            self.pressed = False
            return pressed
        else:
            # Fallback: check for keyboard input (for testing)
            try:
                key = input("Press ENTER or type 'q' to quit: ")
                if key.lower() == 'q':
                    return None  # Signal quit
                return True if key == "" else False
            except:
                return False

    def cleanup(self):
        """Cleanup GPIO"""
        if self.has_gpio:
            try:
                self.gpio.cleanup()
            except:
                pass


# ============================================================
# Facial Recognition Engine
# ============================================================


    def cleanup(self):
        """Release camera resources"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()


# ============================================================
# Main FSM Controller
# ============================================================

class RobotFSM:
    """Finite State Machine for robot operation"""

    def __init__(self):
        self.previous_state = SystemState.IDLE
        self.state = SystemState.IDLE
        self.next_state = SystemState.IDLE
        self.current_user = None
        self.running = True

        # Initialize hardware handlers
        logger.info("Initializing hardware...")
        self.arduino = ArduinoHandler()
        self.arduino.connect()
        self.button = ButtonHandler()
        # self.face_engine = FacialRecognitionEngine()

        # Initialize UWB handlers (ports may need adjustment)
        self.master_uwb = None
        self.slave_uwb = None
        self.uwb_initialized = False
        self._initialize_uwb()

        # Robot state for following
        self.decision_engine = None
        self.user_name = None

        logger.info("=" * 60)
        logger.info("AeroCart Robot FSM Initialized")
        logger.info("=" * 60)

    def _initialize_uwb(self):
        """Initialize UWB modules"""
        try:
            # Find available serial ports
            import glob
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/tty.usbmodem*')


        except Exception as e:
            logger.error(f"UWB initialization error: {e}")
            self.uwb_initialized = False

    def run(self):
        """Main FSM loop"""
        try:
            while self.running:
                self.process_state()
                time.sleep(0.1)  # Prevention of busy-waiting

        except KeyboardInterrupt:
            logger.info("\n[INFO] Shutdown signal received")
        finally:
            self.cleanup()

    def reset_fsm_flags(self):
        """Resets all state-specific initialization flags for a new run"""
        # self._idle_logged = False
        # self._training_started = False
        # self._opening_started = False
        # self._closing_started = False
        # self._following_started = False
        # self._storage_started = False
        # self.current_user = None
        # self.user_name = None
        for attr_name in [
            "_idle_logged",
            "_training_started",
            "_opening_started",
            "_closing_started",
            "_following_started",
            "_storage_started",
            "_known_user"
        ]:
            if hasattr(self, attr_name):
                delattr(self, attr_name)
        self.current_user = None
        self.user_name = None
        logger.info("FSM flags reset for new user cycle.")

    def _recognize_users(self) -> list:
        """Run face recognition and normalize the returned user list."""
        try:
            recognized_users = face_rec.face_rec()
        except SystemExit:
            logger.error("Face recognition exited unexpectedly (camera/model issue).")
            return []
        except Exception as e:
            logger.error(f"Face recognition failed: {e}")
            return []

        if not recognized_users:
            return []
        if isinstance(recognized_users, str):
            return [recognized_users]
        return [name for name in recognized_users if name]

    def process_state(self):
        """Process current state and handle transitions"""

        if self.state == SystemState.IDLE:
            self._state_idle()
        elif self.state == SystemState.TRAINING:
            self._state_training()
        elif self.state == SystemState.OPENING_COMPARTMENT:
            self._state_opening_compartment()
        elif self.state == SystemState.CLOSING_COMPARTMENT:
            self._state_closing_compartment()
        elif self.state == SystemState.FOLLOWING:
            self._state_following()
        elif self.state == SystemState.STORAGE:
            self._state_storage()
        elif self.state == SystemState.SHUTDOWN:
            self.running = False

        # Update state
        self.previous_state = self.state
        self.state = self.next_state

    def _state_idle(self):
        """IDLE state - waiting for user interaction"""
        if not hasattr(self, '_idle_logged'):
            logger.info(f"[STATE] IDLE - Waiting for user button press")
            self._idle_logged = True

        recognized_users = self._recognize_users()
        known_users = [name for name in recognized_users if name != "Unknown"]
        if known_users:
            logger.info(f"Recognized users in IDLE: {known_users}")
        # Check for button press
        button_pressed = self.button.check_press()

        if button_pressed is None:  # Quit signal
            self.next_state = SystemState.SHUTDOWN
            return

        if known_users:
            self.user_name = known_users[0]
            self.next_state = SystemState.OPENING_COMPARTMENT
            self._idle_logged = False
        elif button_pressed:
            logger.info("User initiated - entering TRAINING state")
            self.next_state = SystemState.TRAINING
            self._idle_logged = False

    def _state_training(self):
        """TRAINING state - facial recognition model training"""
        train_status = False
        if not hasattr(self, '_training_started'):
            logger.info(f"[STATE] TRAINING - Starting facial recognition training")
            logger.info("Taking headshots for user identification...")

            try:
                # Capture photos for the user
                self.user_name = self._get_user_name()


                if (self.user_name == "admin"): self._training_started = True
                else: train_status = headshots.capture_photos(self.user_name)
                # Train the model
                logger.info("Training facial recognition model...")

                if train_status:
                    model_training.train_model()
                    logger.info("Training complete. Please press button to continue.")
                    self._training_started = True
            except Exception as e:
                logger.error(f"Training failed: {e}")
                self.next_state = SystemState.IDLE
                self._training_started = False
                return

        # Wait for button press to open compartment
        button_pressed = self.button.check_press()

        if button_pressed and hasattr(self, '_training_started'):
            logger.info("Training complete - entering compartment opening state")
            self.next_state = SystemState.OPENING_COMPARTMENT
            self._training_started = False

    def _state_opening_compartment(self):
        """OPENING_COMPARTMENT state - open storage servo"""
        if not hasattr(self, '_opening_started'):
            logger.info(f"[STATE] OPENING_COMPARTMENT - Opening storage compartment")

            # Send open command to Arduino
            if self.arduino.connected:
                success = self.arduino.open_compartment()
                if success:
                    logger.info("Compartment opened successfully")
                else:
                    logger.warning("Failed to open compartment")
            else:
                logger.warning("Arduino not connected - cannot open compartment")

            self._opening_started = True
            self.compartment_open_time = time.time()

        # Wait for user to place luggage and press button
        button_pressed = self.button.check_press()

        if button_pressed:
            logger.info("Item placed - closing compartment")
            self.next_state = SystemState.CLOSING_COMPARTMENT
            self._opening_started = False

    def _state_closing_compartment(self):
        """CLOSING_COMPARTMENT state - close storage servo"""
        if not hasattr(self, '_closing_started'):
            logger.info(f"[STATE] CLOSING_COMPARTMENT - Closing storage compartment")

            # Send close command to Arduino
            if self.arduino.connected:
                success = self.arduino.close_compartment()
                if success:
                    logger.info("Compartment closed successfully")
                else:
                    logger.warning("Failed to close compartment")
            else:
                logger.warning("Arduino not connected - cannot close compartment")

            self._closing_started = True
            self.compartment_close_time = time.time()

        # Wait for user to press button to start following
        button_pressed = self.button.check_press()

        if button_pressed:
            logger.info("Compartment closed - entering FOLLOWING state")
            self.next_state = SystemState.FOLLOWING
            self._closing_started = False

    def _state_following(self):
        """FOLLOWING state - follow user using UWB"""
        if not hasattr(self, '_following_started'):
            logger.info(f"[STATE] FOLLOWING - Following user with UWB")
            self.arduino.disconnect()
            jetson_gate_guide.GateGuidanceSystem()
            self.arduino.connect()

        # Check for button press to enter storage state
        button_pressed = self.button.check_press()

        if button_pressed:
            logger.info("User pressed button - entering STORAGE state")
            self.next_state = SystemState.STORAGE
            self._following_started = False

    def _state_storage(self):
        """STORAGE state - verify user and unlock compartment"""
        if not hasattr(self, '_storage_started'):
            logger.info(f"[STATE] STORAGE - Verifying user and checking proximity")
            self._storage_started = True
            self.storage_start_time = time.time()

        # Check condition 1: UWB distance (user is close)
        # uwb_ok = False
        # if self.current_user:
        #     time_since_uwb = time.time() - self.current_user.last_seen
        #     uwb_distance_ok = (
        #         self.current_user.uwb_distance > 0 and
        #         self.current_user.uwb_distance < 1.5 and  # Within 1.5 meters
        #         time_since_uwb < 2.0  # Recent reading
        #     )
        #     uwb_ok = uwb_distance_ok
        #     logger.info(f"UWB Check: Distance={self.current_user.uwb_distance:.2f}m, OK={uwb_ok}")
        uwb_ok = True

        # Check condition 2: Facial recognition (correct user)
        face_ok = False
        recognized_users = self._recognize_users()
        logger.info(f"Found users: {recognized_users}")
        print(self.user_name)
        print(recognized_users)

        if recognized_users:
            if self.user_name in recognized_users:
                face_ok = True
                logger.info(f"Face Check: User {self.user_name} verified, OK=True")
            else:
                logger.warning(f"Face Check: Unknown user {self.user_name}")
        else:
            logger.info("Face Check: No face detected")

        # If both conditions met, unlock compartment
        if uwb_ok and face_ok:
            logger.info("✓ User verified! Opening compartment for unloading")
            if self.arduino.connected:
                self.arduino.open_compartment()
                time.sleep(3)
                button_pressed = self.button.check_press()
                while button_pressed == False:
                    button_pressed = self.button.check_press()
                self.arduino.close_compartment()

            # Return to IDLE after user unloads
            time.sleep(3)  # Give user time to interact
            self.reset_fsm_flags()
            #TODO maybe clear the images and pickles generated in this, or can keep the images if building multi person system idk
            logger.info("Returning to IDLE state")
            self.next_state = SystemState.IDLE
            self._storage_started = False
            self.current_user = None

        # Timeout - return to FOLLOWING if verification fails
        elif time.time() - self.storage_start_time > 15.0:
            logger.warning("Storage verification timeout - returning to FOLLOWING")
            self.next_state = SystemState.FOLLOWING
            self._storage_started = False

    def _get_user_name(self) -> str:
        """Get user name for training"""
        try:
            # Try to read from input
            name = input("\n[INPUT] Enter user name for facial recognition training: ").strip()
            if name:#big errors starting from here
                return name
            else:
                return "USER"
        except:
            return "USER"

    def cleanup(self):
        """Cleanup resources"""
        logger.info("\n[INFO] Cleaning up resources...")

        # Stop motors
        if self.arduino.connected:
            self.arduino.send_motor_command(0, 0)

        # Disconnect Arduino
        if self.arduino:
            self.arduino.disconnect()

        # Stop UWB modules
        if self.master_uwb:
            self.master_uwb.stop()
        if self.slave_uwb:
            self.slave_uwb.stop()



        # Cleanup button GPIO
        if self.button:
            self.button.cleanup()

        logger.info("All resources cleaned up")
        logger.info("=" * 60)
        logger.info("AeroCart Robot FSM Shut Down")
        logger.info("=" * 60)


# ============================================================
# Main Entry Point
# ============================================================

def main():
    """Main entry point"""
    print("\n" + "=" * 60)
    print("AeroCart - Autonomous Luggage-Carrying Robot")
    print("FSM Control System for Jetson Nano")
    print("=" * 60 + "\n")

    # Create and run FSM
    fsm = RobotFSM()
    fsm.run()


if __name__ == "__main__":
    main()

