# AeroCart
An Autonomous Mobile Robot using Arduino Mega, servo/DC motors, sensors, and Nvidia Jetson Nano

**Key Features:**

1. RFID Triangulation: 4 RC522 readers mounted on corners for direction estimation
2. Mecanum Wheel Drive: Full omnidirectional movement (forward, backward, strafe, rotate)
3. Ultrasonic Obstacle Detection: 4 HC-SR04 sensors for 360° obstacle detection
4. OLED Display: Real-time status and sensor data
5. MPU6050 Gyroscope: Heading stabilization (can be used for orientation)
6. Jetson Nano Integration: Serial communication for face detection
7. State Machine: 6 states with smooth transitions
8. LED Indicators: Visual status feedback
9. Buzzer Alerts: Audio feedback for tag acquisition/loss

    
**Pin Mapping Summary:**

Component;	Pins
RFID Readers	SS: 53,52,51,50; RST: 22,24,26,28
Ultrasonic	Trig: 62,64,66,68; Echo: 61,63,65,67
Motors	PWM: 12,8,9,5; DIR: 34,35,37,36,43,42,A4,A5
OLED	SDA/SCL (I2C), Reset: 28
MPU6050	SDA/SCL (I2C)
LEDs	Blue:11, Green:13, Red:14
Buzzer	15


** Final Code files to use: **
full_arduino_robot.ino
face_rec.py
headshots-backup.py
headshots.py
jetson_gate_guide.py 
jetsonnanoUWB.py
main.py
model_training.py
PC_UWB_Anchor.py
