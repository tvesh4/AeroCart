import cv2
import os
from datetime import datetime
import time
import face_recognition

# Change this to the name of the person you're photographing
PERSON_NAME = "YOUR_NAME"


def gstreamer_pipeline(
        sensor_id=0,
        capture_width=1920,
        capture_height=1080,
        display_width=960,
        display_height=540,
        framerate=30,
        flip_method=0,
):
    return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
    )


def create_folder(name):
    dataset_folder = "dataset"
    if not os.path.exists(dataset_folder):
        os.makedirs(dataset_folder)

    person_folder = os.path.join(dataset_folder, name)
    if not os.path.exists(person_folder):
        os.makedirs(person_folder)
    return person_folder


def detect_face(frame):
    """
    Detect if there's a face in the frame.
    Returns True if a face is detected, False otherwise.
    """
    try:
        # Resize frame for faster processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

        # Detect faces using HOG model (faster) or CNN model (more accurate)
        face_locations = face_recognition.face_locations(rgb_small_frame, model="cnn")

        return len(face_locations) > 0
    except Exception as e:
        print(f"Face detection error: {e}")
        return False


def draw_face_box(frame):
    """
    Draw a box around detected faces for visual feedback.
    Returns the modified frame.
    """
    try:
        # Resize frame for processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

        # Detect faces
        face_locations = face_recognition.face_locations(rgb_small_frame, model="cnn")

        # Draw boxes on original frame size
        for (top, right, bottom, left) in face_locations:
            # Scale back up since we resized the frame
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            # Draw rectangle around face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

        return frame, len(face_locations) > 0
    except Exception as e:
        print(f"Face drawing error: {e}")
        return frame, False


def capture_photos(name):
    start_second = time.time()
    folder = create_folder(name)

    # Initialize the CSI camera using GStreamer pipeline
    cap = cv2.VideoCapture(gstreamer_pipeline(sensor_id=0, capture_width=640, capture_height=480, display_width=640, display_height=480, framerate=30, flip_method=0), cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Error: Could not open CSI camera.")
        return

    photo_count = 0
    last_capture_time = 0

    print(f"Taking photos for {name}.")
    print(f"Position your face in front of the camera.")
    print(f"Auto-capturing up to 30 photos within 30 seconds when face is detected.")
    print(f"Press 'q' to quit early.\n")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Warning: Failed to grab frame.")
            continue  # Skip this iteration and try again

        # Detect face and draw box if found
        display_frame, face_detected = draw_face_box(frame)

        # Add status text
        status_text = "FACE DETECTED" if face_detected else "NO FACE"
        status_color = (0, 255, 0) if face_detected else (0, 0, 255)
        cv2.putText(display_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, status_color, 2)

        # Display the frame
        cv2.imshow('Capture', display_frame)

        key = cv2.waitKey(1) & 0xFF

        # Check for quit key
        if key == ord('q'):
            break

        curr_second = time.time()
        time_elapsed = curr_second - start_second

        # Auto-capture logic: capture if face detected, with 0.3s delay between captures
        if face_detected and photo_count < 30 and time_elapsed < 30:
            if curr_second - last_capture_time > 0.3:  # Prevent capturing too fast
                photo_count += 1
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{name}_{timestamp}_{photo_count}.jpg"
                filepath = os.path.join(folder, filename)
                cv2.imwrite(filepath, frame)  # Save original frame, not display_frame
                print(f"Photo {photo_count} saved: {filepath}")
                last_capture_time = curr_second

        # Exit conditions: 5 photos captured or 30 seconds elapsed
        if photo_count >= 30 or time_elapsed >= 30:
            break

    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    print(f"Photo capture completed. {photo_count} photos saved for {name}.")


if __name__ == "__main__":
    capture_photos(PERSON_NAME)