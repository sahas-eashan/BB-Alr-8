import cv2
import numpy as np
import time
from kobukidriver import Kobuki
import threading


class ColorBoxFollower:
    def __init__(self, target_color):
        self.robot = Kobuki()
        self.cap = cv2.VideoCapture(0)
        self.direction = "Stop"
        self.running = True
        self.last_detection_time = time.time()
        self.timeout = 2  # seconds to stop if no color detected
        self.target_color = target_color

        # Define color ranges
        self.color_ranges = {
            "blue": (np.array([100, 100, 100]), np.array([140, 255, 255])),
            "red": (np.array([0, 120, 70]), np.array([10, 255, 255])),
            "green": (np.array([40, 40, 40]), np.array([80, 255, 255])),
            "yellow": (np.array([20, 100, 100]), np.array([30, 255, 255])),
        }

    def detect_color_box(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detected_color = None
        mask = None

        if self.target_color in self.color_ranges:
            lower, upper = self.color_ranges[self.target_color]
            mask = cv2.inRange(hsv, lower, upper)
        else:
            # Check for unknown color
            masks = []
            for color, (lower, upper) in self.color_ranges.items():
                color_mask = cv2.inRange(hsv, lower, upper)
                if np.any(color_mask):
                    masks.append((color_mask, color))

            if masks:
                mask, detected_color = max(masks, key=lambda x: cv2.countNonZero(x[0]))
            else:
                detected_color = "unknown"
                mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        color_detected = False

        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Adjust area threshold as needed
                color_detected = True
                self.last_detection_time = time.time()

                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                label = detected_color if detected_color else self.target_color
                cv2.putText(
                    frame,
                    f"{label.capitalize()} Box",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )

                # Determine position
                frame_center = frame.shape[1] // 2
                box_center = x + w // 2

                if box_center < frame_center - 50:
                    self.direction = "Left"
                elif box_center > frame_center + 50:
                    self.direction = "Right"
                else:
                    self.direction = "Forward"

                cv2.putText(
                    frame,
                    self.direction,
                    (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )
                break

        if not color_detected:
            if time.time() - self.last_detection_time > self.timeout:
                self.direction = "Stop"
                cv2.putText(
                    frame,
                    "No target detected - Stopped",
                    (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

        return frame

    def control_robot(self):
        while self.running:
            if self.direction == "Forward":
                self.robot.move(80, 80, 0)
            elif self.direction == "Left":
                self.robot.move(40, 80, 0)
            elif self.direction == "Right":
                self.robot.move(80, 40, 0)
            elif self.direction == "Stop":
                self.robot.move(0, 0, 0)

            time.sleep(0.1)

    def run(self):
        robot_thread = threading.Thread(target=self.control_robot)
        robot_thread.daemon = True
        robot_thread.start()

        self.robot.play_on_sound()

        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    break

                frame = self.detect_color_box(frame)

                cv2.imshow(f"{self.target_color.capitalize()} Box Tracking", frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        finally:
            self.running = False
            self.robot.move(0, 0, 0)
            self.robot.play_off_sound()
            self.cap.release()
            cv2.destroyAllWindows()
            robot_thread.join(timeout=1.0)


def main():
    color = (
        input(
            "Enter the color of the cube to follow (blue, red, green, yellow, unknown): "
        )
        .strip()
        .lower()
    )
    while color not in ["blue", "red", "green", "yellow", "unknown"]:
        print("Invalid color choice. Choose from blue, red, green, yellow, or unknown.")
        color = (
            input(
                "Enter the color of the cube to follow (blue, red, green, yellow, unknown): "
            )
            .strip()
            .lower()
        )

    follower = ColorBoxFollower(color)
    follower.run()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"An error occurred: {e}")
