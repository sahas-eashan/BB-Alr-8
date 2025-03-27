import cv2
import numpy as np
import time
from kobukidriver import Kobuki
import threading


class ColorBoxFollower:
    def __init__(self):
        self.robot = Kobuki()
        self.cap = cv2.VideoCapture(0)
        self.direction = "Stop"
        self.running = True
        self.last_detection_time = time.time()
        self.timeout = 2  # seconds to stop if no color detected
        self.current_task = "blue"
        self.task_sequence = ["blue", "red", "yellow", "green"]

        # Define color ranges
        self.color_ranges = {
            "blue": (np.array([100, 100, 100]), np.array([140, 255, 255])),
            "red": (np.array([0, 120, 70]), np.array([10, 255, 255])),
            "green": (np.array([40, 40, 40]), np.array([80, 255, 255])),
            "yellow": (np.array([20, 100, 100]), np.array([30, 255, 255])),
            "white": (np.array([0, 0, 200]), np.array([180, 30, 255])),
        }

    def detect_color_box(self, frame, color):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower, upper = self.color_ranges[color]
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Adjust threshold as needed
                x, y, w, h = cv2.boundingRect(contour)
                return (x, y, w, h)
        return None

    def detect_obstacles(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower, upper = self.color_ranges["white"]
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

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
            while self.running and self.current_task:
                ret, frame = self.cap.read()
                if not ret:
                    break

                box_coords = self.detect_color_box(frame, self.current_task)
                area_coords = self.detect_color_box(frame, self.current_task)
                obstacles = self.detect_obstacles(frame)

                if box_coords is None:
                    self.direction = "Stop"
                else:
                    x, y, w, h = box_coords
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)

                    frame_center = frame.shape[1] // 2
                    box_center = x + w // 2

                    # Obstacle Avoidance
                    for obstacle in obstacles:
                        if (
                            cv2.contourArea(obstacle) > 500
                            and cv2.pointPolygonTest(
                                obstacle, (box_center, y + h // 2), False
                            )
                            >= 0
                        ):
                            self.direction = (
                                "Left" if box_center > frame_center else "Right"
                            )
                            break
                    else:
                        # Box Alignment and Pushing
                        if box_center < frame_center - 50:
                            self.direction = "Left"
                        elif box_center > frame_center + 50:
                            self.direction = "Right"
                        else:
                            self.direction = "Forward"

                    # Check if box is near the matching area
                    if area_coords:
                        ax, ay, aw, ah = area_coords
                        if abs(x - ax) < 30 and abs(y - ay) < 30:
                            self.direction = "Stop"
                            print(f"{self.current_task.capitalize()} Box Delivered!")
                            self.task_sequence.pop(0)
                            self.current_task = (
                                self.task_sequence[0] if self.task_sequence else None
                            )

                cv2.putText(
                    frame,
                    self.direction,
                    (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow(
                    (
                        f"{self.current_task.capitalize()} Box Tracking"
                        if self.current_task
                        else "Task Complete"
                    ),
                    frame,
                )

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
    follower = ColorBoxFollower()
    follower.run()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"An error occurred: {e}")
