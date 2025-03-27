import cv2
import numpy as np
import math
import time
from controller import Robot, Camera, Motor
import kobukidriver  # Assuming standard Kobuki controller library


class BBAlr8OpenCVController:
    def __init__(self, target_colors=None, stop_threshold=0.7):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        # Motors
        self.left_motor = self.robot.getDevice("leftMotor")
        self.right_motor = self.robot.getDevice("rightMotor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Camera
        self.camera = self.robot.getDevice("fcam")
        self.camera.enable(self.time_step)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()

        # Parameters
        self.target_colors = target_colors or [
            "red",
            "blue",
            "green",
            "yellow",
            "unknown",
        ]
        self.stop_threshold = stop_threshold
        self.MAX_SPEED = 7.0

        self.color_ranges = {
            "red": ([0, 100, 100], [10, 255, 255]),
            "blue": ([100, 100, 100], [140, 255, 255]),
            "green": ([40, 100, 100], [80, 255, 255]),
            "yellow": ([20, 100, 100], [30, 255, 255]),
        }

        # Initialize Kobuki Driver
        kobukidriver.init()

    def detect_color_object(self, frame):
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        best_match = None
        best_area = 0

        frame_with_detection = frame.copy()

        for color in self.target_colors:
            if color == "unknown":
                continue
            lower, upper = self.color_ranges.get(color, ([0, 100, 100], [10, 255, 255]))
            mask = cv2.inRange(img_hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                if area > best_area:
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        best_match = {
                            "color": color,
                            "x": cx,
                            "y": cy,
                            "area": area,
                            "frame": frame_with_detection,
                        }
                        best_area = area

        if best_match is not None:
            label = f"{best_match['color'].capitalize()} Object"
            cv2.putText(
                frame_with_detection,
                label,
                (best_match["x"], best_match["y"]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )
            return best_match

        return {"color": "unknown", "frame": frame_with_detection, "x": None, "y": None}

    def adjust_movement(self, object_info):
        if object_info["x"] is None:
            return

        center = self.width // 2
        error = object_info["x"] - center
        turn_rate = error * 0.1

        left_speed = self.MAX_SPEED - turn_rate
        right_speed = self.MAX_SPEED + turn_rate
        left_speed = max(min(left_speed, self.MAX_SPEED), -self.MAX_SPEED)
        right_speed = max(min(right_speed, self.MAX_SPEED), -self.MAX_SPEED)

        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def move_forward(self):
        self.left_motor.setVelocity(self.MAX_SPEED)
        self.right_motor.setVelocity(self.MAX_SPEED)

    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def run_distance(self, distance_cm, speed=0.2):
        distance_m = distance_cm / 100.0
        start_pose = kobukidriver.get_odometry()

        kobukidriver.set_velocity(speed, speed)
        while self.robot.step(self.time_step) != -1:
            current_pose = kobukidriver.get_odometry()
            dx = current_pose[0] - start_pose[0]
            dy = current_pose[1] - start_pose[1]
            traveled = math.sqrt(dx * dx + dy * dy)
            if traveled >= distance_m:
                break

        kobukidriver.set_velocity(0, 0)

    def turn_angle(self, direction, angle_deg=90, turn_speed=0.2):
        angle_rad = math.radians(angle_deg)
        left_speed, right_speed = (0, 0)

        if direction.lower() == "left":
            left_speed = -turn_speed
            right_speed = turn_speed
        elif direction.lower() == "right":
            left_speed = turn_speed
            right_speed = -turn_speed
        else:
            print("Invalid direction. Use 'left' or 'right'.")
            return

        angular_speed = 0.5 * (turn_speed / 0.2)
        turn_time = angle_rad / angular_speed

        kobukidriver.set_velocity(left_speed, right_speed)
        start_time = self.robot.getTime()
        while self.robot.step(self.time_step) != -1:
            if self.robot.getTime() - start_time >= turn_time:
                break

        kobukidriver.set_velocity(0, 0)

    def run(self):
        print("Moving forward 50 cm...")
        self.run_distance(50, speed=0.2)
        print("Turning left 90°...")
        self.turn_angle("left", angle_deg=90, turn_speed=0.2)
        print("Turning right 90°...")
        self.turn_angle("right", angle_deg=90, turn_speed=0.2)


def main():
    controller = BBAlr8OpenCVController(
        target_colors=["red", "blue", "green", "yellow", "unknown"], stop_threshold=0.7
    )
    controller.run()


if __name__ == "__main__":
    main()
