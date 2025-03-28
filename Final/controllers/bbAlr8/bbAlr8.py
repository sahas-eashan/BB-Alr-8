from controller import Robot, Camera, Motor
import numpy as np
import cv2

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
        self.target_colors = target_colors or ["red", "cyan", "green", "yellow", "white"]
        self.stop_threshold = stop_threshold
        self.MAX_SPEED = 7.0
        self.detected_objects = set()

        # Optimized color ranges for accurate detection
        self.color_ranges = {
            "red": ([0, 80, 50], [10, 255, 255]),
            "red2": ([170, 80, 50], [180, 255, 255]), # Covers hue wrap-around
            "cyan": ([85, 80, 80], [100, 255, 255]), # Pure cyan detection
            "green": ([40, 80, 80], [90, 255, 255]),
            "yellow": ([20, 100, 100], [35, 255, 255]),
            "white": ([0, 0, 220], [180, 30, 255]),
        }

    def get_mask(self, img_hsv, color):
        """
        Generate mask using color ranges with combined red mask support.
        """
        if color == "red":
            mask1 = cv2.inRange(img_hsv, np.array(self.color_ranges["red"][0]), np.array(self.color_ranges["red"][1]))
            mask2 = cv2.inRange(img_hsv, np.array(self.color_ranges["red2"][0]), np.array(self.color_ranges["red2"][1]))
            return cv2.bitwise_or(mask1, mask2)
        else:
            lower, upper = self.color_ranges.get(color, ([0, 0, 0], [0, 0, 0]))
            return cv2.inRange(img_hsv, np.array(lower), np.array(upper))

    def classify_by_distance(self, area, width, height):
        """
        Classify objects based on size and distance using bounding box analysis.
        """
        size_ratio = width * height
        if area < 3000 or size_ratio < 20000:
            return "Small Cube"
        else:
            return "Large Object"

    def is_too_close(self, width, height):
        """
        Detect if the object is too close to the camera.
        """
        size_ratio = width * height
        if size_ratio > 120000: # Adjust this threshold based on experiments
            return True
        return False

    def draw_label(self, frame, text, x, y, color):
        """
        Draws a label with a filled background to prevent overlapping text.
        """
        (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x, y - 20), (x + text_width, y), (0, 0, 0), -1)
        cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def detect_objects_by_size(self, frame, color):
        """
        Detect objects using size and confidence-based filtering.
        """
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self.get_mask(img_hsv, color)

        # Confidence-based filtering to remove noise
        confidence = cv2.countNonZero(mask) / (self.width * self.height)
        if confidence < 0.005:
            return  # Ignore low-confidence detections

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)

            # Ignore small noise
            if area < 300:
                continue
            
            # Apply near-object filter
            if self.is_too_close(w, h):
                classification = "Small Cube"
            else:
                classification = self.classify_by_distance(area, w, h)

            label = f"{classification} ({color})"

            # Prevent duplicate detections using a set
            if label not in self.detected_objects:
                self.detected_objects.add(label)

                # Draw bounding box and label
                box_color = (0, 255, 0) if classification == "Small Cube" else (255, 0, 0)
                cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                self.draw_label(frame, f"Detected: {label}", x, y, box_color)

    def visualize_hsv(self, frame):
        """
        Visualizes the HSV representation of the frame to adjust color ranges if necessary.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imshow('HSV View', hsv)

    def run(self):
        while self.robot.step(self.time_step) != -1:
            image_data = self.camera.getImage()
            frame = np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            self.detected_objects.clear()  # Clear detected objects per frame

            # Detect objects by size for all colors
            for color in self.target_colors:
                self.detect_objects_by_size(frame, color)
                self.visualize_hsv(frame)

            cv2.imshow("Object Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cv2.destroyAllWindows()

def main():
    controller = BBAlr8OpenCVController(
        target_colors=["red", "cyan", "green", "yellow", "white"], stop_threshold=0.7
    )
    controller.run()

if __name__ == "__main__":
    main()
