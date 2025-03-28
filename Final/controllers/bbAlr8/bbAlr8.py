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
        self.target_colors = target_colors or ["blue", "red", "green", "yellow", "white"]
        self.stop_threshold = stop_threshold
        self.MAX_SPEED = 7.0
        self.detected_objects = set()

        # Updated color ranges for accurate detection
        self.color_ranges = {
            "red": ([0, 100, 100], [10, 255, 255]),  # Red
            "red2": ([170, 100, 100], [180, 255, 255]),  # Red (hue wrap-around)
            "blue": ([100, 100, 100], [140, 255, 255]),  # Light blue
            "green": ([40, 100, 100], [80, 255, 255]),  # Green
            "yellow": ([20, 100, 100], [35, 255, 255]),  # Yellow
            "white": ([0, 0, 200], [180, 50, 255]),  # White
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

    def classify_object(self, contour):
        """
        Advanced object classification using multiple features
        """
        # Calculate contour area and bounding box
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        
        # Aspect ratio and solidity for shape analysis
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity = float(area) / hull_area if hull_area > 0 else 0
        aspect_ratio = float(w) / h if h > 0 else 0
        
        # Classification criteria
        classifications = {
            "Cube": {
                "area_min": 500,
                "area_max": 10000,
                "solidity_min": 0.8,
                "aspect_ratio_min": 0.7,
                "aspect_ratio_max": 1.3
            },
            "Large Object": {
                "area_min": 10001,
                "area_max": float('inf'),
                "solidity_min": 0.6,
                "aspect_ratio_min": 0.5,
                "aspect_ratio_max": 2.0
            }
        }
        
        # Determine classification
        for class_name, criteria in classifications.items():
            if (criteria["area_min"] <= area <= criteria["area_max"] and
                criteria["solidity_min"] <= solidity and
                criteria["aspect_ratio_min"] <= aspect_ratio <= criteria["aspect_ratio_max"]):
                return class_name
        
        return "Unknown Object"

    def draw_label(self, frame, text, x, y, color):
        """
        Draws a label with a filled background to prevent overlapping text.
        """
        (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x, y - 20), (x + text_width, y), (0, 0, 0), -1)
        cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def detect_objects_by_size(self, frame, color):
        """
        Enhanced object detection method
        """
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self.get_mask(img_hsv, color)

        # Confidence-based filtering
        confidence = cv2.countNonZero(mask) / (self.width * self.height)
        if confidence < 0.005:
            return

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Skip very small contours
            if cv2.contourArea(contour) < 300:
                continue
            
            # Classify the object
            classification = self.classify_object(contour)
            
            # Only process valid classifications
            if classification in ["Cube", "Large Object"]:
                x, y, w, h = cv2.boundingRect(contour)
                
                label = f"{classification} ({color})"
                
                # Prevent duplicate detections
                if label not in self.detected_objects:
                    self.detected_objects.add(label)

                    # Color-code based on classification
                    box_color = (0, 255, 0) if classification == "Cube" else (255, 0, 0)
                    
                    # Draw bounding box and label
                    cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                    self.draw_label(frame, f"Detected: {label}", x, y, box_color)

    def run(self):
        while self.robot.step(self.time_step) != -1:
            image_data = self.camera.getImage()
            frame = np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            self.detected_objects.clear()  # Clear detected objects per frame

            # Detect objects by size for all colors
            for color in self.target_colors:
                self.detect_objects_by_size(frame, color)

            cv2.imshow("Object Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cv2.destroyAllWindows()

def main():
    controller = BBAlr8OpenCVController(
        target_colors=["blue", "red", "green", "yellow", "white"], stop_threshold=0.7
    )
    controller.run()

if __name__ == "__main__":
    main()