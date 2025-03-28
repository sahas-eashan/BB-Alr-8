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

        # Refined color ranges with wider and more precise HSV ranges
        self.color_ranges = {
            "red": [
                ([0, 50, 50], [10, 255, 255]),    # Red range 1
                ([170, 50, 50], [180, 255, 255])  # Red range 2 (hue wrap-around)
            ],
            "blue": ([90, 100, 50], [130, 255, 255]),  # Blue range
            "green": ([40, 50, 50], [80, 255, 255]),   # Green range
            "yellow": ([20, 50, 50], [35, 255, 255]),  # Yellow range
            "white": ([0, 0, 200], [180, 50, 255]),    # White range
        }

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
                "area_min": 300,
                "area_max": 10000,
                "solidity_min": 0.7,
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

    def get_mask(self, img_hsv, color):
        """
        Generate mask using color ranges with support for multi-range colors
        """
        if color == "red":
            # Combine masks for both red ranges
            masks = [cv2.inRange(img_hsv, np.array(lower), np.array(upper)) 
                     for lower, upper in self.color_ranges["red"]]
            return cv2.bitwise_or(masks[0], masks[1])
        else:
            # Single range colors
            lower, upper = self.color_ranges.get(color, ([0, 0, 0], [0, 0, 0]))
            return cv2.inRange(img_hsv, np.array(lower), np.array(upper))

    def debug_color_detection(self, frame, color):
        """
        Diagnostic method to help understand color detection issues
        """
        # Convert to HSV
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get mask
        mask = self.get_mask(img_hsv, color)
        
        # Calculate confidence
        confidence = cv2.countNonZero(mask) / (self.width * self.height)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Print diagnostic information
        print(f"\nColor: {color}")
        print(f"Confidence: {confidence:.4f}")
        print(f"Number of contours: {len(contours)}")
        
        # Visualize mask
        cv2.imshow(f'{color} Mask', mask)
        
        # If no contours, print additional info
        if len(contours) == 0:
            print("No contours found. Possible reasons:")
            print("1. Color range might be too narrow")
            print("2. Lighting conditions may affect detection")
            print("3. No objects of this color in the frame")

    def draw_label(self, frame, text, x, y, color):
        """
        Draws a label with a filled background to prevent overlapping text.
        """
        (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x, y - 20), (x + text_width, y), (0, 0, 0), -1)
        cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def detect_objects_by_size(self, frame, color):
        """
        Enhanced object detection method with debugging
        """
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self.get_mask(img_hsv, color)

        # Confidence-based filtering
        confidence = cv2.countNonZero(mask) / (self.width * self.height)
        if confidence < 0.005:
            # Debug if confidence is low
            self.debug_color_detection(frame, color)
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