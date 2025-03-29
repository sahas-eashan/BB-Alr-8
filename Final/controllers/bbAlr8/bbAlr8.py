import cv2
import numpy as np
import sys
from controller import Robot, Camera, Motor, Gyro

class BBAlr8OpenCVController:
    def __init__(self):
        """
        Initialize the robot controller with OpenCV color tracking.
        """
        # Initialize the robot
        self.robot = Robot()
        
        # Get the time step of the current world
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # Initialize motors
        self.left_motor = self.robot.getDevice('leftMotor')
        self.right_motor = self.robot.getDevice('rightMotor')
        
        # Initialize camera
        self.camera = self.robot.getDevice('fcam')
        self.camera.enable(self.time_step)
        
        # Initialize gyro for rotation measurement
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(self.time_step)
        
        # Get camera dimensions
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        
        # Set motor properties
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        
        # Default motor speeds
        self.MAX_SPEED = 7.0
        
        # Color ranges (in HSV)
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'blue': ([100, 100, 100], [140, 255, 255]),
            'green': ([40, 100, 100], [80, 255, 255]),
            'yellow': ([20, 100, 100], [40, 255, 255])  # Filled in yellow color range
        }
        
        # Timer variables
        self.timer = 0
        self.turn_angle = 0.0
        
        # ROI parameters (adjustable)
        self.roi_x_start = 0.1  # 10% from left
        self.roi_x_end = 0.9    # 90% from left
        self.roi_y_bottom = 0.8 # 80% from top
        
        # Box detection parameters
        self.min_box_area = 500  # Minimum area to consider as a valid box
        self.goal_area_threshold = 10000  # Area threshold for when box is at goal
        self.goal_aspect_ratio = 1.5  # Expected aspect ratio when box is at goal
        
        # Box tracking variables
        self.box_detected = False
        self.box_x = 0
        self.box_y = 0
        self.box_width = 0
        self.box_height = 0
    
    def detect_color_box(self, color):
        """
        Detect color box when color is given using OpenCV.
        Returns box coordinates
        """
        # Get image from camera
        image = self.camera.getImage()
        
        # Convert to numpy array
        img_array = np.frombuffer(image, np.uint8).reshape((self.height, self.width, 4))
        
        # Convert from BGRA to BGR
        img_bgr = img_array[:, :, :3]
        
        # Apply ROI if box is already detected
        if self.box_detected:
            img_bgr = self.reduce_ROI_to_remove_distractions(img_bgr)
        
        # Convert to HSV
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        
        # Get color range
        lower_bound, upper_bound = self.color_ranges[color]
        
        # Create mask
        mask = cv2.inRange(img_hsv, np.array(lower_bound), np.array(upper_bound))
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        self.box_detected = False
        max_area = 0
        box_coords = None
        
        # Find the largest contour that meets our criteria
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_box_area and area > max_area:
                x, y, w, h = cv2.boundingRect(contour)
                
                # Store box properties
                self.box_x = x
                self.box_y = y
                self.box_width = w
                self.box_height = h
                self.box_detected = True
                max_area = area
                box_coords = (x, y, w, h)
        
        return box_coords
    
    def reduce_ROI_to_remove_distractions(self, image):
        """
        After the object is locked, clip the sides and section of the bottom 
        so that coloured goals won't be in the image.
        """
        h, w = image.shape[:2]
        
        # Calculate ROI boundaries
        x_start = int(w * self.roi_x_start)
        x_end = int(w * self.roi_x_end)
        y_bottom = int(h * self.roi_y_bottom)
        
        # Create ROI
        roi = image[0:y_bottom, x_start:x_end]
        
        return roi
    
    def is_box_at_center(self, color):
        """
        Check whether the given coloured box is at the center of the FOV.
        Return true or false.
        """
        # First detect the box
        box_coords = self.detect_color_box(color)
        
        if not box_coords or not self.box_detected:
            return False
        
        # Calculate center of image
        center_x = self.width // 2
        
        # Calculate center of box
        box_center_x = self.box_x + (self.box_width // 2)
        
        # Check if box is close to center (within 10% of image width)
        tolerance = self.width * 0.1
        return abs(box_center_x - center_x) < tolerance
    
    def is_box_below_roi(self):
        """
        Check if the box has gone below the ROI (meaning it's captured).
        """
        if not self.box_detected:
            return False
        
        # Check if bottom of box is near bottom of image
        return self.box_y + self.box_height > self.height * self.roi_y_bottom
    
    def is_box_at_goal(self):
        """
        Check if the box is at the goal based on area and aspect ratio.
        """
        if not self.box_detected:
            return False
        
        box_area = self.box_width * self.box_height
        box_aspect = self.box_width / max(1, self.box_height)  # Avoid division by zero
        
        # Check if area exceeds threshold and aspect ratio is close to expected
        return (box_area > self.goal_area_threshold and 
                abs(box_aspect - self.goal_aspect_ratio) < 0.3)
    
    def move_forward(self):
        """Set both motors to move forward"""
        self.left_motor.setVelocity(self.MAX_SPEED)
        self.right_motor.setVelocity(self.MAX_SPEED)

    def move_backward(self):
        """Set both motors to move backward"""
        self.left_motor.setVelocity(-self.MAX_SPEED)
        self.right_motor.setVelocity(-self.MAX_SPEED)

    def start_turn(self):
        """Start turning in place"""
        self.left_motor.setVelocity(-self.MAX_SPEED)
        self.right_motor.setVelocity(self.MAX_SPEED)
    
    def turn_left(self):
        """Turn left by slowing right motor"""
        self.left_motor.setVelocity(0.3 * self.MAX_SPEED)
        self.right_motor.setVelocity(self.MAX_SPEED)
    
    def turn_right(self):
        """Turn right by slowing left motor"""
        self.left_motor.setVelocity(self.MAX_SPEED)
        self.right_motor.setVelocity(0.3 * self.MAX_SPEED)
    
    def stop(self):
        """Stop both motors"""
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
    
    def track_rotation(self):
        """Track rotation using gyro sensor"""
        gyro_values = self.gyro.getValues()
        angular_velocity = gyro_values[2]  # Z-axis rotation
        self.turn_angle += angular_velocity * (self.time_step / 1000.0)
        return self.turn_angle
    
    def reset_rotation_tracking(self):
        """Reset rotation tracking"""
        self.turn_angle = 0.0
    
    def run(self):
        """
        Main control loop for the robot.
        """
        # State variables
        state = "turn_left_initial"
        detect_color = False
        follow_color = 'blue'
        goal_color = 'blue'
        
        # Timer for forward movement
        forward_time = 3000  # 3 seconds in milliseconds
        self.timer = 0
        
        # Reset rotation tracking
        self.reset_rotation_tracking()
        
        while self.robot.step(self.time_step) != -1:
            
            # Update timer
            self.timer += self.time_step
            
            # State machine
            if state == "turn_left_initial":
                self.start_turn()
                # Track rotation angle
                angle = self.track_rotation()
                # Complete 90 degree left turn
                if abs(angle) >= 1.57:  # ~90 degrees in radians
                    self.stop()
                    self.reset_rotation_tracking()
                    state = "move_forward"
                    self.timer = 0
            
            elif state == "move_forward":
                self.move_forward()
                # Move forward for set time
                if self.timer >= forward_time:
                    self.stop()
                    state = "turn_right"
                    self.reset_rotation_tracking()
            
            elif state == "turn_right":
                self.start_turn()
                # Track rotation angle (negative for right turn)
                angle = self.track_rotation()
                # Complete 90 degree right turn
                if angle <= -1.57:  # ~-90 degrees in radians
                    self.stop()
                    self.reset_rotation_tracking()
                    state = "search_for_box"
                    detect_color = True
            
            elif state == "search_for_box":
                if detect_color:
                    self.detect_color_box(follow_color)
                    
                if not self.box_detected:
                    self.start_turn()
                else:
                    state = "center_box"
            
            elif state == "center_box":
                self.detect_color_box(follow_color)
                
                if not self.box_detected:
                    state = "search_for_box"
                    continue
                
                # Calculate center of image
                center_x = self.width // 2
                
                # Calculate center of box
                box_center_x = self.box_x + (self.box_width // 2)
                
                # Adjust direction based on box position
                if abs(box_center_x - center_x) < self.width * 0.05:
                    # Box is centered, move forward
                    self.move_forward()
                    state = "approach_box"
                elif box_center_x < center_x:
                    # Box is to the left, turn left
                    self.turn_left()
                else:
                    # Box is to the right, turn right
                    self.turn_right()
            
            elif state == "approach_box":
                self.detect_color_box(follow_color)
                
                if not self.box_detected:
                    state = "search_for_box"
                    continue
                
                # Check if box is below ROI (captured)
                if self.is_box_below_roi():
                    self.stop()
                    detect_color = False
                    state = "turn_to_goal"
                    self.reset_rotation_tracking()
                else:
                    # Continue moving forward
                    center_x = self.width // 2
                    box_center_x = self.box_x + (self.box_width // 2)
                    
                    # Make minor adjustments while approaching
                    if abs(box_center_x - center_x) < self.width * 0.05:
                        self.move_forward()
                    elif box_center_x < center_x:
                        self.turn_left()
                    else:
                        self.turn_right()
            
            elif state == "turn_to_goal":
                self.start_turn()
                # Track rotation angle
                angle = self.track_rotation()
                # Complete 90 degree left turn
                if abs(angle) >= 1.57:  # ~90 degrees in radians
                    self.stop()
                    self.reset_rotation_tracking()
                    state = "search_for_goal"
                    detect_color = True
            
            elif state == "search_for_goal":
                if detect_color:
                    self.detect_color_box(goal_color)
                    
                if not self.box_detected:
                    self.start_turn()
                else:
                    state = "approach_goal"
            
            elif state == "approach_goal":
                self.detect_color_box(goal_color)
                
                if not self.box_detected:
                    state = "search_for_goal"
                    continue
                
                # Check if box is at goal based on area and aspect ratio
                if self.is_box_at_goal():
                    self.stop()
                    state = "retreat"
                    self.timer = 0
                else:
                    # Continue moving toward goal
                    center_x = self.width // 2
                    box_center_x = self.box_x + (self.box_width // 2)
                    
                    # Make adjustments while approaching
                    if abs(box_center_x - center_x) < self.width * 0.05:
                        self.move_forward()
                    elif box_center_x < center_x:
                        self.turn_left()
                    else:
                        self.turn_right()
            
            elif state == "retreat":
                # Move backward for a short time
                self.move_backward()
                
                if self.timer >= 2000:  # 2 seconds
                    self.stop()
                    state = "finished"
            
            elif state == "finished":
                # Task complete
                self.stop()


def main():
    """
    Main function to create and run the robot controller.
    """
    controller = BBAlr8OpenCVController()
    controller.run()


if __name__ == "__main__":
    main()