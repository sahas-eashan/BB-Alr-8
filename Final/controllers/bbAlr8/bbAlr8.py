import cv2
import numpy as np
from controller import Robot, Camera, Motor, Gyro

class SimplifiedRobotController:
    def __init__(self):
        """Initialize the robot controller with OpenCV color tracking."""
        # Initialize the robot
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # Initialize motors
        self.left_motor = self.robot.getDevice('leftMotor')
        self.right_motor = self.robot.getDevice('rightMotor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        
        # Initialize camera
        self.camera = self.robot.getDevice('fcam')
        self.camera.enable(self.time_step)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        
        # Initialize gyro for rotation measurement
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(self.time_step)
        
        # Default motor speeds
        self.MAX_SPEED = 7.0
        
        # Color detection parameters (HSV ranges)
        self.blue_lower = np.array([90, 100, 50])
        self.blue_upper = np.array([130, 255, 255])
        
        # Box detection parameters
        self.min_box_area = 300
        self.goal_area_threshold = 223000
        self.goal_aspect_ratio = 1.5
        
        # Create visualization windows
        cv2.namedWindow('Robot View', cv2.WINDOW_NORMAL)
        
        # Task management
        self.current_task = "initial_navigation"
        self.has_box = False
        self.turn_angle = 0.0
        self.timer = 0
        
    def get_camera_image(self):
        """Get and process the camera image."""
        image = self.camera.getImage()
        img_array = np.frombuffer(image, np.uint8).reshape((self.height, self.width, 4))
        return img_array[:, :, :3]  # Return BGR image
    
    def detect_blue_object(self, captured_mode=False):
        """
        Detect blue objects in the image with size constraints.
        Returns (detected, x, y, w, h, area, center_x)
        
        If captured_mode is True, we're looking for the goal, which is larger
        """
        # Get camera image
        img_bgr = self.get_camera_image()
        img_display = img_bgr.copy()
        
        # Convert to HSV colorspace
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        
        # Create mask for blue color
        mask = cv2.inRange(img_hsv, self.blue_lower, self.blue_upper)
        
        # Add morphology operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Area constraints
        min_area = self.min_box_area  # Default minimum area (e.g., 500 pixels)
        max_area = self.width * self.height * 0.2  # Max 20% of image
        
        if captured_mode:
            # When looking for the goal, adjust size constraints
            min_area = self.min_box_area * 3
            max_area = self.width * self.height * 0.7  # Allow larger area for goal
        
        # Find the largest contour that meets our criteria
        max_area_found = 0
        best_contour = None
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Apply area constraints - ignore if too small or too large
            if min_area <= area <= max_area:
                if area > max_area_found:
                    max_area_found = area
                    best_contour = contour
        
        # Process the best contour if found
        if best_contour is not None:
            x, y, w, h = cv2.boundingRect(best_contour)
            center_x = x + (w // 2)
            
            # Draw rectangle around detected object
            cv2.rectangle(img_display, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.line(img_display, (center_x, y), (center_x, y + h), (255, 0, 0), 2)
            
            # Draw center of image
            img_center_x = self.width // 2
            cv2.line(img_display, (img_center_x, 0), (img_center_x, self.height), (0, 255, 255), 1)
            
            # Add information text
            aspect_ratio = w / max(1, h)
            cv2.putText(img_display, f"Task: {self.current_task}", (10, 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(img_display, f"Area: {max_area_found:.0f}", (10, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(img_display, f"Aspect: {aspect_ratio:.2f}", (10, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show constraints
            cv2.putText(img_display, f"Min area: {min_area}", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(img_display, f"Max area: {max_area:.0f}", (10, 140),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(img_display, f"Y : {y:.0f}", (10, 170),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(img_display, f"Height*0.9 : {self.height*0.9:.0f}", (10, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Display the image
            cv2.imshow('Robot View', img_display)
            cv2.waitKey(1)
            
            return True, x, y, w, h, max_area_found, center_x
        else:
            # No object detected
            cv2.putText(img_display, f"Task: {self.current_task}", (10, 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(img_display, "No blue object detected", (10, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show constraints
            cv2.putText(img_display, f"Min area: {min_area}", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(img_display, f"Max area: {max_area:.0f}", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Draw center of image
            img_center_x = self.width // 2
            cv2.line(img_display, (img_center_x, 0), (img_center_x, self.height), (0, 255, 255), 1)
            
            # Also display the mask to debug color detection
            cv2.imshow('Blue Mask', mask)
            cv2.imshow('Robot View', img_display)
            cv2.waitKey(1)
            
            return False, 0, 0, 0, 0, 0, 0
    
    def is_box_captured(self, y, h):
        """Check if box is captured (bottom of box is near bottom of image)."""
        return y > self.height * 0.89
    
    def is_at_goal(self, area, aspect_ratio):
        """Check if the box is at the goal based on area and aspect ratio."""
        return (area > self.goal_area_threshold )
    
    def move_motors(self, left_speed, right_speed):
        """Set motor speeds with clamping to MAX_SPEED."""
        self.left_motor.setVelocity(max(-self.MAX_SPEED, min(self.MAX_SPEED, left_speed)))
        self.right_motor.setVelocity(max(-self.MAX_SPEED, min(self.MAX_SPEED, right_speed)))
    
    def track_rotation(self):
        """Track rotation using gyro sensor."""
        gyro_values = self.gyro.getValues()
        angular_velocity = gyro_values[2]  # Z-axis rotation
        self.turn_angle += angular_velocity * (self.time_step / 1000.0)
        return self.turn_angle
    
    def reset_rotation_tracking(self):
        """Reset rotation tracking."""
        self.turn_angle = 0.0
    
    def follow_object(self, detected, center_x, area):
        """
        Follow a detected object with adaptive control based on position and proximity.
        Returns True if object is being followed, False if not detected.
        """
        if not detected:
            return False
        
        # Calculate center error
        img_center_x = self.width // 2
        error = center_x - img_center_x
        
        # Normalized error (-1.0 to 1.0)
        normalized_error = error / (self.width / 2)
        
        # Determine proximity factor (larger = closer = gentler movements)
        max_expected_area = (self.width * self.height) / 4
        proximity_factor = min(1.0, area / max_expected_area)
        
        # Base speed decreases as we get closer
        base_speed = self.MAX_SPEED * (0.7 - 0.3 * proximity_factor)
        
        # Very small error = go straight
        if abs(normalized_error) < 0.05:
            self.move_motors(base_speed, base_speed)
        else:
            # Use proportional control based on the error
            # As proximity increases, turning becomes more gentle
            turn_scale = 1.5 * (1.0 - 0.7 * proximity_factor)
            
            if normalized_error < 0:  # Object is to the left
                left_speed = base_speed * (1.0 - abs(normalized_error) * turn_scale)
                right_speed = base_speed
            else:  # Object is to the right
                left_speed = base_speed
                right_speed = base_speed * (1.0 - normalized_error * turn_scale)
            
            self.move_motors(left_speed, right_speed)
        
        return True
    
    def turn_to_angle(self, target_angle):
        """
        Turn robot to a specific angle.
        Returns True if target is reached, False otherwise.
        """
        current_angle = self.track_rotation()
        angle_error = target_angle - current_angle
        
        if abs(angle_error) < 0.05:  # Allow small error
            self.move_motors(0, 0)
            return True
        
        # Determine turn direction and speed
        if angle_error > 0:
            # Turn left (counterclockwise)
            self.move_motors(-self.MAX_SPEED * 0.7, self.MAX_SPEED * 0.7)
        else:
            # Turn right (clockwise)
            self.move_motors(self.MAX_SPEED * 0.7, -self.MAX_SPEED * 0.7)
        
        return False
    
    def run(self):
        """Main control loop for the robot."""
        # Target angles for initial navigation (radians)
        TURN_LEFT_ANGLE = 1.57   # 90 degrees
        TURN_RIGHT_ANGLE = -1.57  # -90 degrees
        
        # Initial turn complete flag
        self.has_completed_initial_turn = False
        self.has_moved_forward = False
        self.has_turned_right = False
        self.initial_search_started = False
        
        # Drive forward timer
        forward_time = 1500  # milliseconds
        self.timer = 0
        
        # Reset rotation tracking at start
        self.reset_rotation_tracking()
        
        while self.robot.step(self.time_step) != -1:
            # Update timer
            self.timer += self.time_step
            
            # Simple task-based approach instead of complex state machine
            if self.current_task == "initial_navigation":
                # Sub-task 1: Turn left 90 degrees
                if not self.has_completed_initial_turn:
                    if self.turn_to_angle(TURN_LEFT_ANGLE):
                        self.has_completed_initial_turn = True
                        self.timer = 0
                        print("Initial turn complete")
                
                # Sub-task 2: Move forward for a set time
                elif not self.has_moved_forward:
                    self.move_motors(self.MAX_SPEED, self.MAX_SPEED)
                    if self.timer >= forward_time:
                        self.has_moved_forward = True
                        self.reset_rotation_tracking()
                        print("Forward movement complete")
                
                # Sub-task 3: Turn right 90 degrees
                elif not self.has_turned_right:
                    if self.turn_to_angle(TURN_RIGHT_ANGLE):
                        self.has_turned_right = True
                        self.reset_rotation_tracking()
                        print("Right turn complete")
                        self.current_task = "find_box"
            
            elif self.current_task == "find_box":
                # Search for the blue box by rotating
                detected, x, y, w, h, area, center_x = self.detect_blue_object(False)
                
                if not detected:
                    # Turn slowly to search
                    self.move_motors(-self.MAX_SPEED * 0.5, self.MAX_SPEED * 0.5)
                else:
                    self.current_task = "approach_box"
            
            elif self.current_task == "approach_box":
                # Detect and follow the blue box
                detected, x, y, w, h, area, center_x = self.detect_blue_object(False)
                
                if not detected:
                    # Lost the box, go back to search
                    self.current_task = "find_box"
                elif self.is_box_captured(y, h):
                    # Box is captured, stop and prepare to find goal
                    self.move_motors(0, 0)
                    self.has_box = True
                    self.reset_rotation_tracking()
                    self.current_task = "turn_to_goal"
                    print("Box captured, turning to find goal")
                else:
                    # Follow the box using adaptive control
                    self.follow_object(detected, center_x, area)
            
            elif self.current_task == "turn_to_goal":
                # Turn left 90 degrees to face the goal area
                if self.turn_to_angle(TURN_LEFT_ANGLE):
                    self.reset_rotation_tracking()
                    self.current_task = "find_goal"
                    print("Turned to face goal area")
            
            elif self.current_task == "find_goal":
                # Search for the blue goal
                detected, x, y, w, h, area, center_x = self.detect_blue_object(True)
                
                if not detected:
                    # Turn slowly to search
                    self.move_motors(-self.MAX_SPEED * 0.5, self.MAX_SPEED * 0.5)
                else:
                    self.current_task = "approach_goal"
                    print("Goal found, approaching")
            
            elif self.current_task == "approach_goal":
                # Detect and approach the blue goal
                detected, x, y, w, h, area, center_x = self.detect_blue_object(True)
                
                if not detected:
                    # Lost the goal, go back to search
                    self.current_task = "find_goal"
                else:
                    # Check if we've reached the goal based on area and aspect ratio
                    aspect_ratio = w / max(1, h)
                    if self.is_at_goal(area, aspect_ratio):
                        self.move_motors(0, 0)
                        self.current_task = "retreat"
                        self.timer = 0
                        print("Goal reached, retreating")
                    else:
                        # Approach the goal while keeping it centered
                        self.follow_object(detected, center_x, area)
            
            elif self.current_task == "retreat":
                # Back up for 2 seconds
                self.move_motors(-self.MAX_SPEED * 0.7, -self.MAX_SPEED * 0.7)
                if self.timer >= 2000:  # 2 seconds
                    self.move_motors(0, 0)
                    self.current_task = "finished"
                    print("Task completed successfully")
            
            elif self.current_task == "finished":
                # Keep robot stopped
                self.move_motors(0, 0)
    
    def cleanup(self):
        """Close all OpenCV windows when done."""
        cv2.destroyAllWindows()


def main():
    """Main function to create and run the robot controller."""
    try:
        controller = SimplifiedRobotController()
        controller.run()
    except KeyboardInterrupt:
        controller.cleanup()
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()