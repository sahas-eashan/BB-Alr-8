import cv2
import numpy as np
import threading
import time
from kobuki import Kobuki


class KobukiColorController:
    def __init__(self):
        """Initialize the Kobuki robot controller with OpenCV color tracking for multiple colors."""
        # Initialize Kobuki robot
        self.robot = Kobuki()
        self.time_step = 100  # milliseconds

        # Initialize camera
        self.camera = cv2.VideoCapture(0)  # Use default camera
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ret, frame = self.camera.read()
        if ret:
            self.height, self.width = frame.shape[:2]
        else:
            self.width = 640
            self.height = 480
            print("Warning: Could not initialize camera properly!")

        # Default motor speeds
        self.MAX_SPEED = 70  # Adjusted for Kobuki velocity scale

        # Color detection parameters (HSV ranges for different colors)
        self.color_ranges = {
            "blue": {
                "lower": np.array([90, 100, 50]),
                "upper": np.array([130, 255, 255]),
            },
            "red1": {
                "lower": np.array([0, 100, 50]),
                "upper": np.array([10, 255, 255]),
            },  # Lower red range
            "red2": {
                "lower": np.array([160, 100, 50]),
                "upper": np.array([180, 255, 255]),
            },  # Upper red range
            "green": {
                "lower": np.array([40, 100, 50]),
                "upper": np.array([80, 255, 255]),
            },
            "yellow": {
                "lower": np.array([20, 100, 50]),
                "upper": np.array([40, 255, 255]),
            },
        }

        # Current color to detect (for both box and goal)
        self.current_color = "blue"
        self.target_mode = "box"  # Either "box" or "goal"

        # Box detection parameters
        self.min_box_area = 300
        self.goal_area_threshold = 220000
        self.goal_aspect_ratio = 1.5

        # Create visualization windows
        cv2.namedWindow("Robot View", cv2.WINDOW_NORMAL)

        # Task management
        self.main_phase = (
            "blue_phase"  # Starts with blue phase, then red, green, yellow
        )
        self.current_task = "initial_navigation"
        self.has_box = False
        self.turn_angle = 0.0
        self.timer = 0

        # Previous gyro readings for tracking rotation
        self.prev_gyro_time = time.time()
        self.accumulated_angle = 0.0

        # Start the main control loop in a separate thread
        self.running = True
        self.control_thread = threading.Thread(target=self.run)
        self.control_thread.daemon = True
        self.control_thread.start()

    def get_camera_image(self):
        """Get and process the camera image."""
        ret, frame = self.camera.read()
        if not ret:
            print("Failed to capture image!")
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        return frame

    def detect_colored_object(self, goal_mode=False):
        """
        Detect colored objects in the image with size constraints.

        Args:
            goal_mode (bool): If True, looking for goals which are larger than boxes

        Returns:
            (detected, x, y, w, h, area, center_x)
        """
        # Get camera image
        img_bgr = self.get_camera_image()
        img_display = img_bgr.copy()

        # Convert to HSV colorspace
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        # Create mask for the current color
        if self.current_color == "red":
            # For red, we need to combine two ranges due to how hue wraps around
            mask1 = cv2.inRange(
                img_hsv,
                self.color_ranges["red1"]["lower"],
                self.color_ranges["red1"]["upper"],
            )
            mask2 = cv2.inRange(
                img_hsv,
                self.color_ranges["red2"]["lower"],
                self.color_ranges["red2"]["upper"],
            )
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            # For other colors, use the corresponding range
            mask = cv2.inRange(
                img_hsv,
                self.color_ranges[self.current_color]["lower"],
                self.color_ranges[self.current_color]["upper"],
            )

        # Add morphology operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Area constraints
        min_area = self.min_box_area  # Default minimum area
        max_area = self.width * self.height * 0.2  # Max 20% of image

        if goal_mode:
            # When looking for the goal, adjust size constraints
            min_area = self.min_box_area * 3
            max_area = self.width * self.height * 0.8  # Allow larger area for goal

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

            # Set display color based on current color
            color_display = (0, 0, 255)  # Default red rectangle
            if self.current_color == "blue":
                color_display = (255, 0, 0)
            elif self.current_color == "green":
                color_display = (0, 255, 0)
            elif self.current_color == "yellow":
                color_display = (0, 255, 255)

            cv2.rectangle(img_display, (x, y), (x + w, y + h), color_display, 2)
            cv2.line(img_display, (center_x, y), (center_x, y + h), (255, 0, 0), 2)

            # Draw center of image
            img_center_x = self.width // 2
            cv2.line(
                img_display,
                (img_center_x, 0),
                (img_center_x, self.height),
                (0, 255, 255),
                1,
            )

            # Add information text
            aspect_ratio = w / max(1, h)
            cv2.putText(
                img_display,
                f"Phase: {self.main_phase}",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Task: {self.current_task}",
                (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Target: {self.target_mode} ({self.current_color})",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Area: {max_area_found:.0f}",
                (10, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Aspect: {aspect_ratio:.2f}",
                (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # Show constraints
            cv2.putText(
                img_display,
                f"Min area: {min_area}",
                (10, 145),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Max area: {max_area:.0f}",
                (10, 170),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # Display the image
            cv2.imshow("Robot View", img_display)
            cv2.waitKey(1)

            return True, x, y, w, h, max_area_found, center_x
        else:
            # No object detected
            cv2.putText(
                img_display,
                f"Phase: {self.main_phase}",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Task: {self.current_task}",
                (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Target: {self.target_mode} ({self.current_color})",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"No object detected",
                (10, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # Show constraints
            cv2.putText(
                img_display,
                f"Min area: {min_area}",
                (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Max area: {max_area:.0f}",
                (10, 145),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # Draw center of image
            img_center_x = self.width // 2
            cv2.line(
                img_display,
                (img_center_x, 0),
                (img_center_x, self.height),
                (0, 255, 255),
                1,
            )

            # Also display the mask to debug color detection
            cv2.imshow("Color Mask", mask)
            cv2.imshow("Robot View", img_display)
            cv2.waitKey(1)

            return False, 0, 0, 0, 0, 0, 0

    def is_box_captured(self, y, h):
        """Check if box is captured (bottom of box is near bottom of image)."""
        return y > self.height * 0.88

    def is_at_goal(self, area, aspect_ratio):
        """Check if the box is at the goal based on area and aspect ratio."""
        return area > self.goal_area_threshold

    def move_motors(self, left_speed, right_speed):
        """Set motor speeds with clamping to MAX_SPEED."""
        # Kobuki move function takes left_velocity, right_velocity, rotate
        # where rotate=0 means normal movement, rotate=1 means rotation movement
        # Scale speeds to Kobuki's range
        left_velocity = max(-self.MAX_SPEED, min(self.MAX_SPEED, left_speed))
        right_velocity = max(-self.MAX_SPEED, min(self.MAX_SPEED, right_speed))

        # Determine if we're rotating or moving
        # If speeds have opposite signs, we're rotating
        if left_velocity * right_velocity < 0:
            self.robot.move(left_velocity, right_velocity, 1)  # Rotation mode
        else:
            self.robot.move(left_velocity, right_velocity, 0)  # Normal movement

    def track_rotation(self):
        """
        Track rotation using gyro data from Kobuki.

        This simulates the Webots gyro sensor using Kobuki's gyro data.
        """
        try:
            gyro_data = self.robot.gyro_velocity_data()
            # Get Z-axis angular velocity (assuming this maps to rotation around vertical axis)
            z_velocity = gyro_data.get("angular velocity of z", [0])[0]

            # Calculate time delta
            current_time = time.time()
            dt = current_time - self.prev_gyro_time
            self.prev_gyro_time = current_time

            # Accumulate angle (integrate angular velocity)
            self.accumulated_angle += z_velocity * dt

            return self.accumulated_angle
        except Exception as e:
            print(f"Error in track_rotation: {e}")
            return self.accumulated_angle

    def reset_rotation_tracking(self):
        """Reset rotation tracking."""
        self.accumulated_angle = 0.0
        self.prev_gyro_time = time.time()

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

    def transition_to_next_color_phase(self):
        """Transition to the next color phase in the sequence."""
        if self.main_phase == "blue_phase":
            self.main_phase = "red_phase"
            self.current_color = "red"
            print("Transitioning to RED phase")
            self.robot.play_button_sound()
        elif self.main_phase == "red_phase":
            self.main_phase = "green_phase"
            self.current_color = "green"
            print("Transitioning to GREEN phase")
            self.robot.play_button_sound()
        elif self.main_phase == "green_phase":
            self.main_phase = "yellow_phase"
            self.current_color = "yellow"
            print("Transitioning to YELLOW phase")
            self.robot.play_button_sound()
        elif self.main_phase == "yellow_phase":
            self.main_phase = "completed"
            print("All phases completed!")
            self.robot.play_clean_stop_sound()

        # Reset task state for the new phase
        self.current_task = "turn_to_find_box"
        self.target_mode = "box"  # Start by finding a box
        self.has_box = False
        self.reset_rotation_tracking()
        self.timer = 0

    def run(self):
        """Main control loop for the robot."""
        # Define angles for turns (radians)
        TURN_LEFT_ANGLE = 1.57  # 90 degrees counterclockwise
        TURN_RIGHT_ANGLE = -1.57  # 90 degrees clockwise
        TURN_180_ANGLE = -3.14 * 0.4  # 180 degrees (full turn)

        # Initial turn complete flag for blue phase
        self.has_completed_initial_turn = False
        self.has_moved_forward = False
        self.has_turned_right = False

        # Drive forward timer
        forward_time = 1600  # milliseconds
        self.timer = 0

        # Reset rotation tracking at start
        self.reset_rotation_tracking()

        # Play startup sound
        self.robot.play_on_sound()

        while self.running:
            # Process time step
            time.sleep(self.time_step / 1000.0)  # Convert ms to seconds

            # Update timer
            self.timer += self.time_step

            # Blue Phase: Initial Navigation
            if (
                self.main_phase == "blue_phase"
                and self.current_task == "initial_navigation"
            ):
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
                        self.target_mode = "box"

            # All Phases: Find Box of Current Color
            elif self.current_task == "find_box":
                # Make sure we're looking for a box
                self.target_mode = "box"

                # Search for the box of current color by rotating
                detected, x, y, w, h, area, center_x = self.detect_colored_object(False)

                if not detected:
                    # Turn slowly to search
                    self.move_motors(-self.MAX_SPEED * 0.5, self.MAX_SPEED * 0.5)
                else:
                    self.current_task = "approach_box"
                    print(f"Found {self.current_color} box, approaching")
                    self.robot.play_button_sound()

            # All Phases: Approach Box
            elif self.current_task == "approach_box":
                # Make sure we're still looking for a box
                self.target_mode = "box"

                # Detect and follow the box
                detected, x, y, w, h, area, center_x = self.detect_colored_object(False)

                if not detected:
                    # Lost the box, go back to search
                    self.current_task = "find_box"
                elif self.is_box_captured(y, h):
                    # Box is captured, stop and prepare to find goal
                    self.move_motors(0, 0)
                    self.has_box = True
                    self.reset_rotation_tracking()
                    self.current_task = "turn_to_goal"
                    print(f"{self.current_color} box captured, turning to find goal")
                    self.robot.set_led1_green_colour()
                else:
                    # Follow the box using adaptive control
                    self.follow_object(detected, center_x, area)

            # All Phases: Turn to Find Goal
            elif self.current_task == "turn_to_goal":
                # Turn left 90 degrees to face the goal area
                if self.turn_to_angle(TURN_LEFT_ANGLE):
                    self.reset_rotation_tracking()
                    self.current_task = "find_goal"
                    self.target_mode = "goal"  # Now looking for a goal
                    print(f"Turned to face {self.current_color} goal area")

            # All Phases: Find Goal - Now uses the same color as the box
            elif self.current_task == "find_goal":
                # We're looking for a goal - keep same color as the box
                self.target_mode = "goal"

                # Search for the matching colored goal
                detected, x, y, w, h, area, center_x = self.detect_colored_object(True)

                if not detected:
                    # Turn slowly to search
                    self.move_motors(-self.MAX_SPEED * 0.5, self.MAX_SPEED * 0.5)
                else:
                    self.current_task = "approach_goal"
                    print(f"{self.current_color} goal found, approaching")

            # All Phases: Approach Goal - Uses same color as box
            elif self.current_task == "approach_goal":
                # Still looking for a goal
                self.target_mode = "goal"

                # Detect and approach the matching colored goal
                detected, x, y, w, h, area, center_x = self.detect_colored_object(True)

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
                        print(f"Goal reached with {self.current_color} box, retreating")
                        self.robot.set_led2_green_colour()
                    else:
                        # Approach the goal while keeping it centered
                        self.follow_object(detected, center_x, area)

            # All Phases: Retreat
            elif self.current_task == "retreat":
                # Back up for 2 seconds
                self.move_motors(-self.MAX_SPEED * 0.7, -self.MAX_SPEED * 0.7)

                retreatTime = 4000

                if self.current_color == "red":
                    retreatTime = 10000

                if self.timer >= retreatTime:  # 2-10 seconds depending on color
                    self.move_motors(0, 0)
                    self.robot.clr_led1()
                    self.robot.clr_led2()

                    # Check if we're done with all phases
                    if self.main_phase == "yellow_phase":
                        self.main_phase = "completed"
                        self.current_task = "finished"
                        print("All tasks completed successfully!")
                        self.robot.play_clean_stop_sound()
                    else:
                        # Prepare for next color phase
                        self.transition_to_next_color_phase()

            # For turning to correct position to find next box
            elif self.current_task == "turn_to_find_box":
                target_angle = 0

                # Different turning angles based on the current phase
                if self.main_phase == "red_phase":
                    # After blue phase, turn right to find red box
                    target_angle = TURN_RIGHT_ANGLE
                elif self.main_phase == "green_phase":
                    # After red phase, turn 180 degrees to find green box
                    target_angle = TURN_180_ANGLE
                elif self.main_phase == "yellow_phase":
                    # After green phase, turn left to find yellow box
                    target_angle = TURN_LEFT_ANGLE

                if self.turn_to_angle(target_angle):
                    self.reset_rotation_tracking()
                    self.current_task = "find_box"
                    print(f"Turned to search for {self.current_color} box")

            # Final completion
            elif self.current_task == "finished":
                # Keep robot stopped
                self.move_motors(0, 0)
                time.sleep(0.5)  # Sleep to prevent CPU hogging

    def cleanup(self):
        """Clean up resources and stop the robot."""
        self.running = False
        if self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)

        # Stop the robot
        try:
            self.move_motors(0, 0)
            self.robot.play_off_sound()
        except:
            pass

        # Release camera
        if self.camera.isOpened():
            self.camera.release()

        # Close OpenCV windows
        cv2.destroyAllWindows()


def main():
    """Main function to create and run the robot controller."""
    controller = None
    try:
        print("Initializing Kobuki Color Controller...")
        controller = KobukiColorController()

        # Keep the main thread alive
        print("Controller running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping controller...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if controller:
            controller.cleanup()
        print("Controller stopped.")


if __name__ == "__main__":
    main()
