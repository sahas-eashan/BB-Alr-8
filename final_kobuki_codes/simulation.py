import cv2
import numpy as np
import time
import threading
from kobukidriver import Kobuki


class KobukiColorController:
    def __init__(self):
        """Initialize the Kobuki robot controller with OpenCV color tracking."""
        # Initialize the robot
        self.robot = Kobuki()

        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        ret, test_frame = self.cap.read()
        if not ret:
            raise Exception("Failed to initialize camera")

        self.width = test_frame.shape[1]
        self.height = test_frame.shape[0]

        # Default motor speeds
        self.MAX_SPEED = 80.0

        # Color detection parameters (HSV ranges)
        self.blue_lower = np.array([100, 100, 100])
        self.blue_upper = np.array([140, 255, 255])

        # Box detection parameters
        self.min_box_area = 500
        self.goal_area_threshold = 15000  # Adjust for Kobuki's camera

        # Create visualization windows
        cv2.namedWindow("Robot View", cv2.WINDOW_NORMAL)

        # Task management
        self.current_task = "initial_navigation"
        self.has_box = False
        self.direction = "Stop"
        self.running = True
        self.last_detection_time = time.time()
        self.timeout = 2  # seconds to stop if no color detected

        # Rotation tracking (simulated since Kobuki might not have gyro)
        self.turn_angle = 0.0
        self.turn_start_time = 0
        self.timer = 0

    def get_camera_image(self):
        """Get and process the camera image."""
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def detect_blue_object(self, captured_mode=False):
        """
        Detect blue objects in the image with size constraints.
        Returns (detected, x, y, w, h, area, center_x)

        If captured_mode is True, we're looking for the goal, which is larger
        """
        # Get camera image
        img_bgr = self.get_camera_image()
        if img_bgr is None:
            return False, 0, 0, 0, 0, 0, 0

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
        min_area = self.min_box_area  # Default minimum area
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
                f"Task: {self.current_task}",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Area: {max_area_found:.0f}",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Aspect: {aspect_ratio:.2f}",
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # Show constraints
            cv2.putText(
                img_display,
                f"Min area: {min_area}",
                (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Max area: {max_area:.0f}",
                (10, 140),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # Display the image
            cv2.imshow("Robot View", img_display)
            cv2.waitKey(1)

            # Update last detection time
            self.last_detection_time = time.time()

            return True, x, y, w, h, max_area_found, center_x
        else:
            # No object detected
            cv2.putText(
                img_display,
                f"Task: {self.current_task}",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                "No blue object detected",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # Show constraints
            cv2.putText(
                img_display,
                f"Min area: {min_area}",
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                img_display,
                f"Max area: {max_area:.0f}",
                (10, 110),
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
            cv2.imshow("Blue Mask", mask)
            cv2.imshow("Robot View", img_display)
            cv2.waitKey(1)

            return False, 0, 0, 0, 0, 0, 0

    def is_box_captured(self, y, h):
        """Check if box is captured (bottom of box is near bottom of image)."""
        return y + h > self.height * 0.85

    def is_at_goal(self, area, aspect_ratio):
        """Check if the box is at the goal based on area and aspect ratio."""
        return area > self.goal_area_threshold

    def move_motors(self, left_speed, right_speed):
        """Set motor speeds for Kobuki with clamping to MAX_SPEED."""
        self.robot.move(
            max(-self.MAX_SPEED, min(self.MAX_SPEED, left_speed)),
            max(-self.MAX_SPEED, min(self.MAX_SPEED, right_speed)),
            0,  # No angular velocity setting needed for Kobuki
        )

    def simulate_rotation_tracking(self, direction, duration=None):
        """
        Simulate rotation tracking using time instead of gyro.
        direction: 1 for left turn, -1 for right turn
        duration: if provided, turn for this duration in ms
        """
        # Approximate turn rate (radians per second)
        TURN_RATE = 0.7  # Adjust based on actual robot behavior

        if duration is None:
            # Just update the angle based on direction
            current_time = time.time()
            elapsed_time = current_time - self.turn_start_time
            self.turn_start_time = current_time

            # Update the angle
            self.turn_angle += direction * TURN_RATE * elapsed_time
        else:
            # Turn for a specific duration
            current_time = time.time()
            if self.turn_start_time == 0:
                self.turn_start_time = current_time

            elapsed_time = (current_time - self.turn_start_time) * 1000  # Convert to ms

            if elapsed_time >= duration:
                # Turn completed
                self.turn_start_time = 0
                return True

        return False

    def reset_rotation_tracking(self):
        """Reset rotation tracking."""
        self.turn_angle = 0.0
        self.turn_start_time = 0

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
            self.direction = "Forward"
        else:
            # Use proportional control based on the error
            # As proximity increases, turning becomes more gentle
            turn_scale = 1.5 * (1.0 - 0.7 * proximity_factor)

            if normalized_error < 0:  # Object is to the left
                left_speed = base_speed * (1.0 - abs(normalized_error) * turn_scale)
                right_speed = base_speed
                self.direction = "Left"
            else:  # Object is to the right
                left_speed = base_speed
                right_speed = base_speed * (1.0 - normalized_error * turn_scale)
                self.direction = "Right"

            self.move_motors(left_speed, right_speed)

        return True

    def turn_for_duration(self, direction, duration):
        """
        Turn robot in a direction for a set duration.
        direction: "left" or "right"
        duration: time in milliseconds
        Returns True if turn is completed, False otherwise
        """
        if self.turn_start_time == 0:
            self.turn_start_time = time.time()

        elapsed_time = (time.time() - self.turn_start_time) * 1000  # Convert to ms

        if elapsed_time >= duration:
            # Turn completed
            self.move_motors(0, 0)
            self.turn_start_time = 0
            return True

        # Perform the turn
        if direction == "left":
            self.move_motors(-self.MAX_SPEED * 0.7, self.MAX_SPEED * 0.7)
            self.direction = "Left"
        else:  # right
            self.move_motors(self.MAX_SPEED * 0.7, -self.MAX_SPEED * 0.7)
            self.direction = "Right"

        return False

    def control_robot(self):
        """Separate thread for robot control."""
        while self.running:
            # This function is not needed as we're controlling directly in main loop
            time.sleep(0.1)

    def run(self):
        """Main control loop for the robot."""
        # Robot control thread (not necessary but kept for compatibility)
        robot_thread = threading.Thread(target=self.control_robot)
        robot_thread.daemon = True
        robot_thread.start()

        # Play startup sound
        self.robot.play_on_sound()

        # Initial turn durations (milliseconds)
        LEFT_TURN_DURATION = 1500  # Adjust based on actual robot turning rate
        RIGHT_TURN_DURATION = 1500
        FORWARD_DURATION = 2000

        # Initial turn complete flag
        self.has_completed_initial_turn = False
        self.has_moved_forward = False
        self.has_turned_right = False

        # Timer
        self.timer = 0
        self.start_time = time.time()

        try:
            while self.running:
                # Update timer
                current_time = time.time()
                self.timer = int(
                    (current_time - self.start_time) * 1000
                )  # Convert to ms

                # Simple task-based approach
                if self.current_task == "initial_navigation":
                    # Sub-task 1: Turn left
                    if not self.has_completed_initial_turn:
                        if self.turn_for_duration("left", LEFT_TURN_DURATION):
                            self.has_completed_initial_turn = True
                            self.start_time = time.time()  # Reset timer
                            print("Initial turn complete")

                    # Sub-task 2: Move forward for a set time
                    elif not self.has_moved_forward:
                        self.move_motors(self.MAX_SPEED, self.MAX_SPEED)
                        self.direction = "Forward"
                        if self.timer >= FORWARD_DURATION:
                            self.has_moved_forward = True
                            self.start_time = time.time()  # Reset timer
                            print("Forward movement complete")

                    # Sub-task 3: Turn right
                    elif not self.has_turned_right:
                        if self.turn_for_duration("right", RIGHT_TURN_DURATION):
                            self.has_turned_right = True
                            self.start_time = time.time()  # Reset timer
                            print("Right turn complete")
                            self.current_task = "find_box"

                elif self.current_task == "find_box":
                    # Search for the blue box by rotating
                    detected, x, y, w, h, area, center_x = self.detect_blue_object(
                        False
                    )

                    if not detected:
                        # Turn slowly to search
                        self.move_motors(-self.MAX_SPEED * 0.5, self.MAX_SPEED * 0.5)
                        self.direction = "Searching"

                        # Check timeout for object detection
                        if time.time() - self.last_detection_time > self.timeout:
                            # Alternate search direction periodically
                            if (
                                int(time.time()) % 6 < 3
                            ):  # Change direction every 3 seconds
                                self.move_motors(
                                    -self.MAX_SPEED * 0.5, self.MAX_SPEED * 0.5
                                )
                            else:
                                self.move_motors(
                                    self.MAX_SPEED * 0.5, -self.MAX_SPEED * 0.5
                                )
                    else:
                        self.current_task = "approach_box"
                        print("Box found, approaching")

                elif self.current_task == "approach_box":
                    # Detect and follow the blue box
                    detected, x, y, w, h, area, center_x = self.detect_blue_object(
                        False
                    )

                    if not detected:
                        # Lost the box, go back to search
                        if time.time() - self.last_detection_time > self.timeout:
                            self.move_motors(0, 0)
                            self.direction = "Stop"
                            print("Box lost, searching again")
                            self.current_task = "find_box"
                    elif self.is_box_captured(y, h):
                        # Box is captured, stop and prepare to find goal
                        self.move_motors(0, 0)
                        self.direction = "Stop"
                        self.has_box = True
                        self.start_time = time.time()  # Reset timer
                        self.current_task = "turn_to_goal"
                        print("Box captured, turning to find goal")
                    else:
                        # Follow the box using adaptive control
                        self.follow_object(detected, center_x, area)

                elif self.current_task == "turn_to_goal":
                    # Turn left to face the goal area
                    if self.turn_for_duration("left", LEFT_TURN_DURATION):
                        self.start_time = time.time()  # Reset timer
                        self.current_task = "find_goal"
                        print("Turned to face goal area")

                elif self.current_task == "find_goal":
                    # Search for the blue goal
                    detected, x, y, w, h, area, center_x = self.detect_blue_object(True)

                    if not detected:
                        # Turn slowly to search
                        self.move_motors(-self.MAX_SPEED * 0.5, self.MAX_SPEED * 0.5)
                        self.direction = "Searching Goal"

                        # Check timeout for object detection
                        if time.time() - self.last_detection_time > self.timeout:
                            # Alternate search direction periodically
                            if (
                                int(time.time()) % 6 < 3
                            ):  # Change direction every 3 seconds
                                self.move_motors(
                                    -self.MAX_SPEED * 0.5, self.MAX_SPEED * 0.5
                                )
                            else:
                                self.move_motors(
                                    self.MAX_SPEED * 0.5, -self.MAX_SPEED * 0.5
                                )
                    else:
                        self.current_task = "approach_goal"
                        print("Goal found, approaching")

                elif self.current_task == "approach_goal":
                    # Detect and approach the blue goal
                    detected, x, y, w, h, area, center_x = self.detect_blue_object(True)

                    if not detected:
                        # Lost the goal, go back to search
                        if time.time() - self.last_detection_time > self.timeout:
                            self.move_motors(0, 0)
                            self.direction = "Stop"
                            print("Goal lost, searching again")
                            self.current_task = "find_goal"
                    else:
                        # Check if we've reached the goal based on area
                        aspect_ratio = w / max(1, h)
                        if self.is_at_goal(area, aspect_ratio):
                            self.move_motors(0, 0)
                            self.direction = "Stop"
                            self.current_task = "retreat"
                            self.start_time = time.time()  # Reset timer
                            print("Goal reached, retreating")
                        else:
                            # Approach the goal while keeping it centered
                            self.follow_object(detected, center_x, area)

                elif self.current_task == "retreat":
                    # Back up for 2 seconds
                    self.move_motors(-self.MAX_SPEED * 0.7, -self.MAX_SPEED * 0.7)
                    self.direction = "Reversing"
                    if self.timer >= 2000:  # 2 seconds
                        self.move_motors(0, 0)
                        self.direction = "Stop"
                        self.current_task = "finished"
                        print("Task completed successfully")
                        self.robot.play_off_sound()

                elif self.current_task == "finished":
                    # Keep robot stopped
                    self.move_motors(0, 0)
                    self.direction = "Stop"

                # Check for exit command
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        except KeyboardInterrupt:
            print("Program interrupted by user")
        finally:
            self.running = False
            self.robot.move(0, 0, 0)
            self.robot.play_off_sound()
            self.cap.release()
            cv2.destroyAllWindows()
            robot_thread.join(timeout=1.0)

    def cleanup(self):
        """Close all OpenCV windows and stop robot when done."""
        self.running = False
        self.robot.move(0, 0, 0)
        self.robot.play_off_sound()
        self.cap.release()
        cv2.destroyAllWindows()


def main():
    """Main function to create and run the robot controller."""
    try:
        print("Starting Kobuki robot with blue box tracking...")
        controller = KobukiColorController()
        controller.run()
    except KeyboardInterrupt:
        print("Program stopped by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        try:
            controller.cleanup()
        except:
            pass


if __name__ == "__main__":
    main()
