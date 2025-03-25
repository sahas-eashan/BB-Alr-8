from controller import Robot, Camera, Motor, PositionSensor
import numpy as np
import math

class BBAlr8Controller:
    def __init__(self):
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
        
        # Set motor properties
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # Initialize encoders (optional, but good for tracking)
        self.left_encoder = self.robot.getDevice('leftEncoder')
        self.right_encoder = self.robot.getDevice('rightEncoder')
        self.left_encoder.enable(self.time_step)
        self.right_encoder.enable(self.time_step)
        
        # Default motor speeds
        self.MAX_SPEED = 7.0
    
    def detect_colored_object(self, color_threshold=100):
        """
        Detect a colored object in the camera view.
        Returns the x-coordinate of the object's center if found.
        
        :param color_threshold: Sensitivity for color detection
        :return: x-coordinate of object (-1 if no object found)
        """
        image = self.camera.getImage()
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        
        # Initialize variables to track object detection
        object_detected = False
        object_center_x = -1
        
        # Scan the image row by row
        for y in range(height):
            for x in range(width):
                # Get pixel color
                r = self.camera.imageGetRed(image, width, x, y)
                g = self.camera.imageGetGreen(image, width, x, y)
                b = self.camera.imageGetBlue(image, width, x, y)
                
                # Simple color detection (adjust thresholds as needed)
                if r > color_threshold and g < 50 and b < 50:
                    object_detected = True
                    object_center_x = x
                    break
            
            if object_detected:
                break
        
        return object_center_x
    
    def adjust_movement(self, object_x):
        """
        Adjust robot movement based on object position.
        
        :param object_x: x-coordinate of the detected object
        """
        width = self.camera.getWidth()
        center = width // 2
        
        # Calculate error (deviation from center)
        error = object_x - center
        
        # Proportional control for turning
        turn_rate = error * 0.01
        
        # Set motor speeds
        left_speed = self.MAX_SPEED - turn_rate
        right_speed = self.MAX_SPEED + turn_rate
        
        # Ensure speeds are within limits
        left_speed = max(min(left_speed, self.MAX_SPEED), -self.MAX_SPEED)
        right_speed = max(min(right_speed, self.MAX_SPEED), -self.MAX_SPEED)
        
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
    
    def move_forward(self):
        """
        Move the robot forward at a constant speed.
        """
        self.left_motor.setVelocity(self.MAX_SPEED)
        self.right_motor.setVelocity(self.MAX_SPEED)
    
    def stop(self):
        """
        Stop the robot's movement.
        """
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
    
    def run(self):
        """
        Main control loop for the robot.
        """
        while self.robot.step(self.time_step) != -1:
            # Detect colored object
            object_x = self.detect_colored_object()
            
            if object_x != -1:
                # Object detected, adjust movement towards it
                self.adjust_movement(object_x)
            else:
                # No object, move forward
                self.move_forward()

def main():
    controller = BBAlr8Controller()
    controller.run()

if __name__ == "__main__":
    main()