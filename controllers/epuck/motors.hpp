#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "config.hpp"

class Motors {
public:
    Motors();
    ~Motors();
    
    void initializeMotors(webots::Robot* robot);
    
    // Basic motor control
    void setSpeed(double leftSpeed, double rightSpeed);
    void stop();
    
    // Position-based movement
    void moveForward(double distance);  // distance in meters
    void turnRight();  // 90 degrees right turn
    void turnLeft();   // 90 degrees left turn
    void turn180();    // 180 degrees turn
    
    // PID-controlled wall following
    void applyPIDCorrection(double correction);
    
    // Status checks
    bool isMoving() const;
    double getDistanceTraveled() const;
    double getLeftPosition() const;
    double getRightPosition() const;

private:
    webots::Motor* leftMotor;
    webots::Motor* rightMotor;
    
    // Position tracking
    double previousLeftPosition;
    double previousRightPosition;
    double distanceTraveled;
    
    // Movement control
    void setPosition(double leftPosition, double rightPosition);
    void waitForMotorsToStop();
    double convertDistanceToRadians(double distance) const;
    void resetPositions();
};

template <typename T>
T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}


#endif