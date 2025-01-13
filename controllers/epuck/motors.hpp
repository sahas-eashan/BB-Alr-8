#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include "sensor_manager.hpp"
#include "config.hpp"
#include <chrono>

class Motors {
public:
    Motors();
    ~Motors();
    
    void initializeMotors(webots::Robot* robot);
    
    // Basic motor control
    void setSpeed(double leftSpeed, double rightSpeed);
    void stop();
    
    void delay(int ms);

private:
    webots::Motor* leftMotor;
    webots::Motor* rightMotor; 
    
};

template <typename T>
T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

#endif