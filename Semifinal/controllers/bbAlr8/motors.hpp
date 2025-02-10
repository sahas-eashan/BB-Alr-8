#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include "sensorManager.hpp"
#include "config.hpp"
#include <chrono>

class Motors {
public:
    Motors();
    ~Motors();
    
    void initializeMotors(webots::Robot* robot);
    
    // Basic motor control
    void setSpeed(double leftSpeed, double rightSpeed);

    void turnLeft(webots::Robot *robot);
    void turnRight(webots::Robot *robot);
    void turn180(webots::Robot *robot);
    void moveForward(webots::Robot *robot, SensorManager *sensorManager ,int cells, double *sensorValues);
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