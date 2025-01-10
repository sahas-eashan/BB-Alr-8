#ifndef MY_ROBOT_HPP
#define MY_ROBOT_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>

class MyRobot : public webots::Robot {
public:
    MyRobot();
    virtual ~MyRobot();
    void run();

private:
    // Time step in milliseconds
    static const int TIME_STEP = 64;
    
    // Device related constants
    static const int NUM_SENSORS = 8;
    static const int NUM_LEDS = 10;
    static const double MAX_SPEED = 6.28;  // rad/s
    
    // Device pointers
    webots::Motor *leftMotor;
    webots::Motor *rightMotor;
    webots::DistanceSensor *distanceSensors[NUM_SENSORS];
    webots::LED *leds[NUM_LEDS];
    
    // Helper functions
    void initDevices();
    void readSensors(double* sensorValues);
    void setMotorSpeeds(double leftSpeed, double rightSpeed);
};

#endif