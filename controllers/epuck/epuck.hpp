#ifndef EPUCK_HPP
#define EPUCK_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include "sensor_manager.hpp"
#include "config.hpp"

class Epuck : public webots::Robot {
public:
    Epuck();
    virtual ~Epuck();
    void run();

private:
    // Components
    SensorManager sensorManager;
    
    // Hardware
    webots::Motor *leftMotor;
    webots::Motor *rightMotor;
    webots::LED *leds[Config::Epuck::NUM_LEDS];

    // PID control
    bool pidEnabled;
    float lastError;
    float Kp;
    float Kd;

    // Helper functions
    void initDevices();
    void setMotorSpeeds(double leftSpeed, double rightSpeed);
    void togglePID();
    float calculatePID(float leftValue, float rightValue);
};

#endif