#ifndef EPUCK_HPP
#define EPUCK_HPP

#include <webots/Robot.hpp>
#include <webots/LED.hpp>
#include "sensor_manager.hpp"
#include "motors.hpp"
#include "config.hpp"

class Epuck : public webots::Robot {
public:
    Epuck();
    virtual ~Epuck();
    void run();

private:
    // Components
    SensorManager sensorManager;
    Motors motors;

    webots::LED *leds[Config::NUM_LEDS];
    double sensorValues[Config::NUM_SENSORS] = {0};


    void initDevices();
    void turnLeft();
    void turnRight();
    void turn180();
    void moveForward(int cells, double *sensorValues);
};


#endif