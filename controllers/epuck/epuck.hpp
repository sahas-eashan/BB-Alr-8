// epuck.hpp
#ifndef EPUCK_HPP
#define EPUCK_HPP

#include <webots/Robot.hpp>
#include <webots/LED.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include "sensor_manager.hpp"
#include "motors.hpp"
#include "config.hpp"

struct Position {
    int x_mapped, y_mapped;   // Mapped grid coordinates
};

class Epuck : public webots::Supervisor {
public:
    Epuck();
    virtual ~Epuck();
    void run();
    Position recordOwnPosition();
    
private:
    // Components
    SensorManager sensorManager;
    Motors motors;
    webots::LED *leds[Config::NUM_LEDS];
    double sensorValues[Config::NUM_SENSORS] = {0};
    webots::Node *selfNode;  // Added to store robot node reference

    void initDevices();
    void turnLeft();
    void turnRight();
    void turn180();
    void moveForward(int cells, double *sensorValues);
};

#endif 