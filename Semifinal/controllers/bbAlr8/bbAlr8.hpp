// epuck.hpp
#ifndef EPUCK_HPP
#define EPUCK_HPP

#include <webots/Robot.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>
#include "config.hpp"
// #include "sensor_manager.hpp"
// #include "floodfill.hpp"
// #include "motors.hpp"
// #include "API.hpp"
// #include "solver.hpp"


struct Position {
    int x_mapped, y_mapped;   // Mapped grid coordinates
};

class BbAlr8 : public webots::Robot {
public:
    BbAlr8();
    virtual ~BbAlr8();
    void run();

    void lightEachLEDSequentially();
    // SensorManager sensorManager;
    // Motors motors;
    // Floodfill floodfill;

    // bool reachedColor = false;

    // Config::Heading heading;
    // Position position;

    // bool iswallFront();
    // bool iswallRight();
    // bool iswallLeft();

    // double sensorValues[Config::NUM_SENSORS] = {0};

    // void moveForward(int cells, double *sensorValues);
    //  void turnLeft();
    // void turnRight();
    // void turn180();

    // const unsigned char* getCameraImage();
    // int getCameraWidth() const;
    // int getCameraHeight() const;

private:
    webots::LED *leds[Config::NUM_LEDS];

    webots::Camera *camera;
    
    void initDevices();

};

#endif 