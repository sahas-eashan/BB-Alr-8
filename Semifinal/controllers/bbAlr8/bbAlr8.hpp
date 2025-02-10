#ifndef BBALR8_HPP
#define BBALR8_HPP

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include "ledManager.hpp"
#include "config.hpp"
#include "sensorManager.hpp"
#include "motors.hpp"
#include "CameraController.hpp"
// #include "floodfill.hpp"

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

    LEDManager leds;
    SensorManager sensorManager;
    Motors motors;

    char floorColor();
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

    CameraController cameraController;
    void initDevices();

};

#endif 