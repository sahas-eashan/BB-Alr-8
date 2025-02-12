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
    CameraController cameraController;

    char floorColor();


private:
    void initDevices();

};

#endif 