#ifndef LEDMANAGER_HPP
#define LEDMANAGER_HPP

#include <webots/Robot.hpp>
#include <webots/LED.hpp>
#include "config.hpp"

class LEDManager
{
public:
    LEDManager();
    ~LEDManager();
    void initLEDs(webots::Robot& robot);

    void lightGreenOn();
    void lightGreenOff();

    void lightYellowOn();
    void lightYellowOff();

    void lightOrangeOn();
    void lightOrangeOff();

    void lightRedOn();
    void lightRedOff();
    void lightEachLEDSequentially(webots::Robot &robot);

private:
    webots::LED *leds[Config::NUM_LEDS];
};

#endif