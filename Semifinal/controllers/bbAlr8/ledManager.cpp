#include "ledManager.hpp"

LEDManager::LEDManager() {}
LEDManager::~LEDManager() {}

void LEDManager::initLEDs(webots::Robot &robot)
{
    char ledName[5];
    for (int i = 1; i <= Config::NUM_LEDS; i++)
    {
        sprintf(ledName, "led%d", i);
        leds[i - 1] = robot.getLED(ledName);
    }
}

void LEDManager::lightGreenOn()
{
    leds[0]->set(1);
}

void LEDManager::lightGreenOff()
{
    leds[0]->set(0);
}

void LEDManager::lightOrangeOn()
{
    leds[1]->set(1);
}

void LEDManager::lightOrangeOff()
{
    leds[1]->set(0);
}

void LEDManager::lightYellowOn()
{
    leds[2]->set(1);
}

void LEDManager::lightYellowOff()
{
    leds[2]->set(0);
}

void LEDManager::lightRedOn()
{
    leds[3]->set(1);
}

void LEDManager::lightRedOff()
{
    leds[3]->set(0);
}

// In the end and start. for indicating
void LEDManager::lightEachLEDSequentially(webots::Robot &robot)
{
    // Turn on each LED one by one
    for (int i = 1; i <= Config::NUM_LEDS; i++)
    {
        leds[i - 1]->set(1);         // Turn ON the LED
        robot.step(3 * Config::TIME_STEP); // Wait for some time
    }

    // Turn off each LED one by one
    for (int i = 0; i <= Config::NUM_LEDS; i++)
    {
        leds[i - 1]->set(0);         // Turn OFF the LED
        robot.step(3 * Config::TIME_STEP); // Wait for some time
    }
}