#ifndef API_HPP
#define API_HPP

#include "sensor_manager.hpp"
#include "config.hpp"
#include "epuck.hpp"
#include "epuck.hpp"

//static SensorManager sensorManager;
class Epuck;


bool API_wallFront(Epuck& epuck);
bool API_wallRight(Epuck& epuck);
bool API_wallLeft(Epuck& epuck);
void API_moveForward(Epuck& epuck, double* sensorValues);
void API_turnRight(Epuck& epuck);
void API_turnLeft(Epuck& epuck);

void go(Epuck& epuck, double* sensorValues);



#endif // SOLVER_HPP