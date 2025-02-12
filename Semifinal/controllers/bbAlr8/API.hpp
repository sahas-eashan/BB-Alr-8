#ifndef API_H
#define API_H

#include <string>
#include "bbAlr8.hpp"
#include "sensorManager.hpp"
#include "motors.hpp"
#include "solver.hpp"

class BbAlr8;

void API_moveForward();
void API_turnLeft();
void API_turnRight();
void API_turn180();


bool API_wallFront();
bool API_wallRight();
bool API_wallLeft();

void exploreMaze();


#endif // API_H