#ifndef API_H
#define API_H

#include <string>

bool API_wallFront();
bool API_wallRight(Epuck& epuck);
bool API_wallLeft(Epuck& epuck);
void API_moveForward(Epuck& epuck, double* sensorValues);
void API_turnRight(Epuck& epuck);
void API_turnLeft(Epuck& epuck);

void go(Epuck& epuck, double* sensorValues);

#endif // API_H