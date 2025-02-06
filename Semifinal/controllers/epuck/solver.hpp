#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "floodfill.hpp"  // If needed
#include <utility>       // For std::pair
#include <vector>        // For path representation
#include "config.hpp"
#include "floodfill.hpp"
#include "API.hpp"
#include "epuck.hpp"

class Epuck;

Config::Action NextAction(Epuck& epuck);
void UpdatePosition(Epuck& epuck, Config::Action NextAction);
void updateHeading(Epuck& epuck, Config::Action NextAction);
Config::Action solver(Epuck& epuck);

#endif // SOLVER_HPP //
