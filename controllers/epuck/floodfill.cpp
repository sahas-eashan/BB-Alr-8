#include "floodfill.hpp"
#include <queue>
#include <utility>
#include <limits>

// Wall direction constants
const int NORTH = 1;
const int EAST = 2;
const int SOUTH = 4;
const int WEST = 8;

Floodfill::Floodfill() {

    for (int i = 0; i < Config::MAZE_LENGTH; i++) {
        for (int j = 0; j < Config::MAZE_WIDTH; j++) {
            wallConfig[i][j] = 0;  // No walls initially
            costsOfCells[i][j] = std::numeric_limits<int>::max();  // Initialize costs to maximum
        }
    }
}

Floodfill::~Floodfill() {}

void Floodfill::setWall(int x, int y, int direction) {
    if (!isValidCell(x, y)) return;
    
    wallConfig[x][y] |= direction;
    
    // Set wall for adjacent cell
    if (direction == NORTH && isValidCell(x, y+1))
        wallConfig[x][y+1] |= SOUTH;
    else if (direction == SOUTH && isValidCell(x, y-1))
        wallConfig[x][y-1] |= NORTH;
    else if (direction == EAST && isValidCell(x+1, y))
        wallConfig[x+1][y] |= WEST;
    else if (direction == WEST && isValidCell(x-1, y))
        wallConfig[x-1][y] |= EAST;
}

bool Floodfill::hasWall(int x, int y, int direction) const {
    if (!isValidCell(x, y)) return true;  // Treat out-of-bounds as walls
    return (wallConfig[x][y] & direction) != 0;
}

void Floodfill::floodMaze(int startX, int startY, int endX, int endY) {
    // Reset costs
    for (int i = 0; i < Config::MAZE_LENGTH; i++) {
        for (int j = 0; j < Config::MAZE_WIDTH; j++) {
            costsOfCells[i][j] = std::numeric_limits<int>::max();
        }
    }

    // Queue for flooding: {x, y, cost}
    std::queue<std::pair<std::pair<int, int>, int>> q;
    
    // Start from goal
    q.push({{endX, endY}, 0});
    costsOfCells[endX][endY] = 0;

    while (!q.empty()) {
        auto current = q.front();
        q.pop();
        
        int x = current.first.first;
        int y = current.first.second;
        int currentCost = current.second;

        // Check all four directions
        if (!hasWall(x, y, NORTH) && isValidCell(x, y+1)) {
            if (costsOfCells[x][y+1] > currentCost + 1) {
                costsOfCells[x][y+1] = currentCost + 1;
                q.push({{x, y+1}, currentCost + 1});
            }
        }
        
        if (!hasWall(x, y, SOUTH) && isValidCell(x, y-1)) {
            if (costsOfCells[x][y-1] > currentCost + 1) {
                costsOfCells[x][y-1] = currentCost + 1;
                q.push({{x, y-1}, currentCost + 1});
            }
        }
        
        if (!hasWall(x, y, EAST) && isValidCell(x+1, y)) {
            if (costsOfCells[x+1][y] > currentCost + 1) {
                costsOfCells[x+1][y] = currentCost + 1;
                q.push({{x+1, y}, currentCost + 1});
            }
        }
        
        if (!hasWall(x, y, WEST) && isValidCell(x-1, y)) {
            if (costsOfCells[x-1][y] > currentCost + 1) {
                costsOfCells[x-1][y] = currentCost + 1;
                q.push({{x-1, y}, currentCost + 1});
            }
        }
    }
}

int Floodfill::getCellCost(int x, int y) const {
    if (!isValidCell(x, y)) return -1;  // Invalid cell
    return costsOfCells[x][y];
}

bool Floodfill::isValidCell(int x, int y) const {
    return x >= 0 && x < Config::MAZE_LENGTH && y >= 0 && y < Config::MAZE_WIDTH;
}