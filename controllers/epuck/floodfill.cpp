#include "floodfill.hpp"
#include <queue>
#include <utility>
#include <limits>
#include <iomanip> // for setw
#include <iostream>

// Wall direction constants
const int NORTH = 1;
const int EAST = 2;
const int SOUTH = 4;
const int WEST = 8;

Floodfill::Floodfill()
{

    for (int i = 0; i < Config::MAZE_LENGTH; i++)
    {
        for (int j = 0; j < Config::MAZE_WIDTH; j++)
        {
            costsOfCells[i][j] = Config::MAZE_LENGTH * Config::MAZE_WIDTH; // Initialize costs to maximum
        }
    }
}

Floodfill::~Floodfill() {}

bool Floodfill::hasWall(int x, int y, int direction) const {
    if (!isValidCell(x, y)) return true;
    
    // Check if the specified wall exists using bit operations
    // N=1, E=2, S=4, W=8
    return (wallConfig[y][x] & direction) != 0;
}

bool Floodfill::canMove(int fromX, int fromY, int toX, int toY) const
{
    if (!isValidCell(fromX, fromY) || !isValidCell(toX, toY))
        return false;

    // Check if cells are adjacent
    if (abs(fromX - toX) + abs(fromY - toY) != 1)
        return false;

    // Determine direction and check walls
    if (toX > fromX)
    { // Moving East
        return !hasWall(fromX, fromY, 2) && !hasWall(toX, toY, 8);
    }
    if (toX < fromX)
    { // Moving West
        return !hasWall(fromX, fromY, 8) && !hasWall(toX, toY, 2);
    }
    if (toY > fromY)
    { // Moving North
        return !hasWall(fromX, fromY, 1) && !hasWall(toX, toY, 4);
    }
    if (toY < fromY)
    { // Moving South
        return !hasWall(fromX, fromY, 4) && !hasWall(toX, toY, 1);
    }

    return false;
}

void Floodfill::floodMaze(int startX, int startY, int endX, int endY)
{
    // Reset all costs to maximum
    for (int i = 0; i < Config::MAZE_LENGTH; i++)
    {
        for (int j = 0; j < Config::MAZE_WIDTH; j++)
        {
            costsOfCells[i][j] = Config::MAZE_LENGTH * Config::MAZE_WIDTH;
        }
    }

    // Using priority queue for Dijkstra's algorithm
    std::priority_queue<std::pair<int, std::pair<int, int>>,
                        std::vector<std::pair<int, std::pair<int, int>>>,
                        std::greater<std::pair<int, std::pair<int, int>>>>
        pq;

    // Start from the goal
    updateCellCost(endX, endY, 0);
    pq.push({0, {endX, endY}});

    // Possible moves (dx, dy)
    const int moves[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

    while (!pq.empty())
    {
        int cost = pq.top().first;
        int x = pq.top().second.first;
        int y = pq.top().second.second;
        pq.pop();

        // Skip if we found a better path already
        if (cost > getCellCost(x, y))
            continue;

        // Try all possible moves
        for (const auto &move : moves)
        {
            int newX = x + move[0];
            int newY = y + move[1];

            if (canMove(x, y, newX, newY))
            {
                int newCost = cost + 1; // Cost is 1 for each move

                if (newCost < getCellCost(newX, newY))
                {
                    updateCellCost(newX, newY, newCost);
                    pq.push({newCost, {newX, newY}});
                }
            }
        }
    }
}

bool Floodfill::isValidCell(int x, int y) const {
    return x >= 0 && x < Config::MAZE_WIDTH && 
           y >= 0 && y < Config::MAZE_LENGTH;
}

void Floodfill::updateCellCost(int x, int y, int cost) {
    if (isValidCell(x, y)) {
        costsOfCells[y][x] = cost;
    }
}

int Floodfill::getCellCost(int x, int y) const {
    if (!isValidCell(x, y)) return Config::MAZE_LENGTH * Config::MAZE_WIDTH;
    return costsOfCells[y][x];
}

void Floodfill::printCosts() const
{
    std::cout << "Current Flood Fill Costs:" << std::endl;
    std::cout << "   ";
    // Print column headers
    for (int i = 0; i < Config::MAZE_WIDTH; i++)
    {
        std::cout << std::setw(4) << i;
    }
    std::cout << std::endl;

    // Print horizontal line
    std::cout << "   ";
    for (int i = 0; i < Config::MAZE_WIDTH; i++)
    {
        std::cout << "----";
    }
    std::cout << std::endl;

    // Print each row with row number
    for (int y = Config::MAZE_LENGTH - 1; y >= 0; y--)
    { // Print from top to bottom
        std::cout << std::setw(2) << y << "|";
        for (int x = 0; x < Config::MAZE_WIDTH; x++)
        {
            if (costsOfCells[y][x] == std::numeric_limits<int>::max())
            {
                std::cout << std::setw(4) << "-";
            }
            else
            {
                std::cout << std::setw(4) << costsOfCells[y][x];
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void Floodfill::printMazeCell(int cell, bool isTopRow)
{
    // For top border of maze
    if (isTopRow)
    {
        std::cout << "+";
        std::cout << ((cell & NORTH) ? "---" : "   ");
        return;
    }

    // Print west wall
    std::cout << ((cell & WEST) ? "|" : " ");

    // Print cell interior
    std::cout << " . ";
}

void Floodfill::printMaze()
{

    for (int x = 0; x < 10; x++)
    {
        printMazeCell(wallConfig[9][x], true);
    }
    std::cout << "+" << std::endl;

    // Print each row
    for (int y = 9; y >= 0; y--)
    {
        // Print cells in the row
        for (int x = 0; x < 10; x++)
        {
            printMazeCell(wallConfig[y][x]);
        }
        // Print east wall of last cell in row
        std::cout << ((wallConfig[y][9] & EAST) ? "|" : " ") << std::endl;

        // Print south borders for this row
        std::cout << "+";
        for (int x = 0; x < 10; x++)
        {
            std::cout << ((wallConfig[y][x] & SOUTH) ? "---" : "   ") << "+";
        }
        std::cout << std::endl;
    }
}
