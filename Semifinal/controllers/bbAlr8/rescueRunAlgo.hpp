#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <limits>    // For std::numeric_limits
#include <algorithm> // For std::find, std::reverse, std::next_permutation
                     // #include <random>
                     // #include <algorithm>

struct Point
{
    int x, y;
    bool operator==(const Point &other) const
    {
        return x == other.x && y == other.y;
    }
};

struct PathInfo
{
    std::vector<Point> path;
    int distance;
};

class RescueRunAlgo
{
public:
    void findOptimalRoute();
    RescueRunAlgo();
    ~RescueRunAlgo();

private:
    // Maze data and constants
    const int MAZE_S = 20; // MAZE_SIZE
    const int NORTH = 8;   // 1000
    const int SOUTH = 4;   // 0100
    const int EAST = 2;    // 0010
    const int WEST = 1;    // 0001

    // std::vector<Point> redNodes;
    // std::vector<Point> orangeNodes;
    // std::vector<Point> yellowNodes;
    // std::vector<Point> survivors;
    // Point startPoint;

    // Maze data as 2D array
    // North = 8, South = 4, East = 2, West = 1
    const std::vector<std::vector<int>> maze = {
        {5, 4, 12, 12, 6, 5, 6, 5, 12, 12, 14, 13, 4, 12, 6, 5, 4, 12, 4, 6},
        {11, 3, 7, 5, 10, 3, 11, 9, 6, 5, 12, 6, 9, 6, 9, 2, 9, 6, 3, 11},
        {5, 8, 2, 1, 12, 10, 5, 4, 2, 9, 6, 3, 5, 2, 7, 9, 6, 3, 9, 6},
        {11, 5, 10, 9, 12, 6, 11, 1, 8, 12, 10, 1, 10, 11, 1, 4, 10, 11, 5, 10},
        {5, 10, 5, 12, 12, 8, 6, 9, 12, 12, 6, 1, 14, 5, 10, 3, 5, 4, 10, 7},
        {3, 5, 10, 5, 12, 6, 1, 4, 4, 4, 10, 9, 12, 8, 6, 9, 10, 1, 4, 10},
        {3, 1, 14, 3, 7, 9, 0, 2, 1, 8, 12, 12, 14, 5, 2, 7, 7, 1, 10, 7},
        {3, 1, 12, 2, 9, 4, 8, 2, 9, 6, 5, 6, 5, 0, 2, 3, 1, 10, 13, 2},
        {1, 2, 13, 10, 5, 10, 5, 8, 6, 3, 3, 1, 10, 11, 1, 2, 1, 12, 12, 2},
        {1, 8, 6, 5, 10, 13, 0, 14, 1, 2, 3, 11, 5, 12, 8, 8, 8, 6, 5, 2},
        {9, 6, 3, 9, 12, 6, 11, 5, 0, 2, 1, 6, 9, 12, 12, 12, 6, 3, 11, 3},
        {7, 3, 3, 5, 6, 9, 12, 10, 3, 9, 10, 1, 12, 4, 4, 6, 9, 10, 5, 10},
        {3, 3, 3, 3, 3, 5, 6, 13, 10, 13, 6, 3, 5, 2, 11, 3, 5, 12, 8, 6},
        {9, 10, 3, 3, 9, 10, 9, 4, 12, 12, 10, 1, 8, 10, 5, 0, 2, 5, 6, 3},
        {5, 12, 10, 9, 4, 4, 14, 9, 12, 6, 5, 10, 5, 4, 8, 2, 1, 10, 9, 10},
        {1, 6, 5, 4, 8, 2, 7, 5, 6, 3, 3, 5, 10, 1, 6, 1, 8, 6, 13, 6},
        {3, 6, 10, 5, 12, 12, 2, 3, 3, 3, 3, 1, 6, 1, 0, 2, 5, 0, 4, 2},
        {3, 1, 12, 10, 7, 5, 8, 10, 9, 12, 8, 12, 8, 2, 1, 6, 1, 14, 6, 14},
        {1, 8, 12, 12, 10, 9, 6, 5, 6, 5, 4, 6, 5, 8, 10, 3, 9, 6, 9, 6},
        {9, 12, 12, 12, 12, 12, 8, 8, 8, 10, 9, 8, 10, 13, 12, 8, 12, 8, 12, 10}};

    std::vector<Point> redNodes = {{7, 3}, {9, 15}, {15, 15}};
    std::vector<Point> orangeNodes = {{6, 2}, {6, 3}, {6, 4}, {7, 2}, {7, 4}, {8, 2}, {8, 3}, {8, 4}, {8, 14}, {8, 15}, {8, 16}, {9, 14}, {9, 16}, {10, 14}, {10, 15}, {10, 16}, {14, 14}, {14, 15}, {14, 16}, {15, 14}, {15, 16}, {16, 14}, {16, 15}, {16, 16}};
    std::vector<Point> yellowNodes = {};
    std::vector<Point> survivors = {{6, 19}, {17, 18}, {11, 0}};
    Point startPoint = {0, 10};

    PathInfo findShortestPath(Point start, Point end);
    bool canMove(int x, int y, int direction);
    std::vector<std::vector<int>> calculateAllPairDistances();
    PathInfo solveTSP(const std::vector<std::vector<int>> &distances);
    void log(const std::string &text);
};