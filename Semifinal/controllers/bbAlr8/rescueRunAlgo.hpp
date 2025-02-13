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
    void setMaze(const vector<vector<int>> &maze);
    void setDefaults();
    void setRedNodes(const vector<Point> &redNodes);
    void setOrangeNodes(const vector<Point> &orangeNodes);
    void setYellowNodes(const vector<Point> &yellowNodes);
    void setStartPoint(const Point &startPoint);
    void setSurvivors(const vector<Point> &survivors);

    // Add getter for the calculated path
    const std::vector<Point> &getOptimalPath() const { return optimalPath; }
    bool hasPathCalculated() const { return !optimalPath.empty(); }

    // Add movement commands structure
    enum class Command
    {
        MOVE_FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_180
    };

    struct Movement
    {
        Command command;
        Point nextPosition;
    };

    Movement getNextMovement(const Point &currentPos, const Point &nextPos, int currentHeading) const;

private:
    std::vector<Point> optimalPath;
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
    // This maze represent NESW , we will convert it to NSEW later
    // North = 8, South = 2, East = 4, West = 1
    const std::vector<std::vector<int>> maze = {
        {3, 2, 10, 10, 6, 3, 6, 3, 10, 10, 14, 11, 2, 10, 6, 3, 2, 10, 2, 6},
        {13, 5, 7, 3, 12, 5, 13, 9, 6, 3, 10, 6, 9, 6, 9, 4, 9, 6, 5, 13},
        {3, 8, 4, 1, 10, 12, 3, 2, 4, 9, 6, 5, 3, 4, 7, 9, 6, 5, 9, 6},
        {13, 3, 12, 9, 10, 6, 13, 1, 8, 10, 12, 1, 12, 13, 1, 2, 12, 13, 3, 12},
        {3, 12, 3, 10, 10, 8, 6, 9, 10, 10, 6, 1, 14, 3, 12, 5, 3, 2, 12, 7},
        {5, 3, 12, 3, 10, 6, 1, 2, 2, 2, 12, 9, 10, 8, 6, 9, 12, 1, 2, 12},
        {5, 1, 14, 5, 7, 9, 0, 4, 1, 8, 10, 10, 14, 3, 4, 7, 7, 1, 12, 7},
        {5, 1, 10, 4, 9, 2, 8, 4, 9, 6, 3, 6, 3, 0, 4, 5, 1, 12, 11, 4},
        {1, 4, 11, 12, 3, 12, 3, 8, 6, 5, 5, 1, 12, 13, 1, 4, 1, 10, 10, 4},
        {1, 8, 6, 3, 12, 11, 0, 14, 1, 4, 5, 13, 3, 10, 8, 8, 8, 6, 3, 4},
        {9, 6, 5, 9, 10, 6, 13, 3, 0, 4, 1, 6, 9, 10, 10, 10, 6, 5, 13, 5},
        {7, 5, 5, 3, 6, 9, 10, 12, 5, 9, 12, 1, 10, 2, 2, 6, 9, 12, 3, 12},
        {5, 5, 5, 5, 5, 3, 6, 11, 12, 11, 6, 5, 3, 4, 13, 5, 3, 10, 8, 6},
        {9, 12, 5, 5, 9, 12, 9, 2, 10, 10, 12, 1, 8, 12, 3, 0, 4, 3, 6, 5},
        {3, 10, 12, 9, 2, 2, 14, 9, 10, 6, 3, 12, 3, 2, 8, 4, 1, 12, 9, 12},
        {1, 6, 3, 2, 8, 4, 7, 3, 6, 5, 5, 3, 12, 1, 6, 1, 8, 6, 11, 6},
        {5, 6, 12, 3, 10, 10, 4, 5, 5, 13, 5, 9, 6, 1, 0, 12, 3, 8, 10, 12},
        {5, 1, 10, 12, 7, 3, 8, 12, 9, 10, 8, 10, 8, 4, 1, 6, 1, 14, 6, 14},
        {1, 8, 10, 10, 12, 9, 6, 3, 6, 3, 2, 6, 3, 8, 12, 5, 9, 6, 9, 6},
        {9, 10, 10, 10, 10, 10, 8, 8, 8, 12, 9, 8, 12, 11, 10, 8, 10, 8, 10, 12}};

    std::vector<Point> redNodes = {{3, 7}, {15, 9}, {15, 15}};
    std::vector<Point> orangeNodes = {{2, 6}, {3, 6}, {4, 6}, {2, 7}, {4, 7}, {2, 8}, {3, 8}, {4, 8}, {14, 8}, {15, 8}, {16, 8}, {14, 9}, {16, 9}, {14, 10}, {15, 10}, {16, 10}, {14, 14}, {15, 14}, {16, 14}, {14, 15}, {16, 15}, {14, 16}, {15, 16}, {16, 16}};
    std::vector<Point> yellowNodes = {};
    std::vector<Point> survivors = {{19, 6}, {18, 17}, {0, 11}};
    Point startPoint = {10, 0};

    PathInfo findShortestPath(Point start, Point end);
    bool canMove(int x, int y, int direction);
    std::vector<std::vector<int>> calculateAllPairDistances();
    PathInfo solveTSP(const std::vector<std::vector<int>> &distances);
    void log(const std::string &text);
};