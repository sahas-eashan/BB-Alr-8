#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <limits>    // For std::numeric_limits
#include <algorithm> // For std::find, std::reverse, std::next_permutation
#include <array>
struct Point
{
    int x, y;
    bool operator==(const Point &other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator<(const Point &other) const
    {
        if (x != other.x)
        {
            return x < other.x;
        }
        return y < other.y;
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
    // Maze data and constants
    const int MAZE_S = 20; // MAZE_SIZE
    const int NORTH = 8;   // 1000
    const int SOUTH = 4;   // 0100
    const int EAST = 2;    // 0010
    const int WEST = 1;    // 0001

    void findOptimalRoute();
    RescueRunAlgo();
    ~RescueRunAlgo();
    void setMaze(std::array<std::array<unsigned int, 20>, 20> &nesw_maze);
    void setDefaults();
    void setRedNodes(const std::vector<Point> &redNodes);
    void setOrangeNodes(const std::vector<Point> &orangeNodes);
    void setYellowNodes(const std::vector<Point> &yellowNodes);
    void setStartPoint(const Point &startPoint);
    void setSurvivors(const std::vector<Point> &survivors);

    // Add getter for the calculated path
    const std::vector<Point> &getOptimalPath() const { return optimalPath; }
    bool hasPathCalculated() const { return !optimalPath.empty(); }

    // Add movement commands structure
    enum class Command
    {
        MOVE_FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_180,
        WAIT
    };

    struct Movement
    {
        Command command;
        Point nextPosition;
    };

    Movement getNextMovement(const Point &currentPos, const Point &nextPos, int currentHeading);

    std::vector<std::vector<int>> maze = {}; // maze[y][x]
    std::vector<Point> redNodes = {};        // {y, x}
    std::vector<Point> orangeNodes = {};
    std::vector<Point> yellowNodes = {};
    std::vector<Point> survivors = {};

    Point startPoint = {10, 0};

private:
    std::vector<Point> optimalPath;

    std::vector<std::vector<int>> convertNESWtoNSEW(const std::vector<std::vector<int>> &neswMatrix);
    PathInfo findShortestPath(Point start, Point end);
    bool canMove(int x, int y, int direction);
    std::vector<std::vector<int>> calculateAllPairDistances();
    PathInfo solveTSP(const std::vector<std::vector<int>> &distances);
    void log(const std::string &text);
    void printNodes(const std::vector<Point> &nodes, const std::string &color);
};